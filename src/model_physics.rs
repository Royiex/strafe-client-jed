use crate::integer::{Planar64,Planar64Vec3};
use std::borrow::{Borrow,Cow};

#[derive(Debug,Clone,Copy,Hash,Eq,PartialEq)]
pub struct VertId(usize);
#[derive(Debug,Clone,Copy,Hash,Eq,PartialEq)]
pub struct EdgeId(usize);
impl EdgeId{
	fn as_directed_edge_id(&self,parity:bool)->DirectedEdgeId{
		DirectedEdgeId(self.0|((parity as usize)<<(usize::BITS-1)))
	}
}
/// DirectedEdgeId refers to an EdgeId when undirected.
#[derive(Debug,Clone,Copy,Hash,Eq,PartialEq)]
pub struct DirectedEdgeId(usize);
impl DirectedEdgeId{
	fn as_edge_id(&self)->EdgeId{
		EdgeId(self.0&!(1<<(usize::BITS-1)))
	}
	fn signum(&self)->isize{
		((self.0&(1<<(usize::BITS-1))!=0) as isize)*2-1
	}
}
#[derive(Debug,Clone,Copy,Hash,Eq,PartialEq)]
pub struct FaceId(usize);

//Vertex <-> Edge <-> Face -> Collide
pub enum FEV<F,E,V>{
	Face(F),
	Edge(E),
	Vert(V),
}

//use Unit32 #[repr(C)] for map files
struct Face{
	normal:Planar64Vec3,
	dot:Planar64,
}
impl Face{
	fn nd(&self)->(Planar64Vec3,Planar64){
		(self.normal,self.dot)
	}
}
struct Vert(Planar64Vec3);
struct FaceRefs{
	edges:Vec<(EdgeId,FaceId)>,
	//verts:Vec<VertId>,
}
struct EdgeRefs{
	faces:[FaceId;2],//left, right
	verts:[VertId;2],//bottom, top
}
struct VertRefs{
	faces:Vec<FaceId>,
	edges:Vec<DirectedEdgeId>,
}
pub struct PhysicsMesh{
	faces:Vec<Face>,
	verts:Vec<Vert>,
	face_topology:Vec<FaceRefs>,
	edge_topology:Vec<EdgeRefs>,
	vert_topology:Vec<VertRefs>,
}

#[derive(Default,Clone)]
struct VertRefGuy{
	edges:std::collections::HashSet<DirectedEdgeId>,
	faces:std::collections::HashSet<FaceId>,
}
#[derive(Clone,Hash,Eq,PartialEq)]
struct EdgeIdGuy([VertId;2]);
impl EdgeIdGuy{
	fn new(v0:VertId,v1:VertId)->(Self,bool){
		(if v0.0<v1.0{
			Self([v0,v1])
		}else{
			Self([v1,v0])
		},v0.0<v1.0)
	}
}
struct EdgeRefGuy([FaceId;2]);
impl EdgeRefGuy{
	fn new()->Self{
		Self([FaceId(0);2])
	}
	fn push(&mut self,i:usize,face_id:FaceId){
		self.0[i]=face_id;
	}
}
struct FaceRefGuy(Vec<EdgeId>);
#[derive(Default)]
struct EdgePool{
	edge_guys:Vec<(EdgeIdGuy,EdgeRefGuy)>,
	edge_id_from_guy:std::collections::HashMap<EdgeIdGuy,usize>,
}
impl EdgePool{
	fn push(&mut self,edge_id_guy:EdgeIdGuy)->(&mut EdgeRefGuy,EdgeId){
		let edge_id=if let Some(&edge_id)=self.edge_id_from_guy.get(&edge_id_guy){
			edge_id
		}else{
			let edge_id=self.edge_guys.len();
			self.edge_guys.push((edge_id_guy.clone(),EdgeRefGuy::new()));
			self.edge_id_from_guy.insert(edge_id_guy,edge_id);
			edge_id
		};
		(&mut unsafe{self.edge_guys.get_unchecked_mut(edge_id)}.1,EdgeId(edge_id))
	}
}
impl From<&crate::model::IndexedModel> for PhysicsMesh{
	fn from(indexed_model:&crate::model::IndexedModel)->Self{
		let verts=indexed_model.unique_pos.iter().map(|v|Vert(v.clone())).collect();
		let mut vert_ref_guys=vec![VertRefGuy::default();indexed_model.unique_pos.len()];
		let mut edge_pool=EdgePool::default();
		let mut face_i=0;
		let mut faces=Vec::new();
		let mut face_ref_guys=Vec::new();
		for group in indexed_model.groups.iter(){for poly in group.polys.iter(){
			let face_id=FaceId(face_i);
			//one face per poly
			let mut normal=Planar64Vec3::ZERO;
			let len=poly.vertices.len();
			let face_edges=poly.vertices.iter().enumerate().map(|(i,&vert_id)|{
				let vert0_id=indexed_model.unique_vertices[vert_id as usize].pos as usize;
				let vert1_id=indexed_model.unique_vertices[poly.vertices[(i+1)%len] as usize].pos as usize;
				//https://www.khronos.org/opengl/wiki/Calculating_a_Surface_Normal (Newell's Method)
				let v0=indexed_model.unique_pos[vert0_id];
				let v1=indexed_model.unique_pos[vert1_id];
				normal+=Planar64Vec3::new(
					(v0.y()-v1.y())*(v0.z()+v1.z()),
					(v0.z()-v1.z())*(v0.x()+v1.x()),
					(v0.x()-v1.x())*(v0.y()+v1.y()),
				);
				//get/create edge and push face into it
				let (edge_id_guy,is_sorted)=EdgeIdGuy::new(VertId(vert0_id),VertId(vert1_id));
				let (edge_ref_guy,edge_id)=edge_pool.push(edge_id_guy);
				//polygon vertices as assumed to be listed clockwise
				//populate the edge face on the left or right depending on how the edge vertices got sorted
				edge_ref_guy.push(is_sorted as usize,face_id);
				//index edges & face into vertices
				{
					let vert_ref_guy=unsafe{vert_ref_guys.get_unchecked_mut(vert0_id)};
					vert_ref_guy.edges.insert(edge_id.as_directed_edge_id(!is_sorted));
					vert_ref_guy.faces.insert(face_id);
					unsafe{vert_ref_guys.get_unchecked_mut(vert1_id)}.edges.insert(edge_id.as_directed_edge_id(is_sorted));
				}
				//return edge_id
				edge_id
			}).collect();
			//choose precision loss randomly idk
			normal=normal/len as i64;
			let mut dot=Planar64::ZERO;
			for &v in poly.vertices.iter(){
				dot+=normal.dot(indexed_model.unique_pos[indexed_model.unique_vertices[v as usize].pos as usize]);
			}
			faces.push(Face{normal,dot:dot/len as i64});
			face_ref_guys.push(FaceRefGuy(face_edges));
			face_i+=1;
		}}
		//conceivably faces, edges, and vertices exist now
		Self{
			faces,
			verts,
			face_topology:face_ref_guys.into_iter().enumerate().map(|(i,face_ref_guy)|{
				let face_id=FaceId(i);
				FaceRefs{edges:face_ref_guy.0.into_iter().map(|edge_id|{
					//get the edge face that's not this face
					let edge_faces=&edge_pool.edge_guys[edge_id.0].1.0;
					if edge_faces[0]==face_id{
						(edge_id,edge_faces[1])
					}else if edge_faces[1]==face_id{
						(edge_id,edge_faces[0])
					}else{
						panic!("edge does not contain face edge_faces={:?} face={:?}",edge_faces,face_id)
					}
				}).collect()}
			}).collect(),
			edge_topology:edge_pool.edge_guys.into_iter().map(|(edge_id_guy,edge_ref_guy)|
				EdgeRefs{faces:edge_ref_guy.0,verts:edge_id_guy.0}
			).collect(),
			vert_topology:vert_ref_guys.into_iter().map(|vert_ref_guy|
				VertRefs{
					edges:vert_ref_guy.edges.into_iter().collect(),
					faces:vert_ref_guy.faces.into_iter().collect(),
				}
			).collect(),
		}
	}
}

pub trait MeshQuery<FACE:Clone,EDGE:Clone,VERT:Clone>{
	fn closest_fev(&self,point:Planar64Vec3)->FEV<FACE,EDGE,VERT>;
	fn edge_n(&self,edge_id:EDGE)->Planar64Vec3{
		let verts=self.edge_verts(edge_id);
		self.vert(verts[1].clone())-self.vert(verts[0].clone())
	}
	fn vert(&self,vert_id:VERT)->Planar64Vec3;
	fn face_nd(&self,face_id:FACE)->(Planar64Vec3,Planar64);
	fn face_edges(&self,face_id:FACE)->Cow<Vec<(EDGE,FACE)>>;
	fn edge_faces(&self,edge_id:EDGE)->Cow<[FACE;2]>;
	fn edge_verts(&self,edge_id:EDGE)->Cow<[VERT;2]>;
	fn vert_edges(&self,vert_id:VERT)->Cow<Vec<EDGE>>;
	fn vert_faces(&self,vert_id:VERT)->Cow<Vec<FACE>>;
}
impl PhysicsMesh{
	pub fn verts<'a>(&'a self)->impl Iterator<Item=Planar64Vec3>+'a{
		self.verts.iter().map(|Vert(pos)|*pos)
	}
	pub fn brute(&self,body:&crate::physics::Body,time_limit:crate::integer::Time)->Option<(FaceId,crate::integer::Time)>{
		//check each face
		let mut best_time=time_limit;
		let mut best_face=None;
		for (i,face) in self.faces.iter().enumerate(){
			let face_id=FaceId(i);
			let (n,d)=face.nd();
			for t in crate::zeroes::zeroes2((n.dot(body.position)-d)*2,n.dot(body.velocity)*2,n.dot(body.acceleration)){
				let t=body.time+crate::integer::Time::from(t);
				if body.time<t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
					let p=body.extrapolated_position(t);
					if self.face_edges(face_id).iter().all(|&(_,face_id)|{
						let (n,d)=self.face_nd(face_id);
						n.dot(p)<=d
					}){
						best_time=t;
						best_face=Some(face_id);
					}
				}
			}
		}
		best_face.map(|f|(f,best_time))
	}
	fn vert_directed_edges(&self,vert_id:VertId)->Cow<Vec<DirectedEdgeId>>{
		Cow::Borrowed(&self.vert_topology[vert_id.0].edges)
	}
	fn directed_edge_n(&self,directed_edge_id:DirectedEdgeId)->Planar64Vec3{
		let verts=self.edge_verts(directed_edge_id.as_edge_id());
		(self.vert(verts[1].clone())-self.vert(verts[0].clone()))*(directed_edge_id.signum() as i64)
	}
}
impl MeshQuery<FaceId,EdgeId,VertId> for PhysicsMesh{
	fn closest_fev(&self,point:Planar64Vec3)->FEV<FaceId,EdgeId,VertId>{
		//TODO: put some genius code right here

		//brute force for now
		let mut best_distance_squared=Planar64::MAX;
		//make something up as default ret
		//hopefully empty meshes don't make their way through here
		let mut best_fev=FEV::<FaceId,EdgeId,VertId>::Vert(VertId(0));
		//check each vert
		for (i,v) in self.verts.iter().enumerate(){
			let d=(v.0-point).dot(v.0-point);
			if d<best_distance_squared{
				best_distance_squared=d;
				best_fev=FEV::<FaceId,EdgeId,VertId>::Vert(VertId(i));
			}
		}
		//check each edge
		for (i,e) in self.edge_topology.iter().enumerate(){
			let v0=self.vert(e.verts[0]);
			let v1=self.vert(e.verts[1]);
			let n=v1-v0;
			//n.cross(point-v0)=sin(t)*n*dis
			let d=n.dot(point-v0);
			if d<n.dot(v1)&&n.dot(v0)<d{
				let c=n.cross(point-v0);
				let edge_distance_squared=c.dot(c)/n.dot(n);
				if edge_distance_squared<best_distance_squared{
					best_distance_squared=edge_distance_squared;
					best_fev=FEV::<FaceId,EdgeId,VertId>::Edge(EdgeId(i));
				}
			}
		}
		let face_dots:Vec<Planar64>=self.faces.iter().map(|f|f.normal.dot(point)).collect();
		//check each face
		for (i,f) in self.face_topology.iter().enumerate(){
			if face_dots[i]<best_distance_squared&&f.edges.iter().all(|&(_,face_id)|face_dots[face_id.0]<=Planar64::ZERO){
				best_distance_squared=face_dots[i];
				best_fev=FEV::<FaceId,EdgeId,VertId>::Face(FaceId(i));
			}
		}
		best_fev
	}
	fn face_nd(&self,face_id:FaceId)->(Planar64Vec3,Planar64){
		(self.faces[face_id.0].normal,self.faces[face_id.0].dot)
	}
	//ideally I never calculate the vertex position, but I have to for the graphical meshes...
	fn vert(&self,vert_id:VertId)->Planar64Vec3{
		self.verts[vert_id.0].0
	}
	fn face_edges(&self,face_id:FaceId)->Cow<Vec<(EdgeId,FaceId)>>{
		Cow::Borrowed(&self.face_topology[face_id.0].edges)
	}
	fn edge_faces(&self,edge_id:EdgeId)->Cow<[FaceId;2]>{
		Cow::Borrowed(&self.edge_topology[edge_id.0].faces)
	}
	fn edge_verts(&self,edge_id:EdgeId)->Cow<[VertId;2]>{
		Cow::Borrowed(&self.edge_topology[edge_id.0].verts)
	}
	fn vert_edges(&self,vert_id:VertId)->Cow<Vec<EdgeId>>{
		//not poggers
		Cow::Owned(self.vert_topology[vert_id.0].edges.iter().map(|directed_edge_id|directed_edge_id.as_edge_id()).collect())
	}
	fn vert_faces(&self,vert_id:VertId)->Cow<Vec<FaceId>>{
		Cow::Borrowed(&self.vert_topology[vert_id.0].faces)
	}
}

pub struct TransformedMesh<'a>{
	mesh:&'a PhysicsMesh,
	transform:&'a crate::integer::Planar64Affine3,
	normal_transform:&'a crate::integer::Planar64Mat3,
}
impl TransformedMesh<'_>{
	pub fn new<'a>(
		mesh:&'a PhysicsMesh,
		transform:&'a crate::integer::Planar64Affine3,
		normal_transform:&'a crate::integer::Planar64Mat3,
		)->TransformedMesh<'a>{
		TransformedMesh{
			mesh,
			transform,
			normal_transform,
		}
	}
	pub fn brute_in(&self,body:&crate::physics::Body,time_limit:crate::integer::Time)->Option<(FaceId,crate::integer::Time)>{
		//check each face
		let mut best_time=time_limit;
		let mut best_face=None;
		for i in 0..self.mesh.faces.len(){
			let face_id=FaceId(i);
			let (n,d)=self.face_nd(face_id);
			for t in crate::zeroes::zeroes2((n.dot(body.position)-d)*2,n.dot(body.velocity)*2,n.dot(body.acceleration)){
				let t=body.time+crate::integer::Time::from(t);
				if body.time<t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
					let p=body.extrapolated_position(t);
					if self.face_edges(face_id).iter().all(|&(_,face_id)|{
						let (n,d)=self.face_nd(face_id);
						n.dot(p)<=d
					}){
						best_time=t;
						best_face=Some(face_id);
					}
				}
			}
		}
		best_face.map(|f|(f,best_time))
	}
	pub fn brute_out(&self,body:&crate::physics::Body,time_limit:crate::integer::Time)->Option<(FaceId,crate::integer::Time)>{
		//check each face
		let mut best_time=time_limit;
		let mut best_face=None;
		for i in 0..self.mesh.faces.len(){
			let face_id=FaceId(i);
			let (n,d)=self.face_nd(face_id);
			for t in crate::zeroes::zeroes2((n.dot(body.position)-d)*2,n.dot(body.velocity)*2,n.dot(body.acceleration)){
				let t=body.time+crate::integer::Time::from(t);
				if body.time<t&&t<best_time&&n.dot(body.extrapolated_velocity(t))>Planar64::ZERO{
					let p=body.extrapolated_position(t);
					if self.face_edges(face_id).iter().all(|&(_,test_face_id)|{
						let (n,d)=self.face_nd(test_face_id);
						n.dot(p)<=d
					}){
						best_time=t;
						best_face=Some(face_id);
					}
				}
			}
		}
		best_face.map(|f|(f,best_time))
	}
	pub fn brute_out_face(&self,body:&crate::physics::Body,time_limit:crate::integer::Time,face_id:FaceId)->Option<(FaceId,crate::integer::Time)>{
		//check each face
		let mut best_time=time_limit;
		let mut best_face=None;
		for &(_,test_face_id) in self.mesh.face_edges(face_id).iter(){
			let (n,d)=self.face_nd(test_face_id);
			for t in crate::zeroes::zeroes2((n.dot(body.position)-d)*2,n.dot(body.velocity)*2,n.dot(body.acceleration)){
				let t=body.time+crate::integer::Time::from(t);
				if body.time<t&&t<best_time&&n.dot(body.extrapolated_velocity(t))>Planar64::ZERO{
					best_time=t;
					best_face=Some(test_face_id);
				}
			}
		}
		best_face.map(|f|(f,best_time))
	}
	#[inline]
	fn vert_directed_edges(&self,vert_id:VertId)->Cow<Vec<DirectedEdgeId>>{
		self.mesh.vert_directed_edges(vert_id)
	}
	#[inline]
	fn directed_edge_n(&self,directed_edge_id:DirectedEdgeId)->Planar64Vec3{
		self.mesh.directed_edge_n(directed_edge_id)
	}
}
impl MeshQuery<FaceId,EdgeId,VertId> for TransformedMesh<'_>{
	fn closest_fev(&self,point:Planar64Vec3)->FEV<FaceId,EdgeId,VertId>{
		//TODO: put some genius code right here

		//brute force for now
		let mut best_distance_squared=Planar64::MAX;
		//make something up as default ret
		//hopefully empty meshes don't make their way through here
		let mut best_fev=FEV::<FaceId,EdgeId,VertId>::Vert(VertId(0));
		//check each vert
		for i in 0..self.mesh.verts.len(){
			let v=self.vert(VertId(i));
			let d=(v-point).dot(v-point);
			if d<best_distance_squared{
				best_distance_squared=d;
				best_fev=FEV::<FaceId,EdgeId,VertId>::Vert(VertId(i));
			}
		}
		//check each edge
		for (i,e) in self.mesh.edge_topology.iter().enumerate(){
			let v0=self.vert(e.verts[0]);
			let v1=self.vert(e.verts[1]);
			let n=v1-v0;
			//n.cross(point-v0)=sin(t)*n*dis
			let d=n.dot(point-v0);
			if d<n.dot(v1)&&n.dot(v0)<d{
				let c=n.cross(point-v0);
				let edge_distance_squared=c.dot(c)/n.dot(n);
				if edge_distance_squared<best_distance_squared{
					best_distance_squared=edge_distance_squared;
					best_fev=FEV::<FaceId,EdgeId,VertId>::Edge(EdgeId(i));
				}
			}
		}
		let face_dots:Vec<Planar64>=self.mesh.faces.iter().map(|f|(*self.normal_transform*f.normal).dot(point)).collect();
		//check each face
		for (i,f) in self.mesh.face_topology.iter().enumerate(){
			if face_dots[i]<best_distance_squared&&f.edges.iter().all(|&(_,face_id)|face_dots[face_id.0]<=Planar64::ZERO){
				best_distance_squared=face_dots[i];
				best_fev=FEV::<FaceId,EdgeId,VertId>::Face(FaceId(i));
			}
		}
		best_fev
	}
	fn face_nd(&self,face_id:FaceId)->(Planar64Vec3,Planar64){
		let (n,d)=self.mesh.face_nd(face_id);
		let transformed_n=*self.normal_transform*n;
		let transformed_d=Planar64::raw(((transformed_n.dot128(self.transform.matrix3*(n*d))<<32)/n.dot128(n)) as i64)+transformed_n.dot(self.transform.translation);
		(transformed_n,transformed_d)
	}
	fn vert(&self,vert_id:VertId)->Planar64Vec3{
		self.transform.transform_point3(self.mesh.vert(vert_id))
	}
	#[inline]
	fn face_edges(&self,face_id:FaceId)->Cow<Vec<(EdgeId,FaceId)>>{
		self.mesh.face_edges(face_id)
	}
	#[inline]
	fn edge_faces(&self,edge_id:EdgeId)->Cow<[FaceId;2]>{
		self.mesh.edge_faces(edge_id)
	}
	#[inline]
	fn edge_verts(&self,edge_id:EdgeId)->Cow<[VertId;2]>{
		self.mesh.edge_verts(edge_id)
	}
	#[inline]
	fn vert_edges(&self,vert_id:VertId)->Cow<Vec<EdgeId>>{
		self.mesh.vert_edges(vert_id)
	}
	#[inline]
	fn vert_faces(&self,vert_id:VertId)->Cow<Vec<FaceId>>{
		self.mesh.vert_faces(vert_id)
	}
}

//Note that a face on a minkowski mesh refers to a pair of fevs on the meshes it's summed from
//(face,vertex)
//(edge,edge)
//(vertex,face)
#[derive(Clone,Copy)]
enum MinkowskiVert{
	VertVert(VertId,VertId),
}
#[derive(Clone,Copy)]
enum MinkowskiEdge{
	VertEdge(VertId,EdgeId),
	EdgeVert(EdgeId,VertId),
}
#[derive(Debug,Clone,Copy,Hash,Eq,PartialEq)]
pub enum MinkowskiFace{
	VertFace(VertId,FaceId),
	EdgeEdge(EdgeId,EdgeId),
	FaceVert(FaceId,VertId),
}

pub struct MinkowskiMesh<'a>{
	mesh0:&'a TransformedMesh<'a>,
	mesh1:&'a TransformedMesh<'a>,
}

impl MinkowskiMesh<'_>{
	pub fn minkowski_sum<'a>(mesh0:&'a TransformedMesh,mesh1:&'a TransformedMesh)->MinkowskiMesh<'a>{
		MinkowskiMesh{
			mesh0,
			mesh1,
		}
	}
}
impl MeshQuery<MinkowskiFace,MinkowskiEdge,MinkowskiVert> for MinkowskiMesh<'_>{
	fn closest_fev(&self,point:Planar64Vec3)->FEV<MinkowskiFace,MinkowskiEdge,MinkowskiVert>{
		//put some genius code right here
		todo!()
	}
	fn face_nd(&self,face_id:MinkowskiFace)->(Planar64Vec3,Planar64){
		match face_id{
			MinkowskiFace::VertFace(v0,f1)=>{
				let (n,d)=self.mesh1.face_nd(f1);
				(-n,d-n.dot(self.mesh0.vert(v0)))
			},
			MinkowskiFace::EdgeEdge(e0,e1)=>{
				let edge0_n=self.mesh0.edge_n(e0);
				let edge1_n=self.mesh1.edge_n(e1);
				let &[e0v0,e0v1]=self.mesh0.edge_verts(e0).borrow();
				let &[e1v0,e1v1]=self.mesh1.edge_verts(e1).borrow();
				let n=edge0_n.cross(edge1_n);
				let e0d=n.dot(self.mesh0.vert(e0v0)+self.mesh0.vert(e0v1));
				let e1d=n.dot(self.mesh0.vert(e1v0)+self.mesh0.vert(e1v1));
				let sign=e0d.signum_i64();
				(n*(sign*2),(e0d-e1d)*sign)
			},
			MinkowskiFace::FaceVert(f0,v1)=>{
				let (n,d)=self.mesh0.face_nd(f0);
				(n,d+n.dot(self.mesh1.vert(v1)))
			},
		}
	}
	fn vert(&self,vert_id:MinkowskiVert)->Planar64Vec3{
		match vert_id{
			MinkowskiVert::VertVert(v0,v1)=>{
				self.mesh0.vert(v0)-self.mesh1.vert(v1)
			},
		}
	}
	fn face_edges(&self,face_id:MinkowskiFace)->Cow<Vec<(MinkowskiEdge,MinkowskiFace)>>{
		match face_id{
			MinkowskiFace::VertFace(v0,f1)=>{
				let face1_n=self.mesh1.face_nd(f1).0;
				Cow::Owned(self.mesh1.face_edges(f1).iter().map(|&(edge_id1,edge_face_id1)|{
					//same as above
					(MinkowskiEdge::VertEdge(v0,edge_id1),{
						let mut best_edge=None;
						let mut best_d=Planar64::MAX;
						let edge_face1_n=self.mesh1.face_nd(edge_face_id1).0;
						let v0e=self.mesh0.vert_directed_edges(v0);
						for &directed_edge_id0 in v0e.iter(){
							let edge0_n=self.mesh0.directed_edge_n(directed_edge_id0);
							if edge_face1_n.dot(edge0_n)<Planar64::ZERO{
								let d=face1_n.dot(edge0_n);
								if d<best_d{
									best_d=d;
									best_edge=Some(directed_edge_id0)
								}
							}
						}
						best_edge.map_or(
							MinkowskiFace::VertFace(v0,edge_face_id1),
							|directed_edge_id0|MinkowskiFace::EdgeEdge(directed_edge_id0.as_edge_id(),edge_id1)
						)
					})
				}).collect())
			},
			MinkowskiFace::EdgeEdge(e0,e1)=>{
				/*
				let e0v=self.mesh0.edge_verts(e0);
				let e1v=self.mesh1.edge_verts(e1);
				let [r0,r1]=e0v.map(|vert_id0|{
					//sort e1 ends by e0 edge dir to get v1
					//find face normal formulation without cross products
					let v1=if 0<(e0.v1-e0.v0).dot(e1.v1-e1.v0){
						e1.v0
					}else{
						e1.v1
					};
					(MinkowskiEdge::VertEdge(vert_id0,e1),MinkowskiFace::FaceVert(face_id0,v1))
				});
				let [r2,r3]=e1v.map(|vert_id1|{
					//sort e0 ends by e1 edge dir to get v0
					let v0=if 0<(e0.v1-e0.v0).dot(e1.v1-e1.v0){
						e0.v0
					}else{
						e0.v1
					};
					(MinkowskiEdge::EdgeVert(e0,vert_id1),MinkowskiFace::VertFace(v0,face_id1))
				});
				Cow::Owned(vec![r0,r1,r2,r3])
				*/
				todo!()
			},
			MinkowskiFace::FaceVert(f0,v1)=>{
				let face0_n=self.mesh0.face_nd(f0).0;
				Cow::Owned(self.mesh0.face_edges(f0).iter().map(|&(edge_id0,edge_face_id0)|{
					//compare v1 edges
					//candidate edges have negative dot with edge_face_id0 normal
					//choose the edge with the smallest edgedir dot with f0 normal
					//MinkowskiFace::EdgeEdge(edge_id0,edge_id1)
					//if there is no candidate edges
					//MinkowskiFace::FaceVert(edge_face_id0,v1)
					(MinkowskiEdge::EdgeVert(edge_id0,v1),{
						let mut best_edge=None;
						let mut best_d=Planar64::MAX;
						let edge_face0_n=self.mesh0.face_nd(edge_face_id0).0;
						let v1e=self.mesh1.vert_directed_edges(v1);
						for &directed_edge_id1 in v1e.iter(){
							let edge1_n=self.mesh1.directed_edge_n(directed_edge_id1);
							if edge_face0_n.dot(edge1_n)<Planar64::ZERO{
								let d=face0_n.dot(edge1_n);
								if d<best_d{
									best_d=d;
									best_edge=Some(directed_edge_id1)
								}
							}
						}
						best_edge.map_or(
							MinkowskiFace::FaceVert(edge_face_id0,v1),
							|directed_edge_id1|MinkowskiFace::EdgeEdge(edge_id0,directed_edge_id1.as_edge_id())
						)
					})
				}).collect())
			},
		}
	}
	fn edge_faces(&self,edge_id:MinkowskiEdge)->Cow<[MinkowskiFace;2]>{
		//WRONG!!!!!!!!!!!! MORE CASES!!!!!!!!!!!!
		match edge_id{
			MinkowskiEdge::VertEdge(v0,e1)=>{
				//also need to check v0 edges to see if they overtake the face
				Cow::Owned(self.mesh1.edge_faces(e1).map(|face_id1|{
					MinkowskiFace::VertFace(v0,face_id1)
				}))
			},
			MinkowskiEdge::EdgeVert(e0,v1)=>{
				Cow::Owned(self.mesh0.edge_faces(e0).map(|face_id0|{
					MinkowskiFace::FaceVert(face_id0,v1)
				}))
			},
		}
	}
	fn edge_verts(&self,edge_id:MinkowskiEdge)->Cow<[MinkowskiVert;2]>{
		match edge_id{
			MinkowskiEdge::VertEdge(v0,e1)=>{
				Cow::Owned(self.mesh1.edge_verts(e1).map(|vert_id1|{
					MinkowskiVert::VertVert(v0,vert_id1)
				}))
			},
			MinkowskiEdge::EdgeVert(e0,v1)=>{
				Cow::Owned(self.mesh0.edge_verts(e0).map(|vert_id0|{
					MinkowskiVert::VertVert(vert_id0,v1)
				}))
			},
		}
	}
	fn vert_edges(&self,vert_id:MinkowskiVert)->Cow<Vec<MinkowskiEdge>>{
		match vert_id{
			MinkowskiVert::VertVert(v0,v1)=>{
				let mut edges=Vec::new();
				let v0e=self.mesh0.vert_directed_edges(v0);
				let v1f=self.mesh1.vert_faces(v1);
				for &directed_edge_id in v0e.iter(){
					let n=self.mesh0.directed_edge_n(directed_edge_id);
					if v1f.iter().all(|&face_id|n.dot(self.mesh1.face_nd(face_id).0)<Planar64::ZERO){
						edges.push(MinkowskiEdge::EdgeVert(directed_edge_id.as_edge_id(),v1));
					}
				}
				let v1e=self.mesh1.vert_directed_edges(v1);
				let v0f=self.mesh0.vert_faces(v0);
				for &directed_edge_id in v1e.iter(){
					let n=self.mesh1.directed_edge_n(directed_edge_id);
					if v0f.iter().all(|&face_id|n.dot(self.mesh0.face_nd(face_id).0)<Planar64::ZERO){
						edges.push(MinkowskiEdge::VertEdge(v0,directed_edge_id.as_edge_id()));
					}
				}
				Cow::Owned(edges)
			},
		}
	}
	fn vert_faces(&self,vert_id:MinkowskiVert)->Cow<Vec<MinkowskiFace>>{
		todo!()
	}
}

#[test]
fn build_me_a_cube(){
	let unit_cube=crate::primitives::unit_cube();
	let mesh=PhysicsMesh::from(&unit_cube);
	println!("mesh={:?}",mesh);
}
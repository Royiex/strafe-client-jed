use crate::integer::{Planar64,Planar64Vec3};
use std::borrow::{Borrow,Cow};

#[derive(Debug,Clone,Copy,Hash,Eq,PartialEq)]
pub struct VertId(usize);
#[derive(Debug,Clone,Copy,Hash,Eq,PartialEq)]
pub struct EdgeId(usize);
impl EdgeId{
	fn as_directed(&self,parity:bool)->DirectedEdgeId{
		DirectedEdgeId(self.0|((parity as usize)<<(usize::BITS-1)))
	}
}
pub trait DirectedEdge{
	type UndirectedEdge;
	fn as_undirected(&self)->Self::UndirectedEdge;
	fn parity(&self)->bool;
}
/// DirectedEdgeId refers to an EdgeId when undirected.
#[derive(Debug,Clone,Copy,Hash,Eq,PartialEq)]
pub struct DirectedEdgeId(usize);
impl DirectedEdge for DirectedEdgeId{
	type UndirectedEdge=EdgeId;
	fn as_undirected(&self)->EdgeId{
		EdgeId(self.0&!(1<<(usize::BITS-1)))
	}
	fn parity(&self)->bool{
		self.0&(1<<(usize::BITS-1))!=0
	}
}
#[derive(Debug,Clone,Copy,Hash,Eq,PartialEq)]
pub struct FaceId(usize);

//Vertex <-> Edge <-> Face -> Collide
pub enum FEV<F,E:DirectedEdge,V>{
	Face(F),
	Edge(E::UndirectedEdge),
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
pub trait MeshQuery<FACE:Clone,EDGE:Clone+DirectedEdge,VERT:Clone>{
	fn edge_n(&self,edge_id:EDGE::UndirectedEdge)->Planar64Vec3{
		let verts=self.edge_verts(edge_id);
		self.vert(verts[1].clone())-self.vert(verts[0].clone())
	}
	fn directed_edge_n(&self,directed_edge_id:EDGE)->Planar64Vec3{
		let verts=self.edge_verts(directed_edge_id.as_undirected());
		(self.vert(verts[1].clone())-self.vert(verts[0].clone()))*((directed_edge_id.parity() as i64)*2-1)
	}
	fn vert(&self,vert_id:VERT)->Planar64Vec3;
	fn face_nd(&self,face_id:FACE)->(Planar64Vec3,Planar64);
	fn face_edges(&self,face_id:FACE)->Cow<Vec<EDGE>>;
	fn edge_faces(&self,edge_id:EDGE::UndirectedEdge)->Cow<[FACE;2]>;
	fn edge_verts(&self,edge_id:EDGE::UndirectedEdge)->Cow<[VERT;2]>;
	fn vert_edges(&self,vert_id:VERT)->Cow<Vec<EDGE>>;
	fn vert_faces(&self,vert_id:VERT)->Cow<Vec<FACE>>;
}
struct FaceRefs{
	edges:Vec<DirectedEdgeId>,
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
struct FaceRefGuy(Vec<DirectedEdgeId>);
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
					vert_ref_guy.edges.insert(edge_id.as_directed(!is_sorted));
					vert_ref_guy.faces.insert(face_id);
					unsafe{vert_ref_guys.get_unchecked_mut(vert1_id)}.edges.insert(edge_id.as_directed(is_sorted));
				}
				//return directed_edge_id
				edge_id.as_directed(is_sorted)
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
			face_topology:face_ref_guys.into_iter().map(|face_ref_guy|{
				FaceRefs{edges:face_ref_guy.0}
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

impl PhysicsMesh{
	pub fn verts<'a>(&'a self)->impl Iterator<Item=Planar64Vec3>+'a{
		self.verts.iter().map(|Vert(pos)|*pos)
	}
}
impl MeshQuery<FaceId,DirectedEdgeId,VertId> for PhysicsMesh{
	fn face_nd(&self,face_id:FaceId)->(Planar64Vec3,Planar64){
		(self.faces[face_id.0].normal,self.faces[face_id.0].dot)
	}
	//ideally I never calculate the vertex position, but I have to for the graphical meshes...
	fn vert(&self,vert_id:VertId)->Planar64Vec3{
		self.verts[vert_id.0].0
	}
	fn face_edges(&self,face_id:FaceId)->Cow<Vec<DirectedEdgeId>>{
		Cow::Borrowed(&self.face_topology[face_id.0].edges)
	}
	fn edge_faces(&self,edge_id:EdgeId)->Cow<[FaceId;2]>{
		Cow::Borrowed(&self.edge_topology[edge_id.0].faces)
	}
	fn edge_verts(&self,edge_id:EdgeId)->Cow<[VertId;2]>{
		Cow::Borrowed(&self.edge_topology[edge_id.0].verts)
	}
	fn vert_edges(&self,vert_id:VertId)->Cow<Vec<DirectedEdgeId>>{
		Cow::Borrowed(&self.vert_topology[vert_id.0].edges)
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
	fn farthest_vert(&self,dir:Planar64Vec3)->VertId{
		let best_dot=Planar64::MIN;
		let best_vert;
		for (i,vert) in self.mesh.verts.iter().enumerate(){
			let p=self.transform.transform_point3(vert.0);
			let d=dir.dot(p);
			if best_dot<d{
				best_dot=d;
				best_vert=VertId(i);
			}
		}
		best_vert
	}
}
impl MeshQuery<FaceId,DirectedEdgeId,VertId> for TransformedMesh<'_>{
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
	fn face_edges(&self,face_id:FaceId)->Cow<Vec<DirectedEdgeId>>{
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
	fn vert_edges(&self,vert_id:VertId)->Cow<Vec<DirectedEdgeId>>{
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
pub enum MinkowskiEdge{
	VertEdge(VertId,EdgeId),
	EdgeVert(EdgeId,VertId),
	//EdgeEdge when edges are parallel
}
#[derive(Clone,Copy)]
enum MinkowskiDirectedEdge{
	VertEdge(VertId,DirectedEdgeId),
	EdgeVert(DirectedEdgeId,VertId),
	//EdgeEdge when edges are parallel
}
impl DirectedEdge for MinkowskiDirectedEdge{
	type UndirectedEdge=MinkowskiEdge;
	fn as_undirected(&self)->Self::UndirectedEdge{
		match self{
			MinkowskiDirectedEdge::VertEdge(v0,e1)=>MinkowskiEdge::VertEdge(*v0,e1.as_undirected()),
			MinkowskiDirectedEdge::EdgeVert(e0,v1)=>MinkowskiEdge::EdgeVert(e0.as_undirected(),*v1),
		}
	}
	fn parity(&self)->bool{
		match self{
			MinkowskiDirectedEdge::VertEdge(_,e)
			|MinkowskiDirectedEdge::EdgeVert(e,_)=>e.parity(),
		}
	}
}
#[derive(Debug,Clone,Copy,Hash,Eq,PartialEq)]
pub enum MinkowskiFace{
	VertFace(VertId,FaceId),
	EdgeEdge(EdgeId,EdgeId,bool),
	FaceVert(FaceId,VertId),
	//EdgeFace
	//FaceEdge
	//FaceFace
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
	fn farthest_vert(&self,dir:Planar64Vec3)->MinkowskiVert{
		MinkowskiVert::VertVert(self.mesh0.farthest_vert(dir),self.mesh1.farthest_vert(-dir))
	}
	fn closest_fev(&self,point:Planar64Vec3)->FEV<MinkowskiFace,MinkowskiDirectedEdge,MinkowskiVert>{
		//put some genius code right here instead of this
		//assume that point is outside the mesh and nonzero
		//find vertex on mesh0 farthest in point direction
		let fev=FEV::<MinkowskiFace,MinkowskiDirectedEdge,MinkowskiVert>::Vert(self.farthest_vert(point));
		crate::face_crawler::crawl_fev_dot(fev,self,point)
	}
	pub fn predict_collision_in(&self,relative_body:&crate::physics::Body,time_limit:crate::integer::Time)->Option<(MinkowskiFace,crate::integer::Time)>{
		crate::face_crawler::crawl_fev_body(self.closest_fev(relative_body.position),self,relative_body,time_limit)
	}
	pub fn predict_collision_face_out(&self,relative_body:&crate::physics::Body,time_limit:crate::integer::Time,contact_face_id:MinkowskiFace)->Option<(MinkowskiEdge,crate::integer::Time)>{
		//no algorithm needed, there is only one state and two cases (Edge,None)
		//determine when it passes an edge ("sliding off" case)
		let mut best_time=time_limit;
		let mut best_edge=None;
		let face_n=self.face_nd(contact_face_id).0;
		for &directed_edge_id in self.face_edges(contact_face_id).iter(){
			let edge_n=self.directed_edge_n(directed_edge_id);
			//f x e points out
			let n=-face_n.cross(edge_n);
			let verts=self.edge_verts(directed_edge_id.as_undirected());
			let d=n.dot(self.vert(verts[0]))+n.dot(self.vert(verts[1]));
			//WARNING! d outside of *2
			for t in crate::zeroes::zeroes2((n.dot(relative_body.position))*2-d,n.dot(relative_body.velocity)*2,n.dot(relative_body.acceleration)){
				let t=relative_body.time+crate::integer::Time::from(t);
				if relative_body.time<t&&t<best_time&&n.dot(relative_body.extrapolated_velocity(t))<Planar64::ZERO{
					best_time=t;
					best_edge=Some(directed_edge_id);
					break;
				}
			}
		}
		best_edge.map(|e|(e.as_undirected(),best_time))
	}
}
impl MeshQuery<MinkowskiFace,MinkowskiDirectedEdge,MinkowskiVert> for MinkowskiMesh<'_>{
	fn face_nd(&self,face_id:MinkowskiFace)->(Planar64Vec3,Planar64){
		match face_id{
			MinkowskiFace::VertFace(v0,f1)=>{
				let (n,d)=self.mesh1.face_nd(f1);
				(-n,d-n.dot(self.mesh0.vert(v0)))
			},
			MinkowskiFace::EdgeEdge(e0,e1,parity)=>{
				let edge0_n=self.mesh0.edge_n(e0);
				let edge1_n=self.mesh1.edge_n(e1);
				let &[e0v0,e0v1]=self.mesh0.edge_verts(e0).borrow();
				let &[e1v0,e1v1]=self.mesh1.edge_verts(e1).borrow();
				let n=edge0_n.cross(edge1_n);
				let e0d=n.dot(self.mesh0.vert(e0v0)+self.mesh0.vert(e0v1));
				let e1d=n.dot(self.mesh0.vert(e1v0)+self.mesh0.vert(e1v1));
				(n*(parity as i64*4-2),(e0d-e1d)*(parity as i64*2-1))
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
	fn face_edges(&self,face_id:MinkowskiFace)->Cow<Vec<MinkowskiDirectedEdge>>{
		match face_id{
			MinkowskiFace::VertFace(v0,f1)=>{
				Cow::Owned(self.mesh1.face_edges(f1).iter().map(|&edge_id1|{
					MinkowskiDirectedEdge::VertEdge(v0,edge_id1)
				}).collect())
			},
			MinkowskiFace::EdgeEdge(e0,e1,parity)=>{
				let e0v=self.mesh0.edge_verts(e0);
				let e1v=self.mesh1.edge_verts(e1);
				//could sort this if ordered edges are needed
				Cow::Owned(vec![
					MinkowskiDirectedEdge::VertEdge(e0v[0],e1.as_directed(parity)),
					MinkowskiDirectedEdge::EdgeVert(e0.as_directed(parity),e1v[0]),
					MinkowskiDirectedEdge::VertEdge(e0v[1],e1.as_directed(!parity)),
					MinkowskiDirectedEdge::EdgeVert(e0.as_directed(!parity),e1v[1]),
				])
			},
			MinkowskiFace::FaceVert(f0,v1)=>{
				Cow::Owned(self.mesh0.face_edges(f0).iter().map(|&edge_id0|{
					MinkowskiDirectedEdge::EdgeVert(edge_id0,v1)
				}).collect())
			},
		}
	}
	fn edge_faces(&self,edge_id:MinkowskiEdge)->Cow<[MinkowskiFace;2]>{
		match edge_id{
			MinkowskiEdge::VertEdge(v0,e1)=>{
				let e1f=self.mesh1.edge_faces(e1);
				Cow::Owned([(e1f[0],e1f[1],true),(e1f[1],e1f[0],false)].map(|(edge_face_id1,other_edge_face_id1,face_parity)|{
					let mut best_edge=None;
					let mut best_d=Planar64::MAX;
					let edge_face1_n=self.mesh1.face_nd(edge_face_id1).0;
					let other_edge_face1_n=self.mesh1.face_nd(other_edge_face_id1).0;
					for &directed_edge_id0 in self.mesh0.vert_edges(v0).iter(){
						let edge0_n=self.mesh0.directed_edge_n(directed_edge_id0);
						if edge_face1_n.dot(edge0_n)<Planar64::ZERO{
							let d=other_edge_face1_n.dot(edge0_n);
							if d<best_d{
								best_d=d;
								best_edge=Some(directed_edge_id0);
							}
						}
					}
					best_edge.map_or(
						MinkowskiFace::VertFace(v0,edge_face_id1),
						|directed_edge_id0|MinkowskiFace::EdgeEdge(directed_edge_id0.as_undirected(),e1,directed_edge_id0.parity()^face_parity)
					)
				}))
			},
			MinkowskiEdge::EdgeVert(e0,v1)=>{
				let e0f=self.mesh0.edge_faces(e0);
				Cow::Owned([(e0f[0],e0f[1],true),(e0f[1],e0f[0],false)].map(|(edge_face_id0,other_edge_face_id0,face_parity)|{
					let mut best_edge=None;
					let mut best_d=Planar64::MAX;
					let edge_face0_n=self.mesh0.face_nd(edge_face_id0).0;
					let other_edge_face0_n=self.mesh0.face_nd(other_edge_face_id0).0;
					for &directed_edge_id1 in self.mesh1.vert_edges(v1).iter(){
						let edge1_n=self.mesh1.directed_edge_n(directed_edge_id1);
						if edge_face0_n.dot(edge1_n)<Planar64::ZERO{
							let d=other_edge_face0_n.dot(edge1_n);
							if d<best_d{
								best_d=d;
								best_edge=Some(directed_edge_id1);
							}
						}
					}
					best_edge.map_or(
						MinkowskiFace::FaceVert(edge_face_id0,v1),
						|directed_edge_id1|MinkowskiFace::EdgeEdge(e0,directed_edge_id1.as_undirected(),directed_edge_id1.parity()^face_parity)
					)
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
	fn vert_edges(&self,vert_id:MinkowskiVert)->Cow<Vec<MinkowskiDirectedEdge>>{
		match vert_id{
			MinkowskiVert::VertVert(v0,v1)=>{
				let mut edges=Vec::new();
				let v0e=self.mesh0.vert_edges(v0);
				let v1f=self.mesh1.vert_faces(v1);
				for &directed_edge_id in v0e.iter(){
					let n=self.mesh0.directed_edge_n(directed_edge_id);
					if v1f.iter().all(|&face_id|n.dot(self.mesh1.face_nd(face_id).0)<Planar64::ZERO){
						edges.push(MinkowskiDirectedEdge::EdgeVert(directed_edge_id,v1));
					}
				}
				let v1e=self.mesh1.vert_edges(v1);
				let v0f=self.mesh0.vert_faces(v0);
				for &directed_edge_id in v1e.iter(){
					let n=self.mesh1.directed_edge_n(directed_edge_id);
					if v0f.iter().all(|&face_id|n.dot(self.mesh0.face_nd(face_id).0)<Planar64::ZERO){
						edges.push(MinkowskiDirectedEdge::VertEdge(v0,directed_edge_id));
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
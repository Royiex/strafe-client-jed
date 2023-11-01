use crate::integer::{Planar64,Planar64Vec3};
use std::borrow::Cow;

#[derive(Debug,Clone,Copy,Hash,Eq,PartialEq)]
pub struct VertId(usize);
#[derive(Debug,Clone,Copy,Hash,Eq,PartialEq)]
pub struct EdgeId(usize);
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
	//faces:Vec<FaceId>,
	edges:Vec<EdgeId>,
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
	edges:std::collections::HashSet<EdgeId>,
}
#[derive(Hash,Eq,PartialEq)]
struct EdgeIdGuy([VertId;2]);
impl EdgeIdGuy{
	fn new(v0:VertId,v1:VertId)->Self{
		if v0.0<v1.0{
			Self([v0,v1])
		}else{
			Self([v1,v0])
		}
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
	fn push(&mut self,edge_id_guy:EdgeIdGuy)->(&mut EdgeRefGuy,EdgeId,bool){
		if let Some(&edge_id)=self.edge_id_from_guy.get(&edge_id_guy){
			(&mut unsafe{self.edge_guys.get_unchecked_mut(edge_id)}.1,EdgeId(edge_id),true)
		}else{
			let edge_id=self.edge_guys.len();
			self.edge_guys.push((edge_id_guy,EdgeRefGuy::new()));
			self.edge_id_from_guy.insert(edge_id_guy,edge_id);
			(&mut unsafe{self.edge_guys.get_unchecked_mut(edge_id)}.1,EdgeId(edge_id),false)
		}
	}
}
impl From<&crate::model::IndexedModel> for PhysicsMesh{
	fn from(indexed_model:&crate::model::IndexedModel)->Self{
		let verts=indexed_model.unique_pos.iter().map(|v|Vert(v.clone())).collect();
		let mut vert_edges=vec![VertRefGuy::default();indexed_model.unique_pos.len()];
		let mut edge_pool=EdgePool::default();
		let (faces,face_ref_guys):(Vec<Face>,Vec<FaceRefGuy>)=indexed_model.groups[0].polys.iter().enumerate().map(|(i,poly)|{
			let face_id=FaceId(i);
			//one face per poly
			let mut normal=Planar64Vec3::ZERO;
			let len=poly.vertices.len();
			let face_edges=poly.vertices.iter().enumerate().map(|(i,&vert_id)|{
				let vert0_id=vert_id as usize;
				let vert1_id=poly.vertices[(i+1)%len] as usize;
				//https://www.khronos.org/opengl/wiki/Calculating_a_Surface_Normal (Newell's Method)
				let v0=indexed_model.unique_pos[vert0_id];
				let v1=indexed_model.unique_pos[vert1_id];
				normal+=Planar64Vec3::new(
					(v0.y()-v1.y())*(v0.z()+v1.z()),
					(v0.z()-v1.z())*(v0.x()+v1.x()),
					(v0.x()-v1.x())*(v0.y()+v1.y()),
				);
				//get/create edge and push face into it
				let edge_id_guy=EdgeIdGuy::new(VertId(vert0_id),VertId(vert1_id));
				let (edge_ref_guy,edge_id,exists)=edge_pool.push(edge_id_guy);
				if exists{
					edge_ref_guy.push(1,face_id);
				}else{
					edge_ref_guy.push(0,face_id);
				}
				//index edge into vertices
				unsafe{vert_edges.get_unchecked_mut(vert0_id)}.edges.insert(edge_id);
				unsafe{vert_edges.get_unchecked_mut(vert1_id)}.edges.insert(edge_id);
				//return edge_id
				edge_id
			}).collect();
			//choose precision loss randomly idk
			normal=normal/len as i64;
			let mut dot=Planar64::ZERO;
			for &v in poly.vertices.iter(){
				dot+=normal.dot(indexed_model.unique_pos[v as usize]);
			}
			(Face{normal,dot:dot/len as i64},FaceRefGuy(face_edges))
		}).unzip();
		//conceivably faces, edges, and vertices exist now
		Self{
			faces,
			verts,
			face_topology:face_ref_guys.into_iter().enumerate().map(|(i,face_ref_guy)|{
				let face_id=FaceId(i);
				FaceRefs{edges:face_ref_guy.0.into_iter().map(|edge_id|{
					//get the edge face that's not this face
					let edge_faces=edge_pool.edge_guys[edge_id.0].1.0;
					if edge_faces[0]==face_id{
						(edge_id,edge_faces[1])
					}else if edge_faces[1]==face_id{
						(edge_id,edge_faces[0])
					}else{
						panic!("edge does not contain face")
					}
				}).collect()}
			}).collect(),
			edge_topology:edge_pool.edge_guys.into_iter().map(|(edge_id_guy,edge_ref_guy)|
				EdgeRefs{faces:edge_ref_guy.0,verts:edge_id_guy.0}
			).collect(),
			vert_topology:vert_edges.into_iter().map(|vert_ref_guy|
				VertRefs{edges:vert_ref_guy.edges.into_iter().collect()}
			).collect(),
		}
	}
}

pub trait MeshQuery<FACE:Clone,EDGE:Clone,VERT:Clone>{
	fn closest_fev(&self,point:Planar64Vec3)->FEV<FACE,EDGE,VERT>;
	fn edge_n(&self,edge_id:EDGE)->Planar64Vec3{
		let verts=self.edge_verts(edge_id);
		self.vert(verts[1])-self.vert(verts[0])
	}
	fn vert(&self,vert_id:VERT)->Planar64Vec3;
	fn face_nd(&self,face_id:FACE)->(Planar64Vec3,Planar64);
	fn face_edges(&self,face_id:FACE)->Cow<Vec<(EDGE,FACE)>>;
	fn edge_faces(&self,edge_id:EDGE)->Cow<[FACE;2]>;
	fn edge_verts(&self,edge_id:EDGE)->Cow<[VERT;2]>;
	fn vert_edges(&self,vert_id:VERT)->Cow<Vec<EDGE>>;
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
		Cow::Borrowed(&self.vert_topology[vert_id.0].edges)
	}
}

pub struct TransformedMesh<'a>{
	mesh:&'a PhysicsMesh,
	transform:&'a crate::integer::Planar64Affine3,
	normal_transform:&'a crate::integer::Planar64Mat3,
	normal_determinant:Planar64,
}
impl MeshQuery<FaceId,EdgeId,VertId> for TransformedMesh<'_>{
	fn closest_fev(&self,point:Planar64Vec3)->FEV<FaceId,EdgeId,VertId>{
		//put some genius code right here
		todo!()
	}
	fn face_nd(&self,face_id:FaceId)->(Planar64Vec3,Planar64){
		let (n,d)=self.mesh.face_nd(face_id);
		(*self.normal_transform*n,self.normal_determinant*d)
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
	FaceVert(FaceId,VertId),
	EdgeEdge(EdgeId,EdgeId),
	VertFace(VertId,FaceId),
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
			MinkowskiFace::FaceVert(f0,v1)=>{
				let (n,d)=self.mesh0.face_nd(f0);
				(n,d+n.dot(self.mesh1.vert(v1)))
			},
			MinkowskiFace::EdgeEdge(e0,e1)=>{
				let [e0f0,e0f1]=self.mesh0.edge_faces(e0).into_owned();
				let [e1f0,e1f1]=self.mesh1.edge_faces(e1).into_owned();
				//cross edge faces
				//cross crosses
				todo!()
			},
			MinkowskiFace::VertFace(v0,f1)=>{
				let (n,d)=self.mesh1.face_nd(f1);
				(-n,d-n.dot(self.mesh0.vert(v0)))
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
			MinkowskiFace::FaceVert(f0,v1)=>{
				Cow::Owned(self.mesh0.face_edges(f0).iter().map(|&(edge_id0,face_id0)|{
					(MinkowskiEdge::EdgeVert(edge_id0,v1),MinkowskiFace::FaceVert(face_id0,v1))
				}).collect())
			},
			MinkowskiFace::EdgeEdge(e0,e1)=>{
				let e0v=self.mesh0.edge_verts(e0);
				let e1v=self.mesh1.edge_verts(e1);
				let [r0,r1]=e0v.map(|(vert_id0,face_id0)|{
					//sort e1 ends by e0 edge dir to get v1
					//find face normal formulation without cross products
					let v1=if 0<(e0.v1-e0.v0).dot(e1.v1-e1.v0){
						e1.v0
					}else{
						e1.v1
					};
					(MinkowskiEdge::VertEdge(vert_id0,e1),MinkowskiFace::FaceVert(face_id0,v1))
				});
				let [r2,r3]=e1v.map(|(vert_id1,face_id1)|{
					//sort e0 ends by e1 edge dir to get v0
					let v0=if 0<(e0.v1-e0.v0).dot(e1.v1-e1.v0){
						e0.v0
					}else{
						e0.v1
					};
					(MinkowskiEdge::EdgeVert(e0,vert_id1),MinkowskiFace::VertFace(v0,face_id1))
				});
				Cow::Owned(vec![r0,r1,r2,r3])
			},
			MinkowskiFace::VertFace(v0,f1)=>{
				Cow::Owned(self.mesh1.face_edges(f1).iter().map(|&(edge_id1,face_id1)|{
					(MinkowskiEdge::VertEdge(v0,edge_id1),MinkowskiFace::VertFace(v0,face_id1))
				}).collect())
			},
		}
	}
	fn edge_faces(&self,edge_id:MinkowskiEdge)->Cow<[MinkowskiFace;2]>{
		match edge_id{
			MinkowskiEdge::VertEdge(v0,e1)=>{
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
				Cow::Owned(self.mesh1.edge_verts(e1).map(|(vert_id1,face_id1)|{
					(MinkowskiVert::VertVert(v0,vert_id1),MinkowskiFace::VertFace(v0,face_id1))
				}))
			},
			MinkowskiEdge::EdgeVert(e0,v1)=>{
				Cow::Owned(self.mesh0.edge_verts(e0).map(|(vert_id0,face_id0)|{
					(MinkowskiVert::VertVert(vert_id0,v1),MinkowskiFace::FaceVert(face_id0,v1))
				}))
			},
		}
	}
	fn vert_edges(&self,vert_id:MinkowskiVert)->Cow<Vec<MinkowskiEdge>>{
		match vert_id{
			MinkowskiVert::VertVert(v0,v1)=>{
				let v0e=self.mesh0.vert_edges(v0);
				let v1e=self.mesh1.vert_edges(v1);
				//uh oh dot product
				//pass all dots?
				//it's a convex hull of {v0e,-v1e}
				//each edge needs to know which vert to use from the other mesh
				todo!()
			},
		}
	}
}

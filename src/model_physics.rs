use crate::integer::{Planar64,Planar64Vec3};
use std::borrow::Cow;

#[derive(Clone,Copy)]
pub struct VertId(usize);
#[derive(Clone,Copy)]
pub struct EdgeId(usize);
#[derive(Clone,Copy)]
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
	verts:Vec<VertId>,
}
struct EdgeRefs{
	faces:[FaceId;2],//left, right
	verts:[(VertId,FaceId);2],//bottom, top
}
struct VertRefs{
	edges:Vec<(EdgeId,FaceId)>,
}
pub struct PhysicsMesh{
	faces:Vec<Face>,
	verts:Vec<Vert>,
	face_topology:Vec<FaceRefs>,
	edge_topology:Vec<EdgeRefs>,
	vert_topology:Vec<VertRefs>,
}

pub trait MeshQuery<FACE:Clone,EDGE:Clone,VERT:Clone>{
	fn closest_fev(&self,point:Planar64Vec3)->FEV<FACE,EDGE,VERT>;
	fn face_nd(&self,face_id:FACE)->(Planar64Vec3,Planar64);
	fn vert(&self,vert_id:VERT)->Planar64Vec3;
	fn face_edges(&self,face_id:FACE)->Cow<Vec<(EDGE,FACE)>>;
	fn edge_faces(&self,edge_id:EDGE)->Cow<[FACE;2]>;
	fn edge_verts(&self,edge_id:EDGE)->Cow<[(VERT,FACE);2]>;
	fn vert_edges(&self,vert_id:VERT)->Cow<Vec<(EDGE,FACE)>>;
}
impl MeshQuery<FaceId,EdgeId,VertId> for PhysicsMesh{
	fn closest_fev(&self,point:Planar64Vec3)->FEV<FaceId,EdgeId,VertId>{
		//put some genius code right here
		todo!()
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
	fn edge_verts(&self,edge_id:EdgeId)->Cow<[(VertId,FaceId);2]>{
		Cow::Borrowed(&self.edge_topology[edge_id.0].verts)
	}
	fn vert_edges(&self,vert_id:VertId)->Cow<Vec<(EdgeId,FaceId)>>{
		Cow::Borrowed(&self.vert_topology[vert_id.0].edges)
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
#[derive(Clone,Copy)]
enum MinkowskiFace{
	FaceVert(FaceId,VertId),
	EdgeEdge(EdgeId,EdgeId),
	VertFace(VertId,FaceId),
}

pub struct MinkowskiMesh<'a>{
	mesh0:&'a PhysicsMesh,
	mesh1:&'a PhysicsMesh,
}

impl MinkowskiMesh<'_>{
	pub fn minkowski_sum<'a>(mesh0:&'a PhysicsMesh,mesh1:&'a PhysicsMesh)->MinkowskiMesh<'a>{
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
	fn edge_verts(&self,edge_id:MinkowskiEdge)->Cow<[(MinkowskiVert,MinkowskiFace);2]>{
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
	fn vert_edges(&self,vert_id:MinkowskiVert)->Cow<Vec<(MinkowskiEdge,MinkowskiFace)>>{
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

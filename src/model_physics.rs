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
	fn edge_side_faces(&self,edge_id:EDGE)->Cow<[FACE;2]>;
	fn edge_ends(&self,edge_id:EDGE)->Cow<[(VERT,FACE);2]>;
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
		todo!()
	}
	fn face_edges(&self,face_id:FaceId)->Cow<Vec<(EdgeId,FaceId)>>{
		Cow::Borrowed(&self.face_topology[face_id.0].edges)
	}
	fn edge_side_faces(&self,edge_id:EdgeId)->Cow<[FaceId;2]>{
		Cow::Borrowed(&self.edge_topology[edge_id.0].faces)
	}
	fn edge_ends(&self,edge_id:EdgeId)->Cow<[(VertId,FaceId);2]>{
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
		todo!()
	}
	fn face_edges(&self,face_id:MinkowskiFace)->Cow<Vec<(MinkowskiEdge,MinkowskiFace)>>{
		todo!()
	}
	fn edge_side_faces(&self,edge_id:MinkowskiEdge)->Cow<[MinkowskiFace;2]>{
		todo!()
	}
	fn edge_ends(&self,edge_id:MinkowskiEdge)->Cow<[(MinkowskiVert,MinkowskiFace);2]>{
		todo!()
	}
	fn vert_edges(&self,vert_id:MinkowskiVert)->Cow<Vec<(MinkowskiEdge,MinkowskiFace)>>{
		todo!()
	}
}

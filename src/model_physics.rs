use crate::integer::{Planar64,Planar64Vec3};

pub struct VertId(usize);
pub struct EdgeId(usize);
pub struct FaceId(usize);

//Vertex <-> Edge <-> Face -> Collide
pub enum FEV{
	Face(FaceId),
	Edge(EdgeId),
	Vertex(VertId),
}

//use Unit32 #[repr(C)] for map files
struct Face{
	normal:Planar64Vec3,
	dot:Planar64,
}
struct FaceRefs{
	edges:Vec<EdgeId>,
	verts:Vec<VertId>,
}
struct EdgeRefs{
	f0:FaceId,//left
	f1:FaceId,//right
	v0:VertId,//bottom
	v1:VertId,//top
	v0f:FaceId,//bottom capping face
	v1f:FaceId,//top capping face
}
struct VertRefs{
	faces:Vec<FaceId>,
	edges:Vec<EdgeId>,
}
pub struct PhysicsMesh{
	faces:Vec<Face>,
	face_topology:Vec<FaceRefs>,
	edge_topology:Vec<EdgeRefs>,
	vert_topology:Vec<VertRefs>,
}
impl PhysicsMesh{
	pub fn face_edges(face_id:FaceId)->Vec<EdgeId>{
		todo!()
	}
	pub fn edge_side_faces(edge_id:EdgeId)->[FaceId;2]{
		todo!()
	}
	pub fn edge_end_faces(edge_id:EdgeId)->[FaceId;2]{
		todo!()
	}
	pub fn edge_verts(edge_id:EdgeId)->[VertId;2]{
		todo!()
	}
	pub fn vert_edges(vert_id:VertId)->Vec<EdgeId>{
		todo!()
	}
	pub fn vert_faces(vert_id:VertId)->Vec<FaceId>{
		todo!()
	}
}

//Note that a face on a minkowski mesh refers to a pair of fevs on the meshes it's summed from
//(face,vertex)
//(edge,edge)
//(vertex,face)

pub struct VirtualMesh<'a>{
	mesh0:&'a PhysicsMesh,
	mesh1:&'a PhysicsMesh,
}

impl VirtualMesh<'_>{
	pub fn minkowski_sum<'a>(mesh0:&'a PhysicsMesh,mesh1:&'a PhysicsMesh)->VirtualMesh<'a>{
		VirtualMesh{
			mesh0,
			mesh1,
		}
	}
	pub fn closest_fev(&self,point:Planar64Vec3)->FEV{
		//put some genius code right here
		todo!()
	}
}

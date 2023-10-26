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
	faces:[FaceId;2],//left, right
	verts:[VertId;2],//bottom, top
	vert_faces:[FaceId;2],//bottom, top
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
	pub fn face_edges(&self,face_id:FaceId)->&Vec<EdgeId>{
		&self.face_topology[face_id.0].edges
	}
	pub fn edge_side_faces(&self,edge_id:EdgeId)->&[FaceId;2]{
		&self.edge_topology[edge_id.0].faces
	}
	pub fn edge_end_faces(&self,edge_id:EdgeId)->&[FaceId;2]{
		&self.edge_topology[edge_id.0].vert_faces
	}
	pub fn edge_verts(&self,edge_id:EdgeId)->&[VertId;2]{
		&self.edge_topology[edge_id.0].verts
	}
	pub fn vert_edges(&self,vert_id:VertId)->&Vec<EdgeId>{
		&self.vert_topology[vert_id.0].edges
	}
	pub fn vert_faces(&self,vert_id:VertId)->&Vec<FaceId>{
		&self.vert_topology[vert_id.0].faces
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

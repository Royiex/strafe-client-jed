use crate::integer::{Planar64,Planar64Vec3};

#[derive(Clone,Copy)]
pub struct VertId(usize);
#[derive(Clone,Copy)]
pub struct EdgeId(usize);
#[derive(Clone,Copy)]
pub struct FaceId(usize);

//Vertex <-> Edge <-> Face -> Collide
pub enum FEV{
	Face(FaceId),
	Edge(EdgeId),
	Vert(VertId),
}

//use Unit32 #[repr(C)] for map files
struct Face{
	normal:Planar64Vec3,
	dot:Planar64,
}
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
	face_topology:Vec<FaceRefs>,
	edge_topology:Vec<EdgeRefs>,
	vert_topology:Vec<VertRefs>,
}
impl PhysicsMesh{
	pub fn face_nd(&self,face_id:FaceId)->(Planar64Vec3,Planar64){
		(self.faces[face_id.0].normal,self.faces[face_id.0].dot)
	}
	//ideally I never calculate the vertex position, but I have to for the graphical meshes...
	pub fn face_edges(&self,face_id:FaceId)->&Vec<(EdgeId,FaceId)>{
		&self.face_topology[face_id.0].edges
	}
	pub fn edge_side_faces(&self,edge_id:EdgeId)->&[FaceId;2]{
		&self.edge_topology[edge_id.0].faces
	}
	pub fn edge_ends(&self,edge_id:EdgeId)->&[(VertId,FaceId);2]{
		&self.edge_topology[edge_id.0].verts
	}
	pub fn vert_edges(&self,vert_id:VertId)->&Vec<(EdgeId,FaceId)>{
		&self.vert_topology[vert_id.0].edges
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
	pub fn face_nd(&self,face_id:FaceId)->(Planar64Vec3,Planar64){
		todo!()
	}
	//ideally I never calculate the vertex position, but I have to for the graphical meshes...
	pub fn face_edges(&self,face_id:FaceId)->&Vec<(EdgeId,FaceId)>{
		todo!()
	}
	pub fn edge_side_faces(&self,edge_id:EdgeId)->&[FaceId;2]{
		todo!()
	}
	pub fn edge_ends(&self,edge_id:EdgeId)->&[(VertId,FaceId);2]{
		todo!()
	}
	pub fn vert_edges(&self,vert_id:VertId)->&Vec<(EdgeId,FaceId)>{
		todo!()
	}
}

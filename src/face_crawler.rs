use crate::physics::Body;
use crate::integer::{Time,Planar64Vec3};

struct VertexId(usize);
struct EdgeId(usize);
struct FaceId(usize);

//Vertex <-> Edge <-> Face -> Collide
enum FEV{
	Face(FaceId),
	Edge(EdgeId),
	Vertex(VertexId),
}

struct State{
	time:Time,
	fev:FEV,
}

enum Transition{
	Miss,
	NextState(State),
	Hit(FaceId,Time),
}

impl State{
	fn next_transition(&self,mesh:&VirtualMesh,body:&Body,time_limit:Time)->Transition{
		//conflicting derivative means it crosses in the wrong direction.
		//if the transition time is equal to an already tested transition, do not replace the current best.
		match &self.fev{
			FEV::Face(face_id)=>{
				//test own face collision time, ignoring edges with zero or conflicting derivative
				//test each edge collision time, ignoring edges with zero or conflicting derivative
				//if face: Transition::Hit(Face,Time)
				//if edge: Transition::NextState(State{time,fev:FEV::Edge(edge_id)})
				//if none:
				Transition::Miss
			},
			FEV::Edge(edge_id)=>{
				//test each face collision time, ignoring faces with zero or conflicting derivative
				//test each vertex collision time, ignoring vertices with zero or conflicting derivative
				//if face: Transition::NextState(State{time,fev:FEV::Face(face_id)})
				//if vert: Transition::NextState(State{time,fev:FEV::Vertex(vertex_id)})
				//if none:
				Transition::Miss
			},
			FEV::Vertex(vertex_id)=>{
				//test each edge collision time, ignoring edges with zero or conflicting derivative
				//if some: Transition::NextState(State{time,fev:FEV::Edge(edge_id)})
				//if none:
				Transition::Miss
			},
		}
	}
}

//Note that a face on a minkowski mesh refers to a pair of fevs on the meshes it's summed from
//(face,vertex)
//(edge,edge)
//(vertex,face)

struct VirtualMesh<'a>{
	mesh0:&'a PhysicsMesh,
	mesh1:&'a PhysicsMesh,
}

impl VirtualMesh<'_>{
	pub fn minkowski_sum<'a>(mesh0:&PhysicsMesh,mesh1:&PhysicsMesh)->VirtualMesh<'a>{
		Self{
			mesh0,
			mesh1,
		}
	}
	fn closest_fev(&self,point:Planar64Vec3)->FEV{
		//put some genius code right here
		todo!()
	}
	pub fn predict_collision(&self,relative_body:&Body,time_limit:Time)->Option<(FaceId,Time)>{
		let mut state=State{
			time:relative_body.time,
			fev:self.closest_fev(relative_body.position),
		};
		//it would be possible to write down the point of closest approach...
		loop{
			match state.next_transition(self,relative_body,time_limit){
				Transition::Miss=>return None,
				Transition::NextState(next_state)=>state=next_state,
				Transition::Hit(hit_face,hit_time)=>return Some((hit_face,hit_time)),
			}
		}
	}
}

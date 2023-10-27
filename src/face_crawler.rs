use crate::physics::Body;
use crate::model_physics::{VirtualMesh,FEV,FaceId};
use crate::integer::{Time,Planar64,Planar64Vec3};
use crate::zeroes::zeroes2;

struct State{
	fev:FEV,
	time:Time,
}

enum Transition{
	Miss,
	Next(FEV,Time),
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

pub fn predict_collision(mesh:&VirtualMesh,relative_body:&Body,time_limit:Time)->Option<(FaceId,Time)>{
	let mut state=State{
		fev:mesh.closest_fev(relative_body.position),
		time:relative_body.time,
	};
	//it would be possible to write down the point of closest approach...
	loop{
		match state.next_transition(mesh,relative_body,time_limit){
			Transition::Miss=>return None,
			Transition::Next(fev,time)=>(state.fev,state.time)=(fev,time),
			Transition::Hit(face,time)=>return Some((face,time)),
		}
	}
}

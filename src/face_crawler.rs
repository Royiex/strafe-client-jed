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
		let mut best_time=time_limit;
		let mut best_transtition=Transition::Miss;
		match &self.fev{
			&FEV::Face(face_id)=>{
				//test own face collision time, ignoring roots with zero or conflicting derivative
				//n=face.normal d=face.dot
				//n.a t^2+n.v t+n.p-d==0
				let (n,d)=mesh.face_nd(face_id);
				for t in zeroes2((n.dot(body.position)-d)*2,n.dot(body.velocity)*2,n.dot(body.acceleration)){
					let t=body.time+Time::from(t);
					if self.time<t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
						best_time=t;
						best_transtition=Transition::Hit(face_id,t);
					}
				}
				//test each edge collision time, ignoring roots with zero or conflicting derivative
				for &(edge_id,test_face_id) in mesh.face_edges(face_id){
					let (n,d)=mesh.face_nd(test_face_id);
					for t in zeroes2((n.dot(body.position)-d)*2,n.dot(body.velocity)*2,n.dot(body.acceleration)){
						let t=body.time+Time::from(t);
						if self.time<t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
							best_time=t;
							best_transtition=Transition::Next(FEV::Edge(edge_id),t);
							break;
						}
					}
				}
				//if none:
			},
			&FEV::Edge(edge_id)=>{
				//test each face collision time, ignoring roots with zero or conflicting derivative
				for &test_face_id in mesh.edge_side_faces(edge_id){
					let (n,d)=mesh.face_nd(test_face_id);
					for t in zeroes2((n.dot(body.position)-d)*2,n.dot(body.velocity)*2,n.dot(body.acceleration)){
						let t=body.time+Time::from(t);
						if self.time<t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
							best_time=t;
							best_transtition=Transition::Next(FEV::Face(test_face_id),t);
							break;
						}
					}
				}
				//test each vertex collision time, ignoring roots with zero or conflicting derivative
				for &(vert_id,test_face_id) in mesh.edge_ends(edge_id){
					let (n,d)=mesh.face_nd(test_face_id);
					for t in zeroes2((n.dot(body.position)-d)*2,n.dot(body.velocity)*2,n.dot(body.acceleration)){
						let t=body.time+Time::from(t);
						if self.time<t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
							best_time=t;
							best_transtition=Transition::Next(FEV::Vert(vert_id),t);
							break;
						}
					}
				}
				//if none:
			},
			&FEV::Vert(vertex_id)=>{
				//test each edge collision time, ignoring roots with zero or conflicting derivative
				for &(edge_id,test_face_id) in mesh.vert_edges(vertex_id){
					let (n,d)=mesh.face_nd(test_face_id);
					for t in zeroes2((n.dot(body.position)-d)*2,n.dot(body.velocity)*2,n.dot(body.acceleration)){
						let t=body.time+Time::from(t);
						if self.time<t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
							best_time=t;
							best_transtition=Transition::Next(FEV::Edge(edge_id),t);
							break;
						}
					}
				}
				//if none:
			},
		}
		best_transtition
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

pub fn predict_collision_end(mesh:&VirtualMesh,relative_body:&Body,time_limit:Time,c:&crate::physics::RelativeCollision)->Option<(FaceId,Time)>{
	//imagine the mesh without the collision face
	//no algorithm needed, there is only one state and three cases (Face,Edge,None)
	//determine when it passes an edge ("sliding off" case) or if it leaves the surface directly
	//the state can be constructed from the RelativeCollision directly
	None
}

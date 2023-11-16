use crate::physics::Body;
use crate::model_physics::{FEV,MeshQuery};
use crate::integer::{Time,Planar64};
use crate::zeroes::zeroes2;

enum Transition<F,E,V>{
	Miss,
	Next(FEV<F,E,V>,Time),
	Hit(F,Time),
}

	pub fn next_transition_body<F:Copy,E:Copy,V:Copy>(fev:&FEV<F,E,V>,time:Time,mesh:&impl MeshQuery<F,E,V>,body:&Body,time_limit:Time)->Transition<F,E,V>{
		//conflicting derivative means it crosses in the wrong direction.
		//if the transition time is equal to an already tested transition, do not replace the current best.
		let mut best_time=time_limit;
		let mut best_transtition=Transition::Miss;
		match fev{
			&FEV::<F,E,V>::Face(face_id)=>{
				//test own face collision time, ignoring roots with zero or conflicting derivative
				//n=face.normal d=face.dot
				//n.a t^2+n.v t+n.p-d==0
				let (n,d)=mesh.face_nd(face_id);
				for t in zeroes2((n.dot(body.position)-d)*2,n.dot(body.velocity)*2,n.dot(body.acceleration)){
					let t=body.time+Time::from(t);
					if time<t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
						best_time=t;
						best_transtition=Transition::Hit(face_id,t);
					}
				}
				//test each edge collision time, ignoring roots with zero or conflicting derivative
				for &edge_id in mesh.face_edges(face_id).iter(){
					let edge_n=mesh.edge_n(edge_id);
					let n=n.cross(edge_n);
					//picking a vert randomly is terrible
					let d=n.dot(mesh.vert(mesh.edge_verts(edge_id)[0]));
					for t in zeroes2((n.dot(body.position)-d)*2,n.dot(body.velocity)*2,n.dot(body.acceleration)){
						let t=body.time+Time::from(t);
						if time<t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
							best_time=t;
							best_transtition=Transition::Next(FEV::<F,E,V>::Edge(edge_id),t);
							break;
						}
					}
				}
				//if none:
			},
			&FEV::<F,E,V>::Edge(edge_id)=>{
				//test each face collision time, ignoring roots with zero or conflicting derivative
				let edge_n=mesh.edge_n(edge_id);
				for &test_face_id in mesh.edge_faces(edge_id).iter(){
					let face_n=mesh.face_nd(test_face_id).0;
					let n=edge_n.cross(face_n);
					let d=n.dot(mesh.vert(mesh.edge_verts(edge_id)[0]));
					for t in zeroes2((n.dot(body.position)-d)*2,n.dot(body.velocity)*2,n.dot(body.acceleration)){
						let t=body.time+Time::from(t);
						if time<t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
							best_time=t;
							best_transtition=Transition::Next(FEV::<F,E,V>::Face(test_face_id),t);
							break;
						}
					}
				}
				//test each vertex collision time, ignoring roots with zero or conflicting derivative
				let n=mesh.edge_n(edge_id);
				for &vert_id in mesh.edge_verts(edge_id).iter(){
					let d=n.dot(mesh.vert(vert_id));
					for t in zeroes2((n.dot(body.position)-d)*2,n.dot(body.velocity)*2,n.dot(body.acceleration)){
						let t=body.time+Time::from(t);
						if time<t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
							best_time=t;
							best_transtition=Transition::Next(FEV::<F,E,V>::Vert(vert_id),t);
							break;
						}
					}
				}
				//if none:
			},
			&FEV::<F,E,V>::Vert(vert_id)=>{
				//test each edge collision time, ignoring roots with zero or conflicting derivative
				for &edge_id in mesh.vert_edges(vert_id).iter(){
					let n=mesh.edge_n(edge_id);
					let d=n.dot(mesh.vert(vert_id));
					for t in zeroes2((n.dot(body.position)-d)*2,n.dot(body.velocity)*2,n.dot(body.acceleration)){
						let t=body.time+Time::from(t);
						if time<t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
							best_time=t;
							best_transtition=Transition::Next(FEV::<F,E,V>::Edge(edge_id),t);
							break;
						}
					}
				}
				//if none:
			},
		}
		best_transtition
	}
pub fn crawl_fev_body<F:Copy,E:Copy,V:Copy>(mut fev:FEV<F,E,V>,mesh:&impl MeshQuery<F,E,V>,relative_body:&Body,time_limit:Time)->Option<(F,Time)>{	
	let mut time=relative_body.time;
	loop{
		match next_transition_body(&fev,time,mesh,relative_body,time_limit){
			Transition::Miss=>return None,
			Transition::Next(next_fev,next_time)=>(fev,time)=(next_fev,next_time),
			Transition::Hit(face,time)=>return Some((face,time)),
		}
	}
}

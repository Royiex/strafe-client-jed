use crate::physics::Body;
use crate::model_physics::{FEV,MeshQuery,DirectedEdge};
use crate::integer::{Time,Planar64};
use crate::zeroes::zeroes2;

enum Transition<F,E:DirectedEdge,V>{
	Miss,
	Next(FEV<F,E,V>,Time),
	Hit(F,Time),
}

	fn next_transition<F:Copy,E:Copy+DirectedEdge,V:Copy>(fev:&FEV<F,E,V>,time:Time,mesh:&impl MeshQuery<F,E,V>,body:&Body,time_limit:Time)->Transition<F,E,V>{
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
					if time<=t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
						best_time=t;
						best_transtition=Transition::Hit(face_id,t);
						break;
					}
				}
				//test each edge collision time, ignoring roots with zero or conflicting derivative
				for &directed_edge_id in mesh.face_edges(face_id).iter(){
					let edge_n=mesh.directed_edge_n(directed_edge_id);
					let n=n.cross(edge_n);
					let verts=mesh.edge_verts(directed_edge_id.as_undirected());
					//WARNING: d is moved out of the *2 block because of adding two vertices!
					for t in zeroes2(n.dot(body.position*2-(mesh.vert(verts[0])+mesh.vert(verts[1]))),n.dot(body.velocity)*2,n.dot(body.acceleration)){
						let t=body.time+Time::from(t);
						if time<=t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
							best_time=t;
							best_transtition=Transition::Next(FEV::<F,E,V>::Edge(directed_edge_id.as_undirected()),t);
							break;
						}
					}
				}
				//if none:
			},
			&FEV::<F,E,V>::Edge(edge_id)=>{
				//test each face collision time, ignoring roots with zero or conflicting derivative
				let edge_n=mesh.edge_n(edge_id);
				let edge_verts=mesh.edge_verts(edge_id);
				let delta_pos=body.position*2-(mesh.vert(edge_verts[0])+mesh.vert(edge_verts[1]));
				for (i,&edge_face_id) in mesh.edge_faces(edge_id).iter().enumerate(){
					let face_n=mesh.face_nd(edge_face_id).0;
					//edge_n gets parity from the order of edge_faces
					let n=face_n.cross(edge_n)*((i as i64)*2-1);
					//WARNING yada yada d *2
					for t in zeroes2(n.dot(delta_pos),n.dot(body.velocity)*2,n.dot(body.acceleration)){
						let t=body.time+Time::from(t);
						if time<=t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
							best_time=t;
							best_transtition=Transition::Next(FEV::<F,E,V>::Face(edge_face_id),t);
							break;
						}
					}
				}
				//test each vertex collision time, ignoring roots with zero or conflicting derivative
				for (i,&vert_id) in edge_verts.iter().enumerate(){
					//vertex normal gets parity from vert index
					let n=edge_n*(1-2*(i as i64));
					for t in zeroes2((n.dot(body.position-mesh.vert(vert_id)))*2,n.dot(body.velocity)*2,n.dot(body.acceleration)){
						let t=body.time+Time::from(t);
						if time<=t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
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
				for &directed_edge_id in mesh.vert_edges(vert_id).iter(){
					//edge is directed away from vertex, but we want the dot product to turn out negative
					let n=-mesh.directed_edge_n(directed_edge_id);
					for t in zeroes2((n.dot(body.position-mesh.vert(vert_id)))*2,n.dot(body.velocity)*2,n.dot(body.acceleration)){
						let t=body.time+Time::from(t);
						if time<=t&&t<best_time&&n.dot(body.extrapolated_velocity(t))<Planar64::ZERO{
							best_time=t;
							best_transtition=Transition::Next(FEV::<F,E,V>::Edge(directed_edge_id.as_undirected()),t);
							break;
						}
					}
				}
				//if none:
			},
		}
		best_transtition
	}
pub enum CrawlResult<F,E:DirectedEdge,V>{
	Miss(FEV<F,E,V>),
	Hit(F,Time),
}
pub fn crawl_fev<F:Copy,E:Copy+DirectedEdge,V:Copy>(mut fev:FEV<F,E,V>,mesh:&impl MeshQuery<F,E,V>,relative_body:&Body,start_time:Time,time_limit:Time)->CrawlResult<F,E,V>{	
	let mut time=start_time;
	for _ in 0..20{
		match next_transition(&fev,time,mesh,relative_body,time_limit){
			Transition::Miss=>return CrawlResult::Miss(fev),
			Transition::Next(next_fev,next_time)=>(fev,time)=(next_fev,next_time),
			Transition::Hit(face,time)=>return CrawlResult::Hit(face,time),
		}
	}
	//TODO: fix all bugs
	println!("Too many iterations!  Using default behaviour instead of crashing...");
	CrawlResult::Miss(fev)
}

use crate::physics::Body;
use crate::integer::Time;

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
	fn next_transition(&self,mesh:&VirtualMesh,time_limit:Time)->Transition{
		//conflicting derivative means it crosses in the wrong direction.
		match self.fev{
			FEV::Face(face_id)=>{
				//test own face collision time
				//test each edge collision time, ignoring edges with zero or conflicting derivative
				//if face: return (Face,Time)
				//if edge: goto edge
			},
			FEV::Edge(edge_id)=>{
				//test each face collision time, ignoring faces with zero or conflicting derivative
				//test each vertex collision time, ignoring vertices with zero or conflicting derivative
				//if face: goto face
				//if vertex: goto vertex
			},
			FEV::Vertex(vertex_id)=>{
				//test each edge collision time, ignoring edges with zero or conflicting derivative
				//goto edge
			},
		}
	}
}

//Note that a face on a minkowski mesh refers to a pair of fevs on the meshes it's summed from
//(face,vertex)
//(edge,edge)
//(vertex,face)

struct VirtualMesh{
	//???
}

impl VirtualMesh{
	pub fn minkowski_sum(mesh0:&PhysicsMesh,mesh1:&PhysicsMesh)->Self{
		//?????
	}
	pub fn predict_collision(&self,body:&Body,time:Time,time_limit:Time)->Option<(FaceId,Time)>{
		//mesh.closest_point returns if the point is on a face, edge or vertex
		let (point,fev)=self.closest_point(body.extrapolated_position(time));
		let mut state=State{
			time,
			fev,
		};
		//it would be possible to write down the point of closest approach...
		loop{
			match state.next_transition(self,time_limit){
				Transition::Miss=>return None,
				Transition::NextState(next_state)=>state=next_state,
				Transition::Hit(hit_face,hit_time)=>return Some((hit_face,hit_time)),
			}
		}
	}
}

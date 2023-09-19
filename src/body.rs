use crate::{instruction::{InstructionEmitter, InstructionConsumer, TimedInstruction}, zeroes::zeroes2};

#[derive(Debug)]
pub enum PhysicsInstruction {
	CollisionStart(RelativeCollision),
	CollisionEnd(RelativeCollision),
	SetControlDir(glam::Vec3),
	StrafeTick,
	Jump,
	SetWalkTargetVelocity(glam::Vec3),
	RefreshWalkTarget,
	ReachWalkTargetVelocity,
	// Water,
	// Spawn(
	// 	Option<SpawnId>,
	// 	bool,//true = Trigger; false = teleport
	// 	bool,//true = Force
	// )
}

pub struct Body {
	position: glam::Vec3,//I64 where 2^32 = 1 u
	velocity: glam::Vec3,//I64 where 2^32 = 1 u/s
	acceleration: glam::Vec3,//I64 where 2^32 = 1 u/s/s
	time: TIME,//nanoseconds x xxxxD!
}
trait MyHash{
	fn hash(&self) -> u64;
}
impl MyHash for Body {
    fn hash(&self) -> u64 {
		let mut hasher=std::collections::hash_map::DefaultHasher::new();
        for &el in self.position.as_ref().iter() {
            std::hash::Hasher::write(&mut hasher, el.to_ne_bytes().as_slice());
        }
        for &el in self.velocity.as_ref().iter() {
            std::hash::Hasher::write(&mut hasher, el.to_ne_bytes().as_slice());
        }
        for &el in self.acceleration.as_ref().iter() {
             std::hash::Hasher::write(&mut hasher, el.to_ne_bytes().as_slice());
        }
        std::hash::Hasher::write(&mut hasher, self.time.to_ne_bytes().as_slice());
		return std::hash::Hasher::finish(&hasher);//hash check to see if walk target is valid
    }
}

pub enum MoveRestriction {
	Air,
	Water,
	Ground,
	Ladder,//multiple ladders how
}

enum MouseInterpolation {
	First,//just checks the last value
	Lerp,//lerps between
}

enum InputInstruction {
	MoveMouse(glam::IVec2),
	Jump(bool),
}

struct InputState {
	controls: u32,
	mouse_interpolation: MouseInterpolation,
	time: TIME,
}

impl InputState {
	pub fn get_control(&self,control:u32) -> bool {
		self.controls&control!=0
	}
	pub fn process_instruction(&mut self,ins:InputInstruction){
		match ins {
			InputInstruction::MoveMouse(m) => todo!("set mouse_interpolation"),
			InputInstruction::Jump(b) => todo!("how does info about style modifiers get here"),
		}
	}
}

pub struct MouseInterpolationState {
	interpolation: MouseInterpolation,
	time0: TIME,
	time1: TIME,
	mouse0: glam::IVec2,
	mouse1: glam::IVec2,
}

impl MouseInterpolationState {
	pub fn move_mouse(&mut self,time:TIME,pos:glam::IVec2){
		self.time0=self.time1;
		self.mouse0=self.mouse1;
		self.time1=time;
		self.mouse1=pos;
	}
	pub fn interpolated_position(&self,time:TIME) -> glam::IVec2 {
		match self.interpolation {
			MouseInterpolation::First => self.mouse0,
			MouseInterpolation::Lerp => {
				let m0=self.mouse0.as_i64vec2();
				let m1=self.mouse1.as_i64vec2();
				//these are deltas
				let t1t=(self.time1-time) as i64;
				let tt0=(time-self.time0) as i64;
				let dt=(self.time1-self.time0) as i64;
				((m0*t1t+m1*tt0)/dt).as_ivec2()
			}
		}
	}
}

pub enum WalkEnum{
	Reached,
	Transient,
	Invalid,
}
pub struct WalkState {
	pub target_velocity: glam::Vec3,
	pub target_time: TIME,
	pub state: WalkEnum,
}
impl WalkState {
	pub fn new() -> Self {
		Self{
			target_velocity:glam::Vec3::ZERO,
			target_time:0,
			state:WalkEnum::Invalid,
		}
	}
}

pub struct PhysicsState {
	pub body: Body,
	pub hitbox_halfsize: glam::Vec3,
	pub contacts: std::collections::HashSet::<RelativeCollision>,
	//pub intersections: Vec<ModelId>,
	//temp
	pub models_cringe_clone: Vec<Model>,
	pub temp_control_dir: glam::Vec3,
	//camera must exist in state because wormholes modify the camera, also camera punch
	//pub camera: Camera,
	//pub mouse_interpolation: MouseInterpolationState,
	pub time: TIME,
	pub strafe_tick_num: TIME,
	pub strafe_tick_den: TIME,
	pub tick: u32,
	pub mv: f32,
	pub walk: WalkState,
	pub walkspeed: f32,
	pub friction: f32,
	pub walk_accel:  f32,
	pub gravity: glam::Vec3,
	pub grounded: bool,
	pub jump_trying: bool,
}

#[derive(Debug,Clone,Copy,Hash,Eq,PartialEq)]
pub enum AabbFace{
	Right,//+X
	Top,
	Back,
	Left,
	Bottom,
	Front,
}

pub struct Aabb {
	min: glam::Vec3,
	max: glam::Vec3,
}

impl Aabb {
	// const FACE_DATA: [[f32; 3]; 6] = [
	// 	[0.0f32, 0., 1.],
	// 	[0.0f32, 0., -1.],
	// 	[1.0f32, 0., 0.],
	// 	[-1.0f32, 0., 0.],
	// 	[0.0f32, 1., 0.],
	// 	[0.0f32, -1., 0.],
	// ];
	const VERTEX_DATA: [glam::Vec3; 8] = [
		glam::vec3(1., -1., -1.),
		glam::vec3(1., 1., -1.),
		glam::vec3(1., 1., 1.),
		glam::vec3(1., -1., 1.),
		glam::vec3(-1., -1., 1.),
		glam::vec3(-1., 1., 1.),
		glam::vec3(-1., 1., -1.),
		glam::vec3(-1., -1., -1.),
	];
	const VERTEX_DATA_RIGHT: [glam::Vec3; 4] = [
		glam::vec3(1., -1., -1.),
		glam::vec3(1., 1., -1.),
		glam::vec3(1., 1., 1.),
		glam::vec3(1., -1., 1.),
	];
	const VERTEX_DATA_TOP: [glam::Vec3; 4] = [
		glam::vec3(1., 1., -1.),
		glam::vec3(-1., 1., -1.),
		glam::vec3(-1., 1., 1.),
		glam::vec3(1., 1., 1.),
	];
	const VERTEX_DATA_BACK: [glam::Vec3; 4] = [
		glam::vec3(-1., -1., 1.),
		glam::vec3(1., -1., 1.),
		glam::vec3(1., 1., 1.),
		glam::vec3(-1., 1., 1.),
	];
	const VERTEX_DATA_LEFT: [glam::Vec3; 4] = [
		glam::vec3(-1., -1., 1.),
		glam::vec3(-1., 1., 1.),
		glam::vec3(-1., 1., -1.),
		glam::vec3(-1., -1., -1.),
	];
	const VERTEX_DATA_BOTTOM: [glam::Vec3; 4] = [
		glam::vec3(1., -1., 1.),
		glam::vec3(-1., -1., 1.),
		glam::vec3(-1., -1., -1.),
		glam::vec3(1., -1., -1.),
	];
	const VERTEX_DATA_FRONT: [glam::Vec3; 4] = [
		glam::vec3(-1., 1., -1.),
		glam::vec3(1., 1., -1.),
		glam::vec3(1., -1., -1.),
		glam::vec3(-1., -1., -1.),
	];

	pub fn new() -> Self {
		Self {min: glam::Vec3::INFINITY,max: glam::Vec3::NEG_INFINITY}
	}

	pub fn grow(&mut self, point:glam::Vec3){
		self.min=self.min.min(point);
		self.max=self.max.max(point);
	}

	pub fn normal(face:AabbFace) -> glam::Vec3 {
		match face {
			AabbFace::Right => glam::vec3(1.,0.,0.),
			AabbFace::Top => glam::vec3(0.,1.,0.),
			AabbFace::Back => glam::vec3(0.,0.,1.),
			AabbFace::Left => glam::vec3(-1.,0.,0.),
			AabbFace::Bottom => glam::vec3(0.,-1.,0.),
			AabbFace::Front => glam::vec3(0.,0.,-1.),
		}
	}
	pub fn unit_vertices() -> [glam::Vec3;8] {
		return Self::VERTEX_DATA;
	}
	pub fn unit_face_vertices(face:AabbFace) -> [glam::Vec3;4] {
		match face {
			AabbFace::Right => Self::VERTEX_DATA_RIGHT,
			AabbFace::Top => Self::VERTEX_DATA_TOP,
			AabbFace::Back => Self::VERTEX_DATA_BACK,
			AabbFace::Left => Self::VERTEX_DATA_LEFT,
			AabbFace::Bottom => Self::VERTEX_DATA_BOTTOM,
			AabbFace::Front => Self::VERTEX_DATA_FRONT,
		}
	}
}

//pretend to be using what we want to eventually do
type TreyMeshFace = AabbFace;
type TreyMesh = Aabb;

pub struct Model {
	//A model is a thing that has a hitbox. can be represented by a list of TreyMesh-es
	//in this iteration, all it needs is extents.
	transform: glam::Mat4,
}

impl Model {
	pub fn new(transform:glam::Mat4) -> Self {
		Self{transform}
	}
	pub fn unit_vertices(&self) -> [glam::Vec3;8] {
		Aabb::unit_vertices()
	}
	pub fn mesh(&self) -> TreyMesh {
		let mut aabb=Aabb::new();
		for &vertex in self.unit_vertices().iter() {
			aabb.grow(glam::Vec4Swizzles::xyz(self.transform*vertex.extend(1.0)));
		}
		return aabb;
	}
	pub fn unit_face_vertices(&self,face:TreyMeshFace) -> [glam::Vec3;4] {
		Aabb::unit_face_vertices(face)
	}
	pub fn face_mesh(&self,face:TreyMeshFace) -> TreyMesh {
		let mut aabb=self.mesh();
		//in this implementation face = worldspace aabb face
		match face {
			AabbFace::Right => aabb.min.x=aabb.max.x,
			AabbFace::Top => aabb.min.y=aabb.max.y,
			AabbFace::Back => aabb.min.z=aabb.max.z,
			AabbFace::Left => aabb.max.x=aabb.min.x,
			AabbFace::Bottom => aabb.max.y=aabb.min.y,
			AabbFace::Front => aabb.max.z=aabb.min.z,
		}
		return aabb;
	}
	pub fn face_normal(&self,face:TreyMeshFace) -> glam::Vec3 {
		glam::Vec4Swizzles::xyz(Aabb::normal(face).extend(0.0))//this is wrong for scale
	}
}

//need non-face (full model) variant for CanCollide false objects
//OR have a separate list from contacts for model intersection
#[derive(Debug,Clone,Eq,Hash,PartialEq)]
pub struct RelativeCollision {
	face: TreyMeshFace,//just an id
	model: u32,//using id to avoid lifetimes
}

impl RelativeCollision {
	pub fn mesh(&self,models:&Vec<Model>) -> TreyMesh {
		return models.get(self.model as usize).unwrap().face_mesh(self.face)
	}
	pub fn normal(&self,models:&Vec<Model>) -> glam::Vec3 {
		return models.get(self.model as usize).unwrap().face_normal(self.face)
	}
}

pub type TIME = i64;

impl Body {
	pub fn with_pva(position:glam::Vec3,velocity:glam::Vec3,acceleration:glam::Vec3) -> Self {
		Self{
			position,
			velocity,
			acceleration,
			time: 0,
		}
	}
	pub fn extrapolated_position(&self,time: TIME)->glam::Vec3{
		let dt=(time-self.time) as f64/1_000_000_000f64;
		self.position+self.velocity*(dt as f32)+self.acceleration*((0.5*dt*dt) as f32)
	}
	pub fn extrapolated_velocity(&self,time: TIME)->glam::Vec3{
		let dt=(time-self.time) as f64/1_000_000_000f64;
		self.velocity+self.acceleration*(dt as f32)
	}
	pub fn advance_time(&mut self, time: TIME){
		self.position=self.extrapolated_position(time);
		self.velocity=self.extrapolated_velocity(time);
		self.time=time;
	}
}

impl PhysicsState {
	//tickless gaming
	pub fn run(&mut self, time_limit:TIME){
		//prepare is ommitted - everything is done via instructions.
		while let Some(instruction) = self.next_instruction(time_limit) {//collect
			//advance
			//self.advance_time(instruction.time);
			//process
			self.process_instruction(instruction);
			//write hash lol
		}
	}

	pub fn advance_time(&mut self, time: TIME){
		self.body.advance_time(time);
		self.time=time;
	}

	fn contact_constrain_velocity(&self,velocity:&mut glam::Vec3){
		for contact in self.contacts.iter() {
			let n=contact.normal(&self.models_cringe_clone);
			let d=velocity.dot(n);
			if d<0f32{
				(*velocity)-=d/n.length_squared()*n;
			}
		}
	}
	fn contact_constrain_acceleration(&self,acceleration:&mut glam::Vec3){
		for contact in self.contacts.iter() {
			let n=contact.normal(&self.models_cringe_clone);
			let d=acceleration.dot(n);
			if d<0f32{
				(*acceleration)-=d/n.length_squared()*n;
			}
		}
	}

	fn next_strafe_instruction(&self) -> Option<TimedInstruction<PhysicsInstruction>> {
		return Some(TimedInstruction{
			time:(self.time*self.strafe_tick_num/self.strafe_tick_den+1)*self.strafe_tick_den/self.strafe_tick_num,
			//only poll the physics if there is a before and after mouse event
			instruction:PhysicsInstruction::StrafeTick
		});
	}

	//state mutated on collision:
	//Accelerator
	//stair step-up

	//state mutated on instruction
	//change fly acceleration (fly_sustain)
	//change fly velocity

	//generic event emmiters
	//PlatformStandTime
	//walk/swim/air/ladder sounds
	//VState?

	//falling under the map
	// fn next_respawn_instruction(&self) -> Option<TimedInstruction<PhysicsInstruction>> {
	// 	if self.body.position<self.world.min_y {
	// 		return Some(TimedInstruction{
	// 			time:self.time,
	// 			instruction:PhysicsInstruction::Trigger(None)
	// 		});
	// 	}
	// }

	// fn next_water_instruction(&self) -> Option<TimedInstruction<PhysicsInstruction>> {
	// 	return Some(TimedInstruction{
	// 		time:(self.time*self.strafe_tick_num/self.strafe_tick_den+1)*self.strafe_tick_den/self.strafe_tick_num,
	// 		//only poll the physics if there is a before and after mouse event
	// 		instruction:PhysicsInstruction::Water
	// 	});
	// }

	fn next_walk_instruction(&self) -> Option<TimedInstruction<PhysicsInstruction>> {
		//check if you have a valid walk state and create an instruction
		if self.grounded{
			match self.walk.state{
				WalkEnum::Transient=>Some(TimedInstruction{
					time:self.walk.target_time,
					instruction:PhysicsInstruction::ReachWalkTargetVelocity
				}),
				WalkEnum::Invalid=>Some(TimedInstruction{
					time:self.time,
					instruction:PhysicsInstruction::RefreshWalkTarget,
				}),
				WalkEnum::Reached=>None,
			}
		}else{
			return None;
		}
	}
	fn mesh(&self) -> TreyMesh {
		let mut aabb=Aabb::new();
		for vertex in Aabb::unit_vertices(){
			aabb.grow(self.body.position+self.hitbox_halfsize*vertex);
		}
		aabb
	}
	fn predict_collision_end(&self,time:TIME,time_limit:TIME,collision_data:&RelativeCollision) -> Option<TimedInstruction<PhysicsInstruction>> {
		//must treat cancollide false objects differently: you may not exit through the same face you entered.
		//RelativeCollsion must reference the full model instead of a particular face
		//this is Ctrl+C Ctrl+V of predict_collision_start but with v=-v before the calc and t=-t after the calc
		//find best t
		let mut best_time=time_limit;
		let mut exit_face:Option<TreyMeshFace>=None;
		let mesh0=self.mesh();
		let mesh1=self.models_cringe_clone.get(collision_data.model as usize).unwrap().mesh();
		let (v,a)=(-self.body.velocity,self.body.acceleration);
		//collect x
		match collision_data.face {
			AabbFace::Top|AabbFace::Back|AabbFace::Bottom|AabbFace::Front=>{
				for t in zeroes2(mesh0.max.x-mesh1.min.x,v.x,0.5*a.x) {
					//negative t = back in time
					//must be moving towards surface to collide
					//must beat the current soonest collision time
					//must be moving towards surface
					let t_time=self.body.time+((-t as f64)*1_000_000_000f64) as TIME;
					if time<=t_time&&t_time<best_time&&0f32<v.x+a.x*-t{
						//collect valid t
						best_time=t_time;
						exit_face=Some(TreyMeshFace::Left);
					}
				}
				for t in zeroes2(mesh0.min.x-mesh1.max.x,v.x,0.5*a.x) {
					//negative t = back in time
					//must be moving towards surface to collide
					//must beat the current soonest collision time
					//must be moving towards surface
					let t_time=self.body.time+((-t as f64)*1_000_000_000f64) as TIME;
					if time<=t_time&&t_time<best_time&&v.x+a.x*-t<0f32{
						//collect valid t
						best_time=t_time;
						exit_face=Some(TreyMeshFace::Right);
					}
				}
			},
			AabbFace::Left=>{
				//generate event if v.x<0||a.x<0
				if -v.x<0f32{
					best_time=time;
					exit_face=Some(TreyMeshFace::Left);
				}
			},
			AabbFace::Right=>{
				//generate event if 0<v.x||0<a.x
				if 0f32<(-v.x){
					best_time=time;
					exit_face=Some(TreyMeshFace::Right);
				}
			},
		}
		//collect y
		match collision_data.face {
			AabbFace::Left|AabbFace::Back|AabbFace::Right|AabbFace::Front=>{
				for t in zeroes2(mesh0.max.y-mesh1.min.y,v.y,0.5*a.y) {
					//negative t = back in time
					//must be moving towards surface to collide
					//must beat the current soonest collision time
					//must be moving towards surface
					let t_time=self.body.time+((-t as f64)*1_000_000_000f64) as TIME;
					if time<=t_time&&t_time<best_time&&0f32<v.y+a.y*-t{
						//collect valid t
						best_time=t_time;
						exit_face=Some(TreyMeshFace::Bottom);
					}
				}
				for t in zeroes2(mesh0.min.y-mesh1.max.y,v.y,0.5*a.y) {
					//negative t = back in time
					//must be moving towards surface to collide
					//must beat the current soonest collision time
					//must be moving towards surface
					let t_time=self.body.time+((-t as f64)*1_000_000_000f64) as TIME;
					if time<=t_time&&t_time<best_time&&v.y+a.y*-t<0f32{
						//collect valid t
						best_time=t_time;
						exit_face=Some(TreyMeshFace::Top);
					}
				}
			},
			AabbFace::Bottom=>{
				//generate event if v.y<0||a.y<0
				if -v.y<0f32{
					best_time=time;
					exit_face=Some(TreyMeshFace::Bottom);
				}
			},
			AabbFace::Top=>{
				//generate event if 0<v.y||0<a.y
				if 0f32<(-v.y){
					best_time=time;
					exit_face=Some(TreyMeshFace::Top);
				}
			},
		}
		//collect z
		match collision_data.face {
			AabbFace::Left|AabbFace::Bottom|AabbFace::Right|AabbFace::Top=>{
				for t in zeroes2(mesh0.max.z-mesh1.min.z,v.z,0.5*a.z) {
					//negative t = back in time
					//must be moving towards surface to collide
					//must beat the current soonest collision time
					//must be moving towards surface
					let t_time=self.body.time+((-t as f64)*1_000_000_000f64) as TIME;
					if time<=t_time&&t_time<best_time&&0f32<v.z+a.z*-t{
						//collect valid t
						best_time=t_time;
						exit_face=Some(TreyMeshFace::Front);
					}
				}
				for t in zeroes2(mesh0.min.z-mesh1.max.z,v.z,0.5*a.z) {
					//negative t = back in time
					//must be moving towards surface to collide
					//must beat the current soonest collision time
					//must be moving towards surface
					let t_time=self.body.time+((-t as f64)*1_000_000_000f64) as TIME;
					if time<=t_time&&t_time<best_time&&v.z+a.z*-t<0f32{
						//collect valid t
						best_time=t_time;
						exit_face=Some(TreyMeshFace::Back);
					}
				}
			},
			AabbFace::Front=>{
				//generate event if v.z<0||a.z<0
				if -v.z<0f32{
					best_time=time;
					exit_face=Some(TreyMeshFace::Front);
				}
			},
			AabbFace::Back=>{
				//generate event if 0<v.z||0<a.z
				if 0f32<(-v.z){
					best_time=time;
					exit_face=Some(TreyMeshFace::Back);
				}
			},
		}
		//generate instruction
		if let Some(face) = exit_face{
			return Some(TimedInstruction {
				time: best_time,
				instruction: PhysicsInstruction::CollisionEnd(collision_data.clone())
			})
		}
		None
	}
	fn predict_collision_start(&self,time:TIME,time_limit:TIME,model_id:u32) -> Option<TimedInstruction<PhysicsInstruction>> {
		//find best t
		let mut best_time=time_limit;
		let mut best_face:Option<TreyMeshFace>=None;
		let mesh0=self.mesh();
		let mesh1=self.models_cringe_clone.get(model_id as usize).unwrap().mesh();
		let (p,v,a)=(self.body.position,self.body.velocity,self.body.acceleration);
		//collect x
		for t in zeroes2(mesh0.max.x-mesh1.min.x,v.x,0.5*a.x) {
			//must collide now or in the future
			//must beat the current soonest collision time
			//must be moving towards surface
			let t_time=self.body.time+((t as f64)*1_000_000_000f64) as TIME;
			if time<=t_time&&t_time<best_time&&0f32<v.x+a.x*t{
				let dp=self.body.extrapolated_position(t_time)-p;
				//faces must be overlapping
				if mesh1.min.y<mesh0.max.y+dp.y&&mesh0.min.y+dp.y<mesh1.max.y&&mesh1.min.z<mesh0.max.z+dp.z&&mesh0.min.z+dp.z<mesh1.max.z {
					//collect valid t
					best_time=t_time;
					best_face=Some(TreyMeshFace::Left);
				}
			}
		}
		for t in zeroes2(mesh0.min.x-mesh1.max.x,v.x,0.5*a.x) {
			//must collide now or in the future
			//must beat the current soonest collision time
			//must be moving towards surface
			let t_time=self.body.time+((t as f64)*1_000_000_000f64) as TIME;
			if time<=t_time&&t_time<best_time&&v.x+a.x*t<0f32{
				let dp=self.body.extrapolated_position(t_time)-p;
				//faces must be overlapping
				if mesh1.min.y<mesh0.max.y+dp.y&&mesh0.min.y+dp.y<mesh1.max.y&&mesh1.min.z<mesh0.max.z+dp.z&&mesh0.min.z+dp.z<mesh1.max.z {
					//collect valid t
					best_time=t_time;
					best_face=Some(TreyMeshFace::Right);
				}
			}
		}
		//collect y
		for t in zeroes2(mesh0.max.y-mesh1.min.y,v.y,0.5*a.y) {
			//must collide now or in the future
			//must beat the current soonest collision time
			//must be moving towards surface
			let t_time=self.body.time+((t as f64)*1_000_000_000f64) as TIME;
			if time<=t_time&&t_time<best_time&&0f32<v.y+a.y*t{
				let dp=self.body.extrapolated_position(t_time)-p;
				//faces must be overlapping
				if mesh1.min.x<mesh0.max.x+dp.x&&mesh0.min.x+dp.x<mesh1.max.x&&mesh1.min.z<mesh0.max.z+dp.z&&mesh0.min.z+dp.z<mesh1.max.z {
					//collect valid t
					best_time=t_time;
					best_face=Some(TreyMeshFace::Bottom);
				}
			}
		}
		for t in zeroes2(mesh0.min.y-mesh1.max.y,v.y,0.5*a.y) {
			//must collide now or in the future
			//must beat the current soonest collision time
			//must be moving towards surface
			let t_time=self.body.time+((t as f64)*1_000_000_000f64) as TIME;
			if time<=t_time&&t_time<best_time&&v.y+a.y*t<0f32{
				let dp=self.body.extrapolated_position(t_time)-p;
				//faces must be overlapping
				if mesh1.min.x<mesh0.max.x+dp.x&&mesh0.min.x+dp.x<mesh1.max.x&&mesh1.min.z<mesh0.max.z+dp.z&&mesh0.min.z+dp.z<mesh1.max.z {
					//collect valid t
					best_time=t_time;
					best_face=Some(TreyMeshFace::Top);
				}
			}
		}
		//collect z
		for t in zeroes2(mesh0.max.z-mesh1.min.z,v.z,0.5*a.z) {
			//must collide now or in the future
			//must beat the current soonest collision time
			//must be moving towards surface
			let t_time=self.body.time+((t as f64)*1_000_000_000f64) as TIME;
			if time<=t_time&&t_time<best_time&&0f32<v.z+a.z*t{
				let dp=self.body.extrapolated_position(t_time)-p;
				//faces must be overlapping
				if mesh1.min.y<mesh0.max.y+dp.y&&mesh0.min.y+dp.y<mesh1.max.y&&mesh1.min.x<mesh0.max.x+dp.x&&mesh0.min.x+dp.x<mesh1.max.x {
					//collect valid t
					best_time=t_time;
					best_face=Some(TreyMeshFace::Front);
				}
			}
		}
		for t in zeroes2(mesh0.min.z-mesh1.max.z,v.z,0.5*a.z) {
			//must collide now or in the future
			//must beat the current soonest collision time
			//must be moving towards surface
			let t_time=self.body.time+((t as f64)*1_000_000_000f64) as TIME;
			if time<=t_time&&t_time<best_time&&v.z+a.z*t<0f32{
				let dp=self.body.extrapolated_position(t_time)-p;
				//faces must be overlapping
				if mesh1.min.y<mesh0.max.y+dp.y&&mesh0.min.y+dp.y<mesh1.max.y&&mesh1.min.x<mesh0.max.x+dp.x&&mesh0.min.x+dp.x<mesh1.max.x {
					//collect valid t
					best_time=t_time;
					best_face=Some(TreyMeshFace::Back);
				}
			}
		}
		//generate instruction
		if let Some(face) = best_face{
			return Some(TimedInstruction {
				time: best_time,
				instruction: PhysicsInstruction::CollisionStart(RelativeCollision {
					face,
					model: model_id
				})
			})
		}
		None
	}
}

impl crate::instruction::InstructionEmitter<PhysicsInstruction> for PhysicsState {
	//this little next instruction function can cache its return value and invalidate the cached value by watching the State.
	fn next_instruction(&self,time_limit:TIME) -> Option<TimedInstruction<PhysicsInstruction>> {
		//JUST POLLING!!! NO MUTATION
		let mut collector = crate::instruction::InstructionCollector::new(time_limit);
		//check for collision stop instructions with curent contacts
		for collision_data in self.contacts.iter() {
			collector.collect(self.predict_collision_end(self.time,time_limit,collision_data));
		}
		//check for collision start instructions (against every part in the game with no optimization!!)
		for i in 0..self.models_cringe_clone.len() {
			collector.collect(self.predict_collision_start(self.time,time_limit,i as u32));
		}
		if self.grounded {
			//walk maintenance
			collector.collect(self.next_walk_instruction());
		}else{
			//check to see when the next strafe tick is
			collector.collect(self.next_strafe_instruction());
		}
		collector.instruction()
	}
}

impl crate::instruction::InstructionConsumer<PhysicsInstruction> for PhysicsState {
	fn process_instruction(&mut self, ins:TimedInstruction<PhysicsInstruction>) {
		match &ins.instruction {
		    PhysicsInstruction::StrafeTick => (),
		    _=>println!("{:?}",ins),
		}
		//selectively update body
		match &ins.instruction {
		    PhysicsInstruction::SetWalkTargetVelocity(_) => (),//TODO: queue instructions and do self.time=ins.time,
		    PhysicsInstruction::RefreshWalkTarget
		    |PhysicsInstruction::ReachWalkTargetVelocity
		    |PhysicsInstruction::CollisionStart(_)
		    |PhysicsInstruction::CollisionEnd(_)
		    |PhysicsInstruction::StrafeTick
		    |PhysicsInstruction::SetControlDir(_)
		    |PhysicsInstruction::Jump => self.advance_time(ins.time),
		}
		match ins.instruction {
			PhysicsInstruction::CollisionStart(c) => {
				//check ground
				match &c.face {
			        AabbFace::Top => {
			        	//ground
			        	self.grounded=true;
			        },
			        _ => (),
			    }
			    self.contacts.insert(c);
				//flatten v
				let mut v=self.body.velocity;
				self.contact_constrain_velocity(&mut v);
				self.body.velocity=v;
				self.walk.state=WalkEnum::Invalid;
			},
			PhysicsInstruction::CollisionEnd(c) => {
			    self.contacts.remove(&c);//remove contact before calling contact_constrain_acceleration
				let mut a=self.gravity;
				self.contact_constrain_acceleration(&mut a);
				self.body.acceleration=a;
				self.walk.state=WalkEnum::Invalid;
				//check ground
				match &c.face {
			        AabbFace::Top => {
			        	self.grounded=false;
			        },
			        _ => (),
			    }
			},
			PhysicsInstruction::SetControlDir(control_dir)=>{
				self.temp_control_dir=control_dir;
				self.walk.state=WalkEnum::Invalid;
			},
			PhysicsInstruction::StrafeTick => {
				//let control_dir=self.get_control_dir();//this should respect your mouse interpolation settings
				let d=self.body.velocity.dot(self.temp_control_dir);
				if d<self.mv {
					let mut v=self.body.velocity+(self.mv-d)*self.temp_control_dir;
					self.contact_constrain_velocity(&mut v);
					self.body.velocity=v;
				}
			}
			PhysicsInstruction::Jump => {
				self.grounded=false;//do I need this?
				let mut v=self.body.velocity+glam::Vec3::new(0.0,0.715588/2.0*100.0,0.0);
				self.contact_constrain_velocity(&mut v);
				self.body.velocity=v;
				self.walk.state=WalkEnum::Invalid;
			},
			PhysicsInstruction::ReachWalkTargetVelocity => {
				//precisely set velocity
				let mut a=glam::Vec3::ZERO;
				self.contact_constrain_acceleration(&mut a);
				self.body.acceleration=a;
				let mut v=self.walk.target_velocity;
				self.contact_constrain_velocity(&mut v);
				self.body.velocity=v;
				self.walk.state=WalkEnum::Reached;
			},
			PhysicsInstruction::RefreshWalkTarget => {
				//calculate acceleration yada yada
				if self.grounded{
					let mut target_diff=self.walk.target_velocity-self.body.velocity;
					target_diff.y=0f32;
					if target_diff==glam::Vec3::ZERO{
						let mut a=glam::Vec3::ZERO;
						self.contact_constrain_acceleration(&mut a);
						self.body.acceleration=a;
						self.walk.state=WalkEnum::Reached;
					}else{
						let accel=self.walk_accel.min(self.gravity.length()*self.friction);
						let time_delta=target_diff.length()/accel;
						let mut a=target_diff/time_delta;
						self.contact_constrain_acceleration(&mut a);
						self.body.acceleration=a;
						self.walk.target_time=self.body.time+((time_delta as f64)*1_000_000_000f64) as TIME;
						self.walk.state=WalkEnum::Transient;
					}
				}
			},
			PhysicsInstruction::SetWalkTargetVelocity(v) => {
				self.walk.target_velocity=v;
				self.walk.state=WalkEnum::Invalid;
			},
		}
	}
}
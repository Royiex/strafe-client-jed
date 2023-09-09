use crate::instruction::{InstructionEmitter, InstructionConsumer, TimedInstruction};

pub enum PhysicsInstruction {
	CollisionStart(RelativeCollision),
	CollisionEnd(RelativeCollision),
	StrafeTick,
	Jump,
	SetWalkTargetVelocity(glam::Vec3),
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

pub enum MoveRestriction {
    Air,
    Water,
    Ground,
    Ladder,//multiple ladders how
}

pub struct PhysicsState {
	pub body: Body,
	pub contacts: Vec<RelativeCollision>,
	//temp
	pub models_cringe_clone: Vec<Model>,
	pub temp_control_dir: glam::Vec3,
	pub time: TIME,
	pub strafe_tick_num: TIME,
	pub strafe_tick_den: TIME,
	pub tick: u32,
	pub mv: f32,
	pub walkspeed: f32,
	pub friction: f32,
	pub walk_target_velocity: glam::Vec3,
	pub gravity: glam::Vec3,
	pub grounded: bool,
	pub jump_trying: bool,
}

#[derive(Clone,Copy)]
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
	pub fn face_vertices(face:AabbFace) -> [glam::Vec3;4] {
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

type Face = AabbFace;
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
	pub fn face_vertices(&self,face:Face) -> [glam::Vec3;4] {
		Aabb::face_vertices(face)
	}
	pub fn face_mesh(&self,face:Face) -> TreyMesh {
		let mut aabb=Aabb::new();
		for &vertex in self.face_vertices(face).iter() {
			aabb.grow(vertex);
		}
		return aabb;
	}
	pub fn face_normal(&self,face:Face) -> glam::Vec3 {
		let mut n=glam::Vec3Swizzles::xyzz(Aabb::normal(face));
		n.w=0.0;//what a man will do to avoid writing out the components
		glam::Vec4Swizzles::xyz(self.transform*n)//this is wrong for scale
	}
}

pub struct RelativeCollision {
	face: Face,//just an id
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
	pub fn with_position(position:glam::Vec3) -> Self {
		Self{
			position: position,
			velocity: glam::Vec3::ZERO,
			acceleration: glam::Vec3::ZERO,
			time: 0,
		}
	}
	pub fn extrapolated_position(&self,time: TIME)->glam::Vec3{
		let dt=(time-self.time) as f64/1_000_000_000f64;
		self.position+self.velocity*(dt as f32)+self.acceleration*((0.5*dt*dt) as f32)
	}
	pub fn advance_time(&mut self, time: TIME){
		self.position=self.extrapolated_position(time);
		self.time=time;
	}
}

impl PhysicsState {
	//tickless gaming
	pub fn run(&mut self, time: TIME){
		//prepare is ommitted - everything is done via instructions.
		while let Some(instruction) = self.next_instruction(time) {//collect
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
		//check if you are accelerating towards a walk target velocity and create an instruction
		return None;
	}
	fn predict_collision_end(&self,model:&Model) -> Option<TimedInstruction<PhysicsInstruction>> {
		//must treat cancollide false objects differently: you may not exit through the same face you entered.
		None
	}
	fn predict_collision_start(&self,model:&Model) -> Option<TimedInstruction<PhysicsInstruction>> {
		None
	}
}

impl crate::instruction::InstructionEmitter<PhysicsInstruction> for PhysicsState {
	//this little next instruction function can cache its return value and invalidate the cached value by watching the State.
	fn next_instruction(&self,time_limit:TIME) -> Option<TimedInstruction<PhysicsInstruction>> {
		//JUST POLLING!!! NO MUTATION
		let mut collector = crate::instruction::InstructionCollector::new(time_limit);
		//autohop (already pressing spacebar; the signal to begin trying to jump is different)
		if self.grounded&&self.jump_trying {
			//scroll will be implemented with InputInstruction::Jump(true) but it blocks setting self.jump_trying=true
			collector.collect(Some(TimedInstruction{
				time:self.time,
				instruction:PhysicsInstruction::Jump
			}));
		}
		//check for collision stop instructions with curent contacts
		for collision_data in self.contacts.iter() {
			collector.collect(self.predict_collision_end(self.models_cringe_clone.get(collision_data.model as usize).unwrap()));
		}
		//check for collision start instructions (against every part in the game with no optimization!!)
		for model in &self.models_cringe_clone {
			collector.collect(self.predict_collision_start(model));
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
		//mutate position and velocity and time
		self.advance_time(ins.time);//should this be in run?
		match ins.instruction {
		    PhysicsInstruction::CollisionStart(_) => todo!(),
		    PhysicsInstruction::CollisionEnd(_) => todo!(),
		    PhysicsInstruction::StrafeTick => {
		    	//let control_dir=self.get_control_dir();//this should respect your mouse interpolation settings
    			let d=self.body.velocity.dot(self.temp_control_dir);
    			if d<self.mv {
    				self.body.velocity+=(self.mv-d)*self.temp_control_dir;
    			}
    		}
		    PhysicsInstruction::Jump => {
				self.grounded=false;//do I need this?
				self.body.velocity+=glam::Vec3::new(0.0,0.715588/2.0*100.0,0.0);
			}
		    PhysicsInstruction::ReachWalkTargetVelocity => {
		    	//precisely set velocity
				self.body.velocity=self.walk_target_velocity;
			}
    		PhysicsInstruction::SetWalkTargetVelocity(v) => {
    			self.walk_target_velocity=v;
    			//calculate acceleration yada yada
    		},
		}
	}
}
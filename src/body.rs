use crate::event::EventStruct;

pub struct Body {
	pub position: glam::Vec3,//I64 where 2^32 = 1 u
	pub velocity: glam::Vec3,//I64 where 2^32 = 1 u/s
	pub time: TIME,//nanoseconds x xxxxD!
}

pub struct PhysicsState {
	pub body: Body,
	pub contacts: Vec<RelativeCollision>,
	pub models_cringe_clone: Vec<Model>,
	pub time: TIME,
	pub strafe_tick_num: TIME,
	pub strafe_tick_den: TIME,
	pub tick: u32,
	pub mv: f32,
	pub walkspeed: f32,
	pub friction: f32,
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

const CONTROL_JUMP:u32 = 0b01000000;//temp DATA NORMALIZATION!@#$
impl PhysicsState {
	//delete this, we are tickless gamers
	pub fn run(&mut self, time: TIME, control_dir: glam::Vec3, controls: u32){
		let target_tick = (time*self.strafe_tick_num/self.strafe_tick_den) as u32;
		//the game code can run for 1 month before running out of ticks
		while self.tick<target_tick {
			self.tick += 1;
			let dt=0.01;
			let d=self.body.velocity.dot(control_dir);
			if d<self.mv {
				self.body.velocity+=(self.mv-d)*control_dir;
			}
			self.body.velocity+=self.gravity*dt;
			self.body.position+=self.body.velocity*dt;
			if self.body.position.y<0.0{
				self.body.position.y=0.0;
				self.body.velocity.y=0.0;
				self.grounded=true;
			}
			if self.grounded&&(controls&CONTROL_JUMP)!=0 {
				self.grounded=false;
				self.body.velocity+=glam::Vec3::new(0.0,0.715588/2.0*100.0,0.0);
			}
			if self.grounded {
				let applied_friction=self.friction*dt;
				let targetv=control_dir*self.walkspeed;
				let diffv=targetv-self.body.velocity;
				if applied_friction*applied_friction<diffv.length_squared() {
					self.body.velocity+=applied_friction*diffv.normalize();
				} else {
					//EventEnum::WalkTargetReached
					self.body.velocity=targetv;
				}
			}
		}

		self.body.time=target_tick as TIME*self.strafe_tick_den/self.strafe_tick_num;
	}

	//delete this
	pub fn extrapolate_position(&self, time: TIME) -> glam::Vec3 {
		let dt=(time-self.body.time) as f64/1_000_000_000f64;
		self.body.position+self.body.velocity*(dt as f32)+self.gravity*((0.5*dt*dt) as f32)
	}

	fn next_strafe_event(&self) -> Option<EventStruct> {
		return Some(EventStruct{
			time:(self.time*self.strafe_tick_num/self.strafe_tick_den+1)*self.strafe_tick_den/self.strafe_tick_num,
			event:crate::event::EventEnum::StrafeTick
		});
	}

	fn next_walk_event(&self) -> Option<EventStruct> {
		//check if you are accelerating towards a walk target velocity and create an event
		return None;
	}
	fn predict_collision_end(&self,model:&Model) -> Option<EventStruct> {
		None
	}
	fn predict_collision_start(&self,model:&Model) -> Option<EventStruct> {
		None
	}
}

impl crate::event::EventTrait for PhysicsState {
	//this little next event function can cache its return value and invalidate the cached value by watching the State.
	fn next_event(&self) -> Option<EventStruct> {
		//JUST POLLING!!! NO MUTATION
		let mut best = crate::event::EventCollector::new();
		//autohop (already pressing spacebar; the signal to begin trying to jump is different)
		if self.grounded&&self.jump_trying {
			//scroll will be implemented with InputEvent::Jump(true) but it blocks setting self.jump_trying=true
			best.collect(Some(EventStruct{
				time:self.time,
				event:crate::event::EventEnum::Jump
			}));
		}
		//check for collision stop events with curent contacts
		for collision_data in self.contacts.iter() {
			best.collect(self.predict_collision_end(self.models_cringe_clone.get(collision_data.model as usize).unwrap()));
		}
		//check for collision start events (against every part in the game with no optimization!!)
		for model in &self.models_cringe_clone {
			best.collect(self.predict_collision_start(model));
		}
		if self.grounded {
			//walk maintenance
			best.collect(self.next_walk_event());
		}else{
			//check to see when the next strafe tick is
			best.collect(self.next_strafe_event());
		}
		best.event()
	}
}

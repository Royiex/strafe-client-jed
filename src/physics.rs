use crate::{instruction::{InstructionEmitter, InstructionConsumer, TimedInstruction}, zeroes::zeroes2};

#[derive(Debug)]
pub enum PhysicsInstruction {
	CollisionStart(RelativeCollision),
	CollisionEnd(RelativeCollision),
	StrafeTick,
	ReachWalkTargetVelocity,
	// Water,
	// Spawn(
	// 	Option<SpawnId>,
	// 	bool,//true = Trigger; false = teleport
	// 	bool,//true = Force
	// )
	//InputInstructions conditionally activate RefreshWalkTarget (by doing what SetWalkTargetVelocity used to do and then flagging it)
	Input(InputInstruction),
}
#[derive(Debug)]
pub enum InputInstruction {
	MoveMouse(glam::IVec2),
	MoveForward(bool),
	MoveLeft(bool),
	MoveBack(bool),
	MoveRight(bool),
	MoveUp(bool),
	MoveDown(bool),
	Jump(bool),
	Zoom(bool),
	Reset,
	Idle,
		//Idle: there were no input events, but the simulation is safe to advance to this timestep
		//for interpolation / networking / playback reasons, most playback heads will always want
		//to be 1 instruction ahead to generate the next state for interpolation.
}
#[derive(Clone)]
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

/*
enum InputInstruction {
}
struct InputState {
}
impl InputState {
	pub fn get_control(&self,control:u32) -> bool {
		self.controls&control!=0
	}
}
impl crate::instruction::InstructionEmitter<InputInstruction> for InputState{
	fn next_instruction(&self, time_limit:crate::body::TIME) -> Option<TimedInstruction<InputInstruction>> {
		//this is polled by PhysicsState for actions like Jump
		//no, it has to be the other way around. physics is run up until the jump instruction, and then the jump instruction is pushed.
		self.queue.get(0)
	}
}
impl crate::instruction::InstructionConsumer<InputInstruction> for InputState{
	fn process_instruction(&mut self,ins:TimedInstruction<InputInstruction>){
		//add to queue
		self.queue.push(ins);
	}
}
*/

//hey dumbass just use a delta
#[derive(Clone)]
pub struct MouseState {
	pub pos: glam::IVec2,
	pub time: TIME,
}
impl Default for MouseState{
	fn default() -> Self {
		Self {
			time:0,
			pos:glam::IVec2::ZERO,
		}
	}
}
impl MouseState {
	pub fn move_mouse(&mut self,pos:glam::IVec2,time:TIME){
		self.time=time;
		self.pos=pos;
	}
	pub fn lerp(&self,target:&MouseState,time:TIME)->glam::IVec2 {
		let m0=self.pos.as_i64vec2();
		let m1=target.pos.as_i64vec2();
		//these are deltas
		let t1t=(target.time-time) as i64;
		let tt0=(time-self.time) as i64;
		let dt=(target.time-self.time) as i64;
		((m0*t1t+m1*tt0)/dt).as_ivec2()
	}
}

pub enum WalkEnum{
	Reached,
	Transient,
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
			state:WalkEnum::Reached,
		}
	}
}

#[derive(Clone)]
pub struct PhysicsCamera {
	offset: glam::Vec3,
	angles: glam::DVec2,//YAW AND THEN PITCH
	//punch: glam::Vec3,
	//punch_velocity: glam::Vec3,
	sensitivity: glam::DVec2,
	mouse:MouseState,
}

#[inline]
fn mat3_from_rotation_y_f64(angle: f64) -> glam::Mat3 {
	let (sina, cosa) = angle.sin_cos();
	glam::Mat3::from_cols(
		glam::Vec3::new(cosa as f32, 0.0, -sina as f32),
		glam::Vec3::Y,
		glam::Vec3::new(sina as f32, 0.0, cosa as f32),
	)
}
impl PhysicsCamera {
	pub fn from_offset(offset:glam::Vec3) -> Self {
		Self{
			offset,
			angles: glam::DVec2::ZERO,
			sensitivity: glam::dvec2(1.0/16384.0,1.0/16384.0),
			mouse:MouseState{pos:glam::IVec2::ZERO,time:-1},//escape initialization hell divide by zero
		}
	}
	pub fn simulate_move_angles(&self, mouse_pos: glam::IVec2) -> glam::DVec2 {
		let mut a=self.angles-self.sensitivity*(mouse_pos-self.mouse.pos).as_dvec2();
		a.y=a.y.clamp(-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2);
		return a
	}
	fn simulate_move_rotation_y(&self, delta_x: i32) -> glam::Mat3 {
		mat3_from_rotation_y_f64(self.angles.x-self.sensitivity.x*(delta_x as f64))
	}
}

pub struct GameMechanicsState{
	pub stage_id:u32,
	//jump_counts:HashMap<u32,u32>,
}
impl std::default::Default for GameMechanicsState{
	fn default() -> Self {
		Self{
			stage_id:0,
		}
	}
}

pub struct WorldState{}

pub struct StyleModifiers{
	pub controls_mask:u32,//controls which are unable to be activated
	pub controls_held:u32,//controls which must be active to be able to strafe
	pub mv:f32,
	pub walkspeed:f32,
	pub friction:f32,
	pub walk_accel:f32,
	pub gravity:glam::Vec3,
	pub strafe_tick_num:TIME,
	pub strafe_tick_den:TIME,
	pub hitbox_halfsize:glam::Vec3,
}
impl std::default::Default for StyleModifiers{
	fn default() -> Self {
		Self{
			controls_mask: !0,//&!(Self::CONTROL_MOVEUP|Self::CONTROL_MOVEDOWN),
			controls_held: 0,
			strafe_tick_num: 100,//100t
			strafe_tick_den: 1_000_000_000,
			gravity: glam::vec3(0.0,-100.0,0.0),
			friction: 1.2,
			walk_accel: 90.0,
			mv: 2.7,
			walkspeed: 18.0,
			hitbox_halfsize: glam::vec3(1.0,2.5,1.0),
		}
	}
}
impl StyleModifiers{
	const CONTROL_MOVEFORWARD:u32 = 0b00000001;
	const CONTROL_MOVEBACK:u32 = 0b00000010;
	const CONTROL_MOVERIGHT:u32 = 0b00000100;
	const CONTROL_MOVELEFT:u32 = 0b00001000;
	const CONTROL_MOVEUP:u32 = 0b00010000;
	const CONTROL_MOVEDOWN:u32 = 0b00100000;
	const CONTROL_JUMP:u32 = 0b01000000;
	const CONTROL_ZOOM:u32 = 0b10000000;

	const FORWARD_DIR:glam::Vec3 = glam::Vec3::NEG_Z;
	const RIGHT_DIR:glam::Vec3 = glam::Vec3::X;
	const UP_DIR:glam::Vec3 = glam::Vec3::Y;

	fn get_control(&self,control:u32,controls:u32)->bool{
		controls&self.controls_mask&control==control
	}

	fn get_control_dir(&self,controls:u32)->glam::Vec3{
		//don't get fancy just do it
		let mut control_dir:glam::Vec3 = glam::Vec3::ZERO;
		//Disallow strafing if held controls are not held
		if controls&self.controls_held!=self.controls_held{
			return control_dir;
		}
		//Apply mask after held check so you can require non-allowed keys to be held for some reason
		let controls=controls&self.controls_mask;
		if controls & Self::CONTROL_MOVEFORWARD == Self::CONTROL_MOVEFORWARD {
			control_dir+=Self::FORWARD_DIR;
		}
		if controls & Self::CONTROL_MOVEBACK == Self::CONTROL_MOVEBACK {
			control_dir+=-Self::FORWARD_DIR;
		}
		if controls & Self::CONTROL_MOVELEFT == Self::CONTROL_MOVELEFT {
			control_dir+=-Self::RIGHT_DIR;
		}
		if controls & Self::CONTROL_MOVERIGHT == Self::CONTROL_MOVERIGHT {
			control_dir+=Self::RIGHT_DIR;
		}
		if controls & Self::CONTROL_MOVEUP == Self::CONTROL_MOVEUP {
			control_dir+=Self::UP_DIR;
		}
		if controls & Self::CONTROL_MOVEDOWN == Self::CONTROL_MOVEDOWN {
			control_dir+=-Self::UP_DIR;
		}
		return control_dir
	}
}

pub struct PhysicsState{
	pub time:TIME,
	pub body:Body,
	pub world:WorldState,//currently there is only one state the world can be in
	pub game:GameMechanicsState,
	pub style:StyleModifiers,
	pub contacts:std::collections::HashMap::<u32,RelativeCollision>,
	pub intersects:std::collections::HashMap::<u32,RelativeCollision>,
	//pub intersections: Vec<ModelId>,
	//camera must exist in state because wormholes modify the camera, also camera punch
	pub camera:PhysicsCamera,
	pub next_mouse:MouseState,//Where is the mouse headed next
	pub controls:u32,
	pub walk:WalkState,
	pub grounded:bool,
	//all models
	pub models:Vec<ModelPhysics>,
	
	pub modes:Vec<crate::model::ModeDescription>,
	pub mode_from_mode_id:std::collections::HashMap::<u32,usize>,
	//the spawn point is where you spawn when you load into the map.
	//This is not the same as Reset which teleports you to Spawn0
	pub spawn_point:glam::Vec3,
}
#[derive(Clone)]
pub struct PhysicsOutputState{
	camera:PhysicsCamera,
	body:Body,
}
impl PhysicsOutputState{
	pub fn adjust_mouse(&self,mouse:&MouseState)->(glam::Vec3,glam::Vec2){
		(self.body.extrapolated_position(mouse.time),self.camera.simulate_move_angles(mouse.pos).as_vec2())
	}
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
#[derive(Clone)]
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

enum PhysicsCollisionAttributes{
	Contact{//track whether you are contacting the object
		contacting:crate::model::ContactingAttributes,
		general:crate::model::GameMechanicAttributes,
	},
	Intersect{//track whether you are intersecting the object
		intersecting:crate::model::IntersectingAttributes,
		general:crate::model::GameMechanicAttributes,
	},
}

pub struct ModelPhysics {
	//A model is a thing that has a hitbox. can be represented by a list of TreyMesh-es
	//in this iteration, all it needs is extents.
	mesh: TreyMesh,
	transform:glam::Affine3A,
	attributes:PhysicsCollisionAttributes,
}

impl ModelPhysics {
	fn from_model_transform_attributes(model:&crate::model::IndexedModel,transform:&glam::Affine3A,attributes:PhysicsCollisionAttributes)->Self{
		let mut aabb=Aabb::new();
		for indexed_vertex in &model.unique_vertices {
			aabb.grow(transform.transform_point3(glam::Vec3::from_array(model.unique_pos[indexed_vertex.pos as usize])));
		}
		Self{
			mesh:aabb,
			attributes,
			transform:transform.clone(),
		}
	}
	pub fn from_model(model:&crate::model::IndexedModel,instance:&crate::model::ModelInstance) -> Option<Self> {
		match &instance.attributes{
			crate::model::CollisionAttributes::Contact{contacting,general}=>Some(ModelPhysics::from_model_transform_attributes(model,&instance.transform,PhysicsCollisionAttributes::Contact{contacting:contacting.clone(),general:general.clone()})),
			crate::model::CollisionAttributes::Intersect{intersecting,general}=>Some(ModelPhysics::from_model_transform_attributes(model,&instance.transform,PhysicsCollisionAttributes::Intersect{intersecting:intersecting.clone(),general:general.clone()})),
			crate::model::CollisionAttributes::Decoration=>None,
		}
	}
	pub fn unit_vertices(&self) -> [glam::Vec3;8] {
		Aabb::unit_vertices()
	}
	pub fn mesh(&self) -> &TreyMesh {
		return &self.mesh;
	}
	pub fn unit_face_vertices(&self,face:TreyMeshFace) -> [glam::Vec3;4] {
		Aabb::unit_face_vertices(face)
	}
	pub fn face_mesh(&self,face:TreyMeshFace) -> TreyMesh {
		let mut aabb=self.mesh.clone();
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
		Aabb::normal(face)//this is wrong for scale
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
	pub fn model<'a>(&self,models:&'a Vec<ModelPhysics>)->Option<&'a ModelPhysics>{
		models.get(self.model as usize)
	}
	pub fn mesh(&self,models:&Vec<ModelPhysics>) -> TreyMesh {
		return self.model(models).unwrap().face_mesh(self.face).clone()
	}
	pub fn normal(&self,models:&Vec<ModelPhysics>) -> glam::Vec3 {
		return self.model(models).unwrap().face_normal(self.face)
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

impl Default for PhysicsState{
	fn default() -> Self {
 		Self{
			spawn_point:glam::vec3(0.0,50.0,0.0),
			body: Body::with_pva(glam::vec3(0.0,50.0,0.0),glam::vec3(0.0,0.0,0.0),glam::vec3(0.0,-100.0,0.0)),
			time: 0,
			style:StyleModifiers::default(),
			grounded: false,
			contacts: std::collections::HashMap::new(),
			intersects: std::collections::HashMap::new(),
			models: Vec::new(),
			walk: WalkState::new(),
			camera: PhysicsCamera::from_offset(glam::vec3(0.0,4.5-2.5,0.0)),
			next_mouse: MouseState::default(),
			controls: 0,
			world:WorldState{},
			game:GameMechanicsState::default(),
			modes:Vec::new(),
			mode_from_mode_id:std::collections::HashMap::new(),
		}
	}
}

impl PhysicsState {
	pub fn clear(&mut self){
		self.models.clear();
		self.modes.clear();
		self.contacts.clear();
		self.intersects.clear();
	}

	pub fn into_worker(mut self)->crate::worker::Worker<TimedInstruction<InputInstruction>,PhysicsOutputState>{
		let mut last_time=0;
			//last_time: this indicates the last time the mouse position was known.
			//Only used to generate a MouseState right before mouse movement
			//to finalize a long period of no movement and avoid interpolating from a long out-of-date MouseState.
		let mut mouse_blocking=true;//waiting for next_mouse to be written
		let mut timeline=std::collections::VecDeque::new();
		crate::worker::Worker::new(self.output(),move |ins:TimedInstruction<InputInstruction>|{
			let run_queue=match &ins.instruction{
				InputInstruction::MoveMouse(_)=>{
					if !mouse_blocking{
						//mouse has not been moving for a while.
						//make sure not to interpolate between two distant MouseStates.
						//generate a mouse instruction with no movement timestamped at last InputInstruction
						//Idle instructions are CRITICAL to keeping this value up to date
						//interpolate normally (now that prev mouse pos is up to date)
						timeline.push_back(TimedInstruction{
							time:last_time,
							instruction:InputInstruction::MoveMouse(self.next_mouse.pos),
						});
					}
					mouse_blocking=true;//block physics until the next mouse event or mouse event timeout.
					true//empty queue
				},
				_=>{
					if mouse_blocking{
						//check if last mouse move is within 50ms
						if ins.time-self.next_mouse.time<50_000_000{
							last_time=ins.time;
							false//do not empty queue
						}else{
							mouse_blocking=false;
							timeline.push_back(TimedInstruction{
								time:ins.time,
								instruction:InputInstruction::MoveMouse(self.next_mouse.pos),
							});
							true
						}
					}else{
						last_time=ins.time;
						true
					}
				},
			};
			timeline.push_back(ins);
			if run_queue{
				//empty queue
				while let Some(instruction)=timeline.pop_front(){
					self.run(instruction.time);
					self.process_instruction(TimedInstruction{
						time:instruction.time,
						instruction:PhysicsInstruction::Input(instruction.instruction),
					});
				}
			}
			self.output()
		})
	}

	pub fn output(&self)->PhysicsOutputState{
		PhysicsOutputState{
			body:self.body.clone(),
			camera:self.camera.clone(),
		}
	}

	pub fn generate_models(&mut self,indexed_models:&crate::model::IndexedModelInstances){
		let mut starts=Vec::new();
		let mut spawns=Vec::new();
		let mut ordered_checkpoints=Vec::new();
		let mut unordered_checkpoints=Vec::new();
		for model in &indexed_models.models{
			//make aabb and run vertices to get realistic bounds
			for model_instance in &model.instances{
				if let Some(model_physics)=ModelPhysics::from_model(model,model_instance){
					let model_id=self.models.len() as u32;
					self.models.push(model_physics);
					for attr in &model_instance.temp_indexing{
						match attr{
							crate::model::TempIndexedAttributes::Start{mode_id}=>starts.push((*mode_id,model_id)),
							crate::model::TempIndexedAttributes::Spawn{mode_id,stage_id}=>spawns.push((*mode_id,model_id,*stage_id)),
							crate::model::TempIndexedAttributes::OrderedCheckpoint{mode_id,checkpoint_id}=>ordered_checkpoints.push((*mode_id,model_id,*checkpoint_id)),
							crate::model::TempIndexedAttributes::UnorderedCheckpoint{mode_id}=>unordered_checkpoints.push((*mode_id,model_id)),
						}
					}
				}
			}
		}
		//I don't wanna write structs for temporary structures
		//this code builds ModeDescriptions from the unsorted lists at the top of the function
		starts.sort_by_key(|tup|tup.0);
		let mut eshmep=std::collections::HashMap::new();
		let mut modedatas:Vec<(u32,Vec<(u32,u32)>,Vec<(u32,u32)>,Vec<u32>)>=starts.into_iter().enumerate().map(|(i,tup)|{
			eshmep.insert(tup.0,i);
			(tup.1,Vec::new(),Vec::new(),Vec::new())
		}).collect();
		for tup in spawns{
			if let Some(mode_id)=eshmep.get(&tup.0){
				if let Some(modedata)=modedatas.get_mut(*mode_id){
					modedata.1.push((tup.2,tup.1));
				}
			}
		}
		for tup in ordered_checkpoints{
			if let Some(mode_id)=eshmep.get(&tup.0){
				if let Some(modedata)=modedatas.get_mut(*mode_id){
					modedata.2.push((tup.2,tup.1));
				}
			}
		}
		for tup in unordered_checkpoints{
			if let Some(mode_id)=eshmep.get(&tup.0){
				if let Some(modedata)=modedatas.get_mut(*mode_id){
					modedata.3.push(tup.1);
				}
			}
		}
		let num_modes=self.modes.len();
		for (mode_id,mode) in eshmep{
			self.mode_from_mode_id.insert(mode_id,num_modes+mode);
		}
		self.modes.append(&mut modedatas.into_iter().map(|mut tup|{
			tup.1.sort_by_key(|tup|tup.0);
			tup.2.sort_by_key(|tup|tup.0);
			let mut eshmep1=std::collections::HashMap::new();
			let mut eshmep2=std::collections::HashMap::new();
			crate::model::ModeDescription{
				start:tup.0,
				spawns:tup.1.into_iter().enumerate().map(|(i,tup)|{eshmep1.insert(tup.0,i);tup.1}).collect(),
				ordered_checkpoints:tup.2.into_iter().enumerate().map(|(i,tup)|{eshmep2.insert(tup.0,i);tup.1}).collect(),
				unordered_checkpoints:tup.3,
				spawn_from_stage_id:eshmep1,
				ordered_checkpoint_from_checkpoint_id:eshmep2,
			}
		}).collect());
		println!("Physics Objects: {}",self.models.len());
	}

	pub fn get_mode(&self,mode_id:u32)->Option<&crate::model::ModeDescription>{
		if let Some(&mode)=self.mode_from_mode_id.get(&mode_id){
			self.modes.get(mode)
		}else{
			None
		}
	}
	//tickless gaming
	pub fn run(&mut self, time_limit:TIME){
		//prepare is ommitted - everything is done via instructions.
		while let Some(instruction) = self.next_instruction(time_limit) {//collect
			//process
			self.process_instruction(instruction);
			//write hash lol
		}
	}

	pub fn advance_time(&mut self, time: TIME){
		self.body.advance_time(time);
		self.time=time;
	}

	fn set_control(&mut self,control:u32,state:bool){
		self.controls=if state{self.controls|control}else{self.controls&!control};
	}
	fn jump(&mut self){
		self.grounded=false;//do I need this?
		let mut v=self.body.velocity+glam::Vec3::new(0.0,0.715588/2.0*100.0,0.0);
		self.contact_constrain_velocity(&mut v);
		self.body.velocity=v;
	}

	fn contact_constrain_velocity(&self,velocity:&mut glam::Vec3){
		for (_,contact) in &self.contacts {
			let n=contact.normal(&self.models);
			let d=velocity.dot(n);
			if d<0f32{
				(*velocity)-=d/n.length_squared()*n;
			}
		}
	}
	fn contact_constrain_acceleration(&self,acceleration:&mut glam::Vec3){
		for (_,contact) in &self.contacts {
			let n=contact.normal(&self.models);
			let d=acceleration.dot(n);
			if d<0f32{
				(*acceleration)-=d/n.length_squared()*n;
			}
		}
	}
	fn next_strafe_instruction(&self) -> Option<TimedInstruction<PhysicsInstruction>> {
		return Some(TimedInstruction{
			time:(self.time*self.style.strafe_tick_num/self.style.strafe_tick_den+1)*self.style.strafe_tick_den/self.style.strafe_tick_num,
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

	fn refresh_walk_target(&mut self){
		//calculate acceleration yada yada
		if self.grounded{
			let mut v=self.walk.target_velocity;
			self.contact_constrain_velocity(&mut v);
			let mut target_diff=v-self.body.velocity;
			target_diff.y=0f32;
			if target_diff==glam::Vec3::ZERO{
				let mut a=glam::Vec3::ZERO;
				self.contact_constrain_acceleration(&mut a);
				self.body.acceleration=a;
				self.walk.state=WalkEnum::Reached;
			}else{
				let accel=self.style.walk_accel.min(self.style.gravity.length()*self.style.friction);
				let time_delta=target_diff.length()/accel;
				let mut a=target_diff/time_delta;
				self.contact_constrain_acceleration(&mut a);
				self.body.acceleration=a;
				self.walk.target_time=self.body.time+((time_delta as f64)*1_000_000_000f64) as TIME;
				self.walk.state=WalkEnum::Transient;
			}
		}else{
			self.walk.state=WalkEnum::Reached;//there is no walk target while not grounded
		}
	}
	fn next_walk_instruction(&self) -> Option<TimedInstruction<PhysicsInstruction>> {
		//check if you have a valid walk state and create an instruction
		if self.grounded{
			match self.walk.state{
				WalkEnum::Transient=>Some(TimedInstruction{
					time:self.walk.target_time,
					instruction:PhysicsInstruction::ReachWalkTargetVelocity
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
			aabb.grow(self.body.position+self.style.hitbox_halfsize*vertex);
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
		let mesh1=self.models.get(collision_data.model as usize).unwrap().mesh();
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
						break;
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
						break;
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
						break;
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
						break;
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
						break;
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
						break;
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
		let mesh1=self.models.get(model_id as usize).unwrap().mesh();
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
					break;
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
					break;
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
					break;
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
					break;
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
					break;
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
					break;
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
		for (_,collision_data) in &self.contacts {
			collector.collect(self.predict_collision_end(self.time,time_limit,collision_data));
		}
		// for collision_data in &self.intersects{
		// 	collector.collect(self.predict_collision_end2(self.time,time_limit,collision_data));
		// }
		//check for collision start instructions (against every part in the game with no optimization!!)
		for i in 0..self.models.len() {
			let i=i as u32;
			if self.contacts.contains_key(&i)||self.intersects.contains_key(&i){
				continue;
			}
			collector.collect(self.predict_collision_start(self.time,time_limit,i));
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
			PhysicsInstruction::Input(InputInstruction::Idle)|
			PhysicsInstruction::StrafeTick => (),
			PhysicsInstruction::Input(InputInstruction::MoveMouse(_)) => (),
			_=>println!("{}|{:?}",ins.time,ins.instruction),
		}
		//selectively update body
		match &ins.instruction {
			PhysicsInstruction::Input(InputInstruction::MoveMouse(_)) => (),//dodge time for mouse movement
			PhysicsInstruction::Input(_)
			|PhysicsInstruction::ReachWalkTargetVelocity
			|PhysicsInstruction::CollisionStart(_)
			|PhysicsInstruction::CollisionEnd(_)
			|PhysicsInstruction::StrafeTick => self.advance_time(ins.time),
		}
		match ins.instruction {
			PhysicsInstruction::CollisionStart(c) => {
				let model=c.model(&self.models).unwrap();
				match &model.attributes{
					PhysicsCollisionAttributes::Contact{contacting,general}=>{
						match &contacting.surf{
							Some(surf)=>println!("I'm surfing!"),
							None=>match &c.face {
								AabbFace::Top => {
									//ground
									self.grounded=true;
								},
								_ => (),
							},
						}
						//check ground
						self.contacts.insert(c.model,c);
						match &general.stage_element{
							Some(stage_element)=>{
								if stage_element.force||self.game.stage_id<stage_element.stage_id{
									self.game.stage_id=stage_element.stage_id;
								}
								match stage_element.behaviour{
									crate::model::StageElementBehaviour::SpawnAt=>(),
									crate::model::StageElementBehaviour::Trigger
									|crate::model::StageElementBehaviour::Teleport=>{
										//TODO make good
										if let Some(mode)=self.get_mode(stage_element.mode_id){
											if let Some(&spawn)=mode.get_spawn_model_id(self.game.stage_id){
												if let Some(model)=self.models.get(spawn as usize){
													self.body.position=model.transform.transform_point3(glam::Vec3::Y)+glam::Vec3::Y*(self.style.hitbox_halfsize.y+0.1);
													//manual clear //for c in self.contacts{process_instruction(CollisionEnd(c))}
													self.contacts.clear();
													self.intersects.clear();
													self.body.acceleration=self.style.gravity;
													self.walk.state=WalkEnum::Reached;
													self.grounded=false;
												}else{println!("bad1");}
											}else{println!("bad2");}
										}else{println!("bad3");}
									},
									crate::model::StageElementBehaviour::Platform=>(),
								}
							},
							None=>(),
						}
						//flatten v
						let mut v=self.body.velocity;
						self.contact_constrain_velocity(&mut v);
						match &general.booster{
							Some(booster)=>{
								v+=booster.velocity;
								self.contact_constrain_velocity(&mut v);
							},
							None=>(),
						}
						self.body.velocity=v;
						if self.grounded&&self.style.get_control(StyleModifiers::CONTROL_JUMP,self.controls){
							self.jump();
						}
						self.refresh_walk_target();
					},
					PhysicsCollisionAttributes::Intersect{intersecting,general}=>{
						//I think that setting the velocity to 0 was preventing surface contacts from entering an infinite loop
						self.intersects.insert(c.model,c);
					},
				}
			},
			PhysicsInstruction::CollisionEnd(c) => {
				let model=c.model(&self.models).unwrap();
				match &model.attributes{
					PhysicsCollisionAttributes::Contact{contacting,general}=>{
						self.contacts.remove(&c.model);//remove contact before calling contact_constrain_acceleration
						let mut a=self.style.gravity;
						self.contact_constrain_acceleration(&mut a);
						self.body.acceleration=a;
						//check ground
						match &c.face {
							AabbFace::Top => {
								self.grounded=false;
							},
							_ => (),
						}
						self.refresh_walk_target();
					},
					PhysicsCollisionAttributes::Intersect{intersecting,general}=>{
						self.intersects.remove(&c.model);
					},
				}
			},
			PhysicsInstruction::StrafeTick => {
				let camera_mat=self.camera.simulate_move_rotation_y(self.camera.mouse.lerp(&self.next_mouse,self.time).x);
				let control_dir=camera_mat*self.style.get_control_dir(self.controls);
				let d=self.body.velocity.dot(control_dir);
				if d<self.style.mv {
					let mut v=self.body.velocity+(self.style.mv-d)*control_dir;
					self.contact_constrain_velocity(&mut v);
					self.body.velocity=v;
				}
			}
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
			PhysicsInstruction::Input(input_instruction) => {
				let mut refresh_walk_target=true;
				let mut refresh_walk_target_velocity=true;
				match input_instruction{
					InputInstruction::MoveMouse(m) => {
						self.camera.angles=self.camera.simulate_move_angles(self.next_mouse.pos);
						self.camera.mouse.move_mouse(self.next_mouse.pos,self.next_mouse.time);
						self.next_mouse.move_mouse(m,self.time);
					},
					InputInstruction::MoveForward(s) => self.set_control(StyleModifiers::CONTROL_MOVEFORWARD,s),
					InputInstruction::MoveLeft(s) => self.set_control(StyleModifiers::CONTROL_MOVELEFT,s),
					InputInstruction::MoveBack(s) => self.set_control(StyleModifiers::CONTROL_MOVEBACK,s),
					InputInstruction::MoveRight(s) => self.set_control(StyleModifiers::CONTROL_MOVERIGHT,s),
					InputInstruction::MoveUp(s) => self.set_control(StyleModifiers::CONTROL_MOVEUP,s),
					InputInstruction::MoveDown(s) => self.set_control(StyleModifiers::CONTROL_MOVEDOWN,s),
					InputInstruction::Jump(s) => {
						self.set_control(StyleModifiers::CONTROL_JUMP,s);
						if self.grounded{
							self.jump();
						}
						refresh_walk_target_velocity=false;
					},
					InputInstruction::Zoom(s) => {
						self.set_control(StyleModifiers::CONTROL_ZOOM,s);
						refresh_walk_target=false;
					},
					InputInstruction::Reset => {
						//temp
						self.body.position=self.spawn_point;
						self.body.velocity=glam::Vec3::ZERO;
						//manual clear //for c in self.contacts{process_instruction(CollisionEnd(c))}
						self.contacts.clear();
						self.body.acceleration=self.style.gravity;
						self.walk.state=WalkEnum::Reached;
						self.grounded=false;
						refresh_walk_target=false;
					},
					InputInstruction::Idle => {refresh_walk_target=false;},//literally idle!
				}
				if refresh_walk_target{
					//calculate walk target velocity
					if refresh_walk_target_velocity{
						let camera_mat=self.camera.simulate_move_rotation_y(self.camera.mouse.lerp(&self.next_mouse,self.time).x);
						let control_dir=camera_mat*self.style.get_control_dir(self.controls);
						self.walk.target_velocity=self.style.walkspeed*control_dir;
					}
					self.refresh_walk_target();
				}
			},
		}
	}
}

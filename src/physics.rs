use crate::{instruction::{InstructionEmitter, InstructionConsumer, TimedInstruction}, zeroes::zeroes2};

use crate::integer::{Time,Planar64,Planar64Vec3,Planar64Mat3,Angle32,Ratio64,Ratio64Vec2};

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
	Input(PhysicsInputInstruction),
}
#[derive(Debug)]
pub enum PhysicsInputInstruction {
	ReplaceMouse(MouseState,MouseState),
	SetNextMouse(MouseState),
	SetMoveRight(bool),
	SetMoveUp(bool),
	SetMoveBack(bool),
	SetMoveLeft(bool),
	SetMoveDown(bool),
	SetMoveForward(bool),
	SetJump(bool),
	SetZoom(bool),
	Reset,
	Idle,
}
#[derive(Debug)]
pub enum InputInstruction {
	MoveMouse(glam::IVec2),
	MoveRight(bool),
	MoveUp(bool),
	MoveBack(bool),
	MoveLeft(bool),
	MoveDown(bool),
	MoveForward(bool),
	Jump(bool),
	Zoom(bool),
	Reset,
	Idle,
		//Idle: there were no input events, but the simulation is safe to advance to this timestep
		//for interpolation / networking / playback reasons, most playback heads will always want
		//to be 1 instruction ahead to generate the next state for interpolation.
}
#[derive(Clone,Hash)]
pub struct Body {
	position: Planar64Vec3,//I64 where 2^32 = 1 u
	velocity: Planar64Vec3,//I64 where 2^32 = 1 u/s
	acceleration: Planar64Vec3,//I64 where 2^32 = 1 u/s/s
	time:Time,//nanoseconds x xxxxD!
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
	fn next_instruction(&self, time_limit:crate::body::Time) -> Option<TimedInstruction<InputInstruction>> {
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
#[derive(Clone,Debug)]
pub struct MouseState {
	pub pos: glam::IVec2,
	pub time:Time,
}
impl Default for MouseState{
	fn default() -> Self {
		Self {
			time:Time::ZERO,
			pos:glam::IVec2::ZERO,
		}
	}
}
impl MouseState {
	pub fn lerp(&self,target:&MouseState,time:Time)->glam::IVec2 {
		let m0=self.pos.as_i64vec2();
		let m1=target.pos.as_i64vec2();
		//these are deltas
		let t1t=(target.time-time).nanos();
		let tt0=(time-self.time).nanos();
		let dt=(target.time-self.time).nanos();
		((m0*t1t+m1*tt0)/dt).as_ivec2()
	}
}

pub enum WalkEnum{
	Reached,
	Transient,
}
pub struct WalkState {
	pub target_velocity: Planar64Vec3,
	pub target_time: Time,
	pub state: WalkEnum,
}
impl WalkState {
	pub fn new() -> Self {
		Self{
			target_velocity:Planar64Vec3::ZERO,
			target_time:Time::ZERO,
			state:WalkEnum::Reached,
		}
	}
}



#[derive(Clone)]
pub struct PhysicsCamera {
	offset: Planar64Vec3,
	//punch: Planar64Vec3,
	//punch_velocity: Planar64Vec3,
	sensitivity:Ratio64Vec2,//dots to Angle32 ratios
	mouse:MouseState,//last seen absolute mouse pos
	clamped_mouse_pos:glam::IVec2,//angles are calculated from this cumulative value
	angle_pitch_lower_limit:Angle32,
	angle_pitch_upper_limit:Angle32,
	//angle limits could be an enum + struct that defines whether it's limited and selects clamp or wrap depending
	// enum AngleLimit{
	// 	Unlimited,
	// 	Limited{lower:Angle32,upper:Angle32},
	// }
	//pitch_limit:AngleLimit,
	//yaw_limit:AngleLimit,
}

impl PhysicsCamera {
	pub fn from_offset(offset:Planar64Vec3) -> Self {
		Self{
			offset,
			sensitivity:Ratio64Vec2::ONE*200_000,
			mouse:MouseState::default(),//t=0 does not cause divide by zero because it's immediately replaced
			clamped_mouse_pos:glam::IVec2::ZERO,
			angle_pitch_lower_limit:-Angle32::FRAC_PI_2,
			angle_pitch_upper_limit:Angle32::FRAC_PI_2,
		}
	}
	pub fn move_mouse(&mut self,mouse_pos:glam::IVec2){
		let mut unclamped_mouse_pos=self.clamped_mouse_pos+mouse_pos-self.mouse.pos;
		unclamped_mouse_pos.y=unclamped_mouse_pos.y.clamp(
			self.sensitivity.y.rhs_div_int(self.angle_pitch_lower_limit.get() as i64) as i32,
			self.sensitivity.y.rhs_div_int(self.angle_pitch_upper_limit.get() as i64) as i32,
		);
		self.clamped_mouse_pos=unclamped_mouse_pos;
	}
	pub fn simulate_move_angles(&self,mouse_pos:glam::IVec2)->glam::Vec2 {
		let a=-self.sensitivity.mul_int((mouse_pos-self.mouse.pos+self.clamped_mouse_pos).as_i64vec2());
		let ax=Angle32::wrap_from_i64(a.x);
		let ay=Angle32::clamp_from_i64(a.y)
		//clamp to actual vertical cam limit
		.clamp(self.angle_pitch_lower_limit,self.angle_pitch_upper_limit);
		return glam::vec2(ax.into(),ay.into());
	}
	fn simulate_move_rotation_y(&self,mouse_pos_x:i32)->Planar64Mat3{
		let ax=-self.sensitivity.x.mul_int((mouse_pos_x-self.mouse.pos.x+self.clamped_mouse_pos.x) as i64);
		Planar64Mat3::from_rotation_y(Angle32::wrap_from_i64(ax))
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
	pub strafe_tick_rate:Ratio64,
	pub jump_time:Time,
	pub mv:Planar64,
	pub walkspeed:Planar64,
	pub friction:Planar64,
	pub walk_accel:Planar64,
	pub gravity:Planar64Vec3,
	pub hitbox_halfsize:Planar64Vec3,
}
impl std::default::Default for StyleModifiers{
	fn default() -> Self {
		Self{
			controls_mask: !0,//&!(Self::CONTROL_MOVEUP|Self::CONTROL_MOVEDOWN),
			controls_held: 0,
			strafe_tick_rate:Ratio64::new(100,Time::ONE_SECOND.nanos() as u64).unwrap(),
			jump_time: Time::from_nanos(715_588_000/2*100),//0.715588/2.0*100.0
			gravity: Planar64Vec3::int(0,-100,0),
			friction: Planar64::int(12)/10,
			walk_accel: Planar64::int(90),
			mv: Planar64::int(27)/10,
			walkspeed: Planar64::int(18),
			hitbox_halfsize: Planar64Vec3::int(2,5,2)/2,
		}
	}
}
impl StyleModifiers{
	const CONTROL_MOVEFORWARD:u32=0b00000001;
	const CONTROL_MOVEBACK:u32=0b00000010;
	const CONTROL_MOVERIGHT:u32=0b00000100;
	const CONTROL_MOVELEFT:u32=0b00001000;
	const CONTROL_MOVEUP:u32=0b00010000;
	const CONTROL_MOVEDOWN:u32=0b00100000;
	const CONTROL_JUMP:u32=0b01000000;
	const CONTROL_ZOOM:u32=0b10000000;

	const RIGHT_DIR:Planar64Vec3=Planar64Vec3::X;
	const UP_DIR:Planar64Vec3=Planar64Vec3::Y;
	const FORWARD_DIR:Planar64Vec3=Planar64Vec3::NEG_Z;

	fn get_control(&self,control:u32,controls:u32)->bool{
		controls&self.controls_mask&control==control
	}

	fn get_control_dir(&self,controls:u32)->Planar64Vec3{
		//don't get fancy just do it
		let mut control_dir:Planar64Vec3 = Planar64Vec3::ZERO;
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
			control_dir-=Self::FORWARD_DIR;
		}
		if controls & Self::CONTROL_MOVELEFT == Self::CONTROL_MOVELEFT {
			control_dir-=Self::RIGHT_DIR;
		}
		if controls & Self::CONTROL_MOVERIGHT == Self::CONTROL_MOVERIGHT {
			control_dir+=Self::RIGHT_DIR;
		}
		if controls & Self::CONTROL_MOVEUP == Self::CONTROL_MOVEUP {
			control_dir+=Self::UP_DIR;
		}
		if controls & Self::CONTROL_MOVEDOWN == Self::CONTROL_MOVEDOWN {
			control_dir-=Self::UP_DIR;
		}
		return control_dir
	}

	fn get_jump_power(&self)->Planar64Vec3{
		Planar64Vec3::int(0,715588,0)/(2*1000000/100)
	}
}

pub struct PhysicsState{
	pub time:Time,
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
	pub bvh:crate::bvh::BvhNode,
	
	pub modes:Vec<crate::model::ModeDescription>,
	pub mode_from_mode_id:std::collections::HashMap::<u32,usize>,
	//the spawn point is where you spawn when you load into the map.
	//This is not the same as Reset which teleports you to Spawn0
	pub spawn_point:Planar64Vec3,
}
#[derive(Clone)]
pub struct PhysicsOutputState{
	camera:PhysicsCamera,
	body:Body,
}
impl PhysicsOutputState{
	pub fn adjust_mouse(&self,mouse:&MouseState)->(glam::Vec3,glam::Vec2){
		((self.body.extrapolated_position(mouse.time)+self.camera.offset).into(),self.camera.simulate_move_angles(mouse.pos))
	}
}

//pretend to be using what we want to eventually do
type TreyMeshFace = crate::aabb::AabbFace;
type TreyMesh = crate::aabb::Aabb;

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
	transform:crate::integer::Planar64Affine3,
	attributes:PhysicsCollisionAttributes,
}

impl ModelPhysics {
	fn from_model_transform_attributes(model:&crate::model::IndexedModel,transform:&crate::integer::Planar64Affine3,attributes:PhysicsCollisionAttributes)->Self{
		let mut aabb=TreyMesh::default();
		for indexed_vertex in &model.unique_vertices {
			aabb.grow(transform.transform_point3(model.unique_pos[indexed_vertex.pos as usize]));
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
	pub fn unit_vertices(&self) -> [Planar64Vec3;8] {
		TreyMesh::unit_vertices()
	}
	pub fn mesh(&self) -> &TreyMesh {
		return &self.mesh;
	}
	// pub fn face_mesh(&self,face:TreyMeshFace)->TreyMesh{
	// 	self.mesh.face(face)
	// }
	pub fn face_normal(&self,face:TreyMeshFace) -> Planar64Vec3 {
		TreyMesh::normal(face)//this is wrong for scale
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
	// pub fn mesh(&self,models:&Vec<ModelPhysics>) -> TreyMesh {
	// 	return self.model(models).unwrap().face_mesh(self.face).clone()
	// }
	pub fn normal(&self,models:&Vec<ModelPhysics>) -> Planar64Vec3 {
		return self.model(models).unwrap().face_normal(self.face)
	}
}

impl Body {
	pub fn with_pva(position:Planar64Vec3,velocity:Planar64Vec3,acceleration:Planar64Vec3) -> Self {
		Self{
			position,
			velocity,
			acceleration,
			time:Time::ZERO,
		}
	}
	pub fn extrapolated_position(&self,time:Time)->Planar64Vec3{
		let dt=time-self.time;
		self.position+self.velocity*dt+self.acceleration*(dt*dt/2)
	}
	pub fn extrapolated_velocity(&self,time:Time)->Planar64Vec3{
		let dt=time-self.time;
		self.velocity+self.acceleration*dt
	}
	pub fn advance_time(&mut self,time:Time){
		self.position=self.extrapolated_position(time);
		self.velocity=self.extrapolated_velocity(time);
		self.time=time;
	}
}
impl std::fmt::Display for Body{
	fn fmt(&self,f:&mut std::fmt::Formatter<'_>)->std::fmt::Result{
		write!(f,"p({}) v({}) a({}) t({})",self.position,self.velocity,self.acceleration,self.time)
	}
}

impl Default for PhysicsState{
	fn default() -> Self {
 		Self{
			spawn_point:Planar64Vec3::int(0,50,0),
			body: Body::with_pva(Planar64Vec3::int(0,50,0),Planar64Vec3::int(0,0,0),Planar64Vec3::int(0,-100,0)),
			time: Time::ZERO,
			style:StyleModifiers::default(),
			grounded: false,
			contacts: std::collections::HashMap::new(),
			intersects: std::collections::HashMap::new(),
			models: Vec::new(),
			bvh:crate::bvh::BvhNode::default(),
			walk: WalkState::new(),
			camera: PhysicsCamera::from_offset(Planar64Vec3::int(0,2,0)),//4.5-2.5=2
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

	pub fn into_worker(mut self)->crate::worker::CompatWorker<TimedInstruction<InputInstruction>,PhysicsOutputState,Box<dyn FnMut(TimedInstruction<InputInstruction>)->PhysicsOutputState>>{
		let mut mouse_blocking=true;
		let mut last_mouse_time=self.next_mouse.time;
		let mut timeline=std::collections::VecDeque::new();
		crate::worker::CompatWorker::new(self.output(),Box::new(move |ins:TimedInstruction<InputInstruction>|{
			if if let Some(phys_input)=match ins.instruction{
				InputInstruction::MoveMouse(m)=>{
					if mouse_blocking{
						//tell the game state which is living in the past about its future
						timeline.push_front(TimedInstruction{
							time:last_mouse_time,
							instruction:PhysicsInputInstruction::SetNextMouse(MouseState{time:ins.time,pos:m}),
						});
					}else{
						//mouse has just started moving again after being still for longer than 10ms.
						//replace the entire mouse interpolation state to avoid an intermediate state with identical m0.t m1.t timestamps which will divide by zero
						timeline.push_front(TimedInstruction{
							time:last_mouse_time,
							instruction:PhysicsInputInstruction::ReplaceMouse(
								MouseState{time:last_mouse_time,pos:self.next_mouse.pos},
								MouseState{time:ins.time,pos:m}
							),
						});
						//delay physics execution until we have an interpolation target
						mouse_blocking=true;
					}
					last_mouse_time=ins.time;
					None
				},
				InputInstruction::MoveForward(s)=>Some(PhysicsInputInstruction::SetMoveForward(s)),
				InputInstruction::MoveLeft(s)=>Some(PhysicsInputInstruction::SetMoveLeft(s)),
				InputInstruction::MoveBack(s)=>Some(PhysicsInputInstruction::SetMoveBack(s)),
				InputInstruction::MoveRight(s)=>Some(PhysicsInputInstruction::SetMoveRight(s)),
				InputInstruction::MoveUp(s)=>Some(PhysicsInputInstruction::SetMoveUp(s)),
				InputInstruction::MoveDown(s)=>Some(PhysicsInputInstruction::SetMoveDown(s)),
				InputInstruction::Jump(s)=>Some(PhysicsInputInstruction::SetJump(s)),
				InputInstruction::Zoom(s)=>Some(PhysicsInputInstruction::SetZoom(s)),
				InputInstruction::Reset=>Some(PhysicsInputInstruction::Reset),
				InputInstruction::Idle=>Some(PhysicsInputInstruction::Idle),
			}{
				//non-mouse event
				timeline.push_back(TimedInstruction{
					time:ins.time,
					instruction:phys_input,
				});
				
				if mouse_blocking{
					//assume the mouse has stopped moving after 10ms.
					//shitty mice are 125Hz which is 8ms so this should cover that.
					//setting this to 100us still doesn't print even though it's 10x lower than the polling rate,
					//so mouse events are probably not handled separately from drawing and fire right before it :(
					if Time::from_millis(10)<ins.time-self.next_mouse.time{
						//push an event to extrapolate no movement from
						timeline.push_front(TimedInstruction{
							time:last_mouse_time,
							instruction:PhysicsInputInstruction::SetNextMouse(MouseState{time:ins.time,pos:self.next_mouse.pos}),
						});
						last_mouse_time=ins.time;
						//stop blocking. the mouse is not moving so the physics does not need to live in the past and wait for interpolation targets.
						mouse_blocking=false;
						true
					}else{
						false
					}
				}else{
					//keep this up to date so that it can be used as a known-timestamp
					//that the mouse was not moving when the mouse starts moving again
					last_mouse_time=ins.time;
					true
				}
			}else{
				//mouse event
				true
			}{
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
		}))
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
		self.bvh=crate::bvh::generate_bvh(self.models.iter().map(|m|m.mesh().clone()).collect());
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

	pub fn load_user_settings(&mut self,user_settings:&crate::settings::UserSettings){
		self.camera.sensitivity=user_settings.calculate_sensitivity();
	}

	pub fn get_mode(&self,mode_id:u32)->Option<&crate::model::ModeDescription>{
		if let Some(&mode)=self.mode_from_mode_id.get(&mode_id){
			self.modes.get(mode)
		}else{
			None
		}
	}
	//tickless gaming
	pub fn run(&mut self, time_limit:Time){
		//prepare is ommitted - everything is done via instructions.
		while let Some(instruction) = self.next_instruction(time_limit) {//collect
			//process
			self.process_instruction(instruction);
			//write hash lol
		}
	}

	pub fn advance_time(&mut self, time: Time){
		self.body.advance_time(time);
		self.time=time;
	}

	fn set_control(&mut self,control:u32,state:bool){
		self.controls=if state{self.controls|control}else{self.controls&!control};
	}
	fn jump(&mut self){
		self.grounded=false;//do I need this?
		let mut v=self.body.velocity+self.style.get_jump_power();
		self.contact_constrain_velocity(&mut v);
		self.body.velocity=v;
	}

	fn contact_constrain_velocity(&self,velocity:&mut Planar64Vec3){
		for (_,contact) in &self.contacts {
			let n=contact.normal(&self.models);
			let d=velocity.dot(n);
			if d<Planar64::ZERO{
				(*velocity)-=n*(d/n.dot(n));
			}
		}
	}
	fn contact_constrain_acceleration(&self,acceleration:&mut Planar64Vec3){
		for (_,contact) in &self.contacts {
			let n=contact.normal(&self.models);
			let d=acceleration.dot(n);
			if d<Planar64::ZERO{
				(*acceleration)-=n*(d/n.dot(n));
			}
		}
	}
	fn next_strafe_instruction(&self) -> Option<TimedInstruction<PhysicsInstruction>> {
		return Some(TimedInstruction{
			time:Time::from_nanos(self.style.strafe_tick_rate.rhs_div_int(self.style.strafe_tick_rate.mul_int(self.time.nanos())+1)),
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
			//remove normal component
			target_diff-=Planar64Vec3::Y*target_diff.y();
			if target_diff==Planar64Vec3::ZERO{
				let mut a=Planar64Vec3::ZERO;
				self.contact_constrain_acceleration(&mut a);
				self.body.acceleration=a;
				self.walk.state=WalkEnum::Reached;
			}else{
				//normal friction acceleration is clippedAcceleration.dot(normal)*friction
				let accel=self.style.walk_accel.min(self.style.gravity.dot(Planar64Vec3::NEG_Y)*self.style.friction);
				let time_delta=target_diff.length()/accel;
				let mut a=target_diff.with_length(accel);
				self.contact_constrain_acceleration(&mut a);
				self.body.acceleration=a;
				self.walk.target_time=self.body.time+Time::from(time_delta);
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
		let mut aabb=TreyMesh::default();
		for vertex in TreyMesh::unit_vertices(){
			aabb.grow(self.body.position+self.style.hitbox_halfsize*vertex);
		}
		aabb
	}
	fn predict_collision_end(&self,time:Time,time_limit:Time,collision_data:&RelativeCollision) -> Option<TimedInstruction<PhysicsInstruction>> {
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
			TreyMeshFace::Top|TreyMeshFace::Back|TreyMeshFace::Bottom|TreyMeshFace::Front=>{
				for t in zeroes2(mesh0.max.x()-mesh1.min.x(),v.x(),a.x()/2) {
					//negative t = back in time
					//must be moving towards surface to collide
					//must beat the current soonest collision time
					//must be moving towards surface
					let t_time=self.body.time-Time::from(t);
					if time<=t_time&&t_time<best_time&&Planar64::ZERO<v.x()+a.x()*-t{
						//collect valid t
						best_time=t_time;
						exit_face=Some(TreyMeshFace::Left);
						break;
					}
				}
				for t in zeroes2(mesh0.min.x()-mesh1.max.x(),v.x(),a.x()/2) {
					//negative t = back in time
					//must be moving towards surface to collide
					//must beat the current soonest collision time
					//must be moving towards surface
					let t_time=self.body.time-Time::from(t);
					if time<=t_time&&t_time<best_time&&v.x()+a.x()*-t<Planar64::ZERO{
						//collect valid t
						best_time=t_time;
						exit_face=Some(TreyMeshFace::Right);
						break;
					}
				}
			},
			TreyMeshFace::Left=>{
				//generate event if v.x<0||a.x<0
				if -v.x()<Planar64::ZERO{
					best_time=time;
					exit_face=Some(TreyMeshFace::Left);
				}
			},
			TreyMeshFace::Right=>{
				//generate event if 0<v.x||0<a.x
				if Planar64::ZERO<(-v.x()){
					best_time=time;
					exit_face=Some(TreyMeshFace::Right);
				}
			},
		}
		//collect y
		match collision_data.face {
			TreyMeshFace::Left|TreyMeshFace::Back|TreyMeshFace::Right|TreyMeshFace::Front=>{
				for t in zeroes2(mesh0.max.y()-mesh1.min.y(),v.y(),a.y()/2) {
					//negative t = back in time
					//must be moving towards surface to collide
					//must beat the current soonest collision time
					//must be moving towards surface
					let t_time=self.body.time-Time::from(t);
					if time<=t_time&&t_time<best_time&&Planar64::ZERO<v.y()+a.y()*-t{
						//collect valid t
						best_time=t_time;
						exit_face=Some(TreyMeshFace::Bottom);
						break;
					}
				}
				for t in zeroes2(mesh0.min.y()-mesh1.max.y(),v.y(),a.y()/2) {
					//negative t = back in time
					//must be moving towards surface to collide
					//must beat the current soonest collision time
					//must be moving towards surface
					let t_time=self.body.time-Time::from(t);
					if time<=t_time&&t_time<best_time&&v.y()+a.y()*-t<Planar64::ZERO{
						//collect valid t
						best_time=t_time;
						exit_face=Some(TreyMeshFace::Top);
						break;
					}
				}
			},
			TreyMeshFace::Bottom=>{
				//generate event if v.y<0||a.y<0
				if -v.y()<Planar64::ZERO{
					best_time=time;
					exit_face=Some(TreyMeshFace::Bottom);
				}
			},
			TreyMeshFace::Top=>{
				//generate event if 0<v.y||0<a.y
				if Planar64::ZERO<(-v.y()){
					best_time=time;
					exit_face=Some(TreyMeshFace::Top);
				}
			},
		}
		//collect z
		match collision_data.face {
			TreyMeshFace::Left|TreyMeshFace::Bottom|TreyMeshFace::Right|TreyMeshFace::Top=>{
				for t in zeroes2(mesh0.max.z()-mesh1.min.z(),v.z(),a.z()/2) {
					//negative t = back in time
					//must be moving towards surface to collide
					//must beat the current soonest collision time
					//must be moving towards surface
					let t_time=self.body.time-Time::from(t);
					if time<=t_time&&t_time<best_time&&Planar64::ZERO<v.z()+a.z()*-t{
						//collect valid t
						best_time=t_time;
						exit_face=Some(TreyMeshFace::Front);
						break;
					}
				}
				for t in zeroes2(mesh0.min.z()-mesh1.max.z(),v.z(),a.z()/2) {
					//negative t = back in time
					//must be moving towards surface to collide
					//must beat the current soonest collision time
					//must be moving towards surface
					let t_time=self.body.time-Time::from(t);
					if time<=t_time&&t_time<best_time&&v.z()+a.z()*-t<Planar64::ZERO{
						//collect valid t
						best_time=t_time;
						exit_face=Some(TreyMeshFace::Back);
						break;
					}
				}
			},
			TreyMeshFace::Front=>{
				//generate event if v.z<0||a.z<0
				if -v.z()<Planar64::ZERO{
					best_time=time;
					exit_face=Some(TreyMeshFace::Front);
				}
			},
			TreyMeshFace::Back=>{
				//generate event if 0<v.z||0<a.z
				if Planar64::ZERO<(-v.z()){
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
	fn predict_collision_start(&self,time:Time,time_limit:Time,model_id:u32) -> Option<TimedInstruction<PhysicsInstruction>> {
		let mesh0=self.mesh();
		let mesh1=self.models.get(model_id as usize).unwrap().mesh();
		let (p,v,a,body_time)=(self.body.position,self.body.velocity,self.body.acceleration,self.body.time);
		//find best t
		let mut best_time=time_limit;
		let mut best_face:Option<TreyMeshFace>=None;
		//collect x
		for t in zeroes2(mesh0.max.x()-mesh1.min.x(),v.x(),a.x()/2) {
			//must collide now or in the future
			//must beat the current soonest collision time
			//must be moving towards surface
			let t_time=body_time+Time::from(t);
			if time<=t_time&&t_time<best_time&&Planar64::ZERO<v.x()+a.x()*t{
				let dp=self.body.extrapolated_position(t_time)-p;
				//faces must be overlapping
				if mesh1.min.y()<mesh0.max.y()+dp.y()&&mesh0.min.y()+dp.y()<mesh1.max.y()&&mesh1.min.z()<mesh0.max.z()+dp.z()&&mesh0.min.z()+dp.z()<mesh1.max.z() {
					//collect valid t
					best_time=t_time;
					best_face=Some(TreyMeshFace::Left);
					break;
				}
			}
		}
		for t in zeroes2(mesh0.min.x()-mesh1.max.x(),v.x(),a.x()/2) {
			//must collide now or in the future
			//must beat the current soonest collision time
			//must be moving towards surface
			let t_time=body_time+Time::from(t);
			if time<=t_time&&t_time<best_time&&v.x()+a.x()*t<Planar64::ZERO{
				let dp=self.body.extrapolated_position(t_time)-p;
				//faces must be overlapping
				if mesh1.min.y()<mesh0.max.y()+dp.y()&&mesh0.min.y()+dp.y()<mesh1.max.y()&&mesh1.min.z()<mesh0.max.z()+dp.z()&&mesh0.min.z()+dp.z()<mesh1.max.z() {
					//collect valid t
					best_time=t_time;
					best_face=Some(TreyMeshFace::Right);
					break;
				}
			}
		}
		//collect y
		for t in zeroes2(mesh0.max.y()-mesh1.min.y(),v.y(),a.y()/2) {
			//must collide now or in the future
			//must beat the current soonest collision time
			//must be moving towards surface
			let t_time=body_time+Time::from(t);
			if time<=t_time&&t_time<best_time&&Planar64::ZERO<v.y()+a.y()*t{
				let dp=self.body.extrapolated_position(t_time)-p;
				//faces must be overlapping
				if mesh1.min.x()<mesh0.max.x()+dp.x()&&mesh0.min.x()+dp.x()<mesh1.max.x()&&mesh1.min.z()<mesh0.max.z()+dp.z()&&mesh0.min.z()+dp.z()<mesh1.max.z() {
					//collect valid t
					best_time=t_time;
					best_face=Some(TreyMeshFace::Bottom);
					break;
				}
			}
		}
		for t in zeroes2(mesh0.min.y()-mesh1.max.y(),v.y(),a.y()/2) {
			//must collide now or in the future
			//must beat the current soonest collision time
			//must be moving towards surface
			let t_time=body_time+Time::from(t);
			if time<=t_time&&t_time<best_time&&v.y()+a.y()*t<Planar64::ZERO{
				let dp=self.body.extrapolated_position(t_time)-p;
				//faces must be overlapping
				if mesh1.min.x()<mesh0.max.x()+dp.x()&&mesh0.min.x()+dp.x()<mesh1.max.x()&&mesh1.min.z()<mesh0.max.z()+dp.z()&&mesh0.min.z()+dp.z()<mesh1.max.z() {
					//collect valid t
					best_time=t_time;
					best_face=Some(TreyMeshFace::Top);
					break;
				}
			}
		}
		//collect z
		for t in zeroes2(mesh0.max.z()-mesh1.min.z(),v.z(),a.z()/2) {
			//must collide now or in the future
			//must beat the current soonest collision time
			//must be moving towards surface
			let t_time=body_time+Time::from(t);
			if time<=t_time&&t_time<best_time&&Planar64::ZERO<v.z()+a.z()*t{
				let dp=self.body.extrapolated_position(t_time)-p;
				//faces must be overlapping
				if mesh1.min.y()<mesh0.max.y()+dp.y()&&mesh0.min.y()+dp.y()<mesh1.max.y()&&mesh1.min.x()<mesh0.max.x()+dp.x()&&mesh0.min.x()+dp.x()<mesh1.max.x() {
					//collect valid t
					best_time=t_time;
					best_face=Some(TreyMeshFace::Front);
					break;
				}
			}
		}
		for t in zeroes2(mesh0.min.z()-mesh1.max.z(),v.z(),a.z()/2) {
			//must collide now or in the future
			//must beat the current soonest collision time
			//must be moving towards surface
			let t_time=body_time+Time::from(t);
			if time<=t_time&&t_time<best_time&&v.z()+a.z()*t<Planar64::ZERO{
				let dp=self.body.extrapolated_position(t_time)-p;
				//faces must be overlapping
				if mesh1.min.y()<mesh0.max.y()+dp.y()&&mesh0.min.y()+dp.y()<mesh1.max.y()&&mesh1.min.x()<mesh0.max.x()+dp.x()&&mesh0.min.x()+dp.x()<mesh1.max.x() {
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
	fn next_instruction(&self,time_limit:Time) -> Option<TimedInstruction<PhysicsInstruction>> {
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
		let mut aabb=crate::aabb::Aabb::default();
		aabb.grow(self.body.extrapolated_position(self.time));
		aabb.grow(self.body.extrapolated_position(time_limit));
		aabb.inflate(self.style.hitbox_halfsize);
		self.bvh.the_tester(&aabb,&mut |id|{
			if !(self.contacts.contains_key(&id)||self.intersects.contains_key(&id)){
				collector.collect(self.predict_collision_start(self.time,time_limit,id));
			}
		});
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
			PhysicsInstruction::Input(PhysicsInputInstruction::Idle)
			|PhysicsInstruction::Input(PhysicsInputInstruction::SetNextMouse(_))
			|PhysicsInstruction::Input(PhysicsInputInstruction::ReplaceMouse(_,_))
			|PhysicsInstruction::StrafeTick => (),
			_=>println!("{}|{:?}",ins.time,ins.instruction),
		}
		//selectively update body
		match &ins.instruction {
			//PhysicsInstruction::Input(InputInstruction::MoveMouse(_)) => (),//dodge time for mouse movement
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
								TreyMeshFace::Top => {
									//ground
									self.grounded=true;
								},
								_ => (),
							},
						}
						//check ground
						self.contacts.insert(c.model,c);
		match &general.teleport_behaviour{
			Some(crate::model::TeleportBehaviour::StageElement(stage_element))=>{
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
									self.body.position=model.transform.transform_point3(Planar64Vec3::Y)+Planar64Vec3::Y*(self.style.hitbox_halfsize.y()+Planar64::ONE/16);
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
			Some(crate::model::TeleportBehaviour::Wormhole(wormhole))=>{
				//telefart
			}
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
		match &general.teleport_behaviour{
			Some(crate::model::TeleportBehaviour::StageElement(stage_element))=>{
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
									self.body.position=model.transform.transform_point3(Planar64Vec3::Y)+Planar64Vec3::Y*(self.style.hitbox_halfsize.y()+Planar64::ONE/16);
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
			Some(crate::model::TeleportBehaviour::Wormhole(wormhole))=>{
				//telefart
			}
			None=>(),
		}
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
							TreyMeshFace::Top => {
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
					let mut v=self.body.velocity+control_dir*(self.style.mv-d);
					self.contact_constrain_velocity(&mut v);
					self.body.velocity=v;
				}
			}
			PhysicsInstruction::ReachWalkTargetVelocity => {
				//precisely set velocity
				let mut a=Planar64Vec3::ZERO;
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
					PhysicsInputInstruction::SetNextMouse(m) => {
						self.camera.move_mouse(self.next_mouse.pos);
						(self.camera.mouse,self.next_mouse)=(self.next_mouse.clone(),m);
					},
					PhysicsInputInstruction::ReplaceMouse(m0,m1) => {
						self.camera.move_mouse(m0.pos);
						(self.camera.mouse,self.next_mouse)=(m0,m1);
					},
					PhysicsInputInstruction::SetMoveForward(s) => self.set_control(StyleModifiers::CONTROL_MOVEFORWARD,s),
					PhysicsInputInstruction::SetMoveLeft(s) => self.set_control(StyleModifiers::CONTROL_MOVELEFT,s),
					PhysicsInputInstruction::SetMoveBack(s) => self.set_control(StyleModifiers::CONTROL_MOVEBACK,s),
					PhysicsInputInstruction::SetMoveRight(s) => self.set_control(StyleModifiers::CONTROL_MOVERIGHT,s),
					PhysicsInputInstruction::SetMoveUp(s) => self.set_control(StyleModifiers::CONTROL_MOVEUP,s),
					PhysicsInputInstruction::SetMoveDown(s) => self.set_control(StyleModifiers::CONTROL_MOVEDOWN,s),
					PhysicsInputInstruction::SetJump(s) => {
						self.set_control(StyleModifiers::CONTROL_JUMP,s);
						if self.grounded{
							self.jump();
						}
						refresh_walk_target_velocity=false;
					},
					PhysicsInputInstruction::SetZoom(s) => {
						self.set_control(StyleModifiers::CONTROL_ZOOM,s);
						refresh_walk_target=false;
					},
					PhysicsInputInstruction::Reset => {
						//temp
						self.body.position=self.spawn_point;
						self.body.velocity=Planar64Vec3::ZERO;
						//manual clear //for c in self.contacts{process_instruction(CollisionEnd(c))}
						self.contacts.clear();
						self.body.acceleration=self.style.gravity;
						self.walk.state=WalkEnum::Reached;
						self.grounded=false;
						refresh_walk_target=false;
					},
					PhysicsInputInstruction::Idle => {refresh_walk_target=false;},//literally idle!
				}
				if refresh_walk_target{
					//calculate walk target velocity
					if refresh_walk_target_velocity{
						let camera_mat=self.camera.simulate_move_rotation_y(self.camera.mouse.lerp(&self.next_mouse,self.time).x);
						let control_dir=camera_mat*self.style.get_control_dir(self.controls);
						self.walk.target_velocity=control_dir*self.style.walkspeed;
					}
					self.refresh_walk_target();
				}
			},
		}
	}
}

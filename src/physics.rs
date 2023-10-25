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

#[derive(Clone,Hash)]
pub struct Body {
	position: Planar64Vec3,//I64 where 2^32 = 1 u
	velocity: Planar64Vec3,//I64 where 2^32 = 1 u/s
	acceleration: Planar64Vec3,//I64 where 2^32 = 1 u/s/s
	time:Time,//nanoseconds x xxxxD!
}

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

enum WalkEnum{
	Reached,
	Transient(WalkTarget),
}
struct WalkTarget{
	velocity:Planar64Vec3,
	time:Time,
}
struct WalkState{
	normal:Planar64Vec3,
	state:WalkEnum,
}
impl WalkEnum{
	//args going crazy
	//(walk_enum,body.acceleration)=with_target_velocity();
	fn with_target_velocity(touching:&TouchingState,body:&Body,style:&StyleModifiers,models:&PhysicsModels,mut velocity:Planar64Vec3,normal:&Planar64Vec3)->(WalkEnum,Planar64Vec3){
		touching.constrain_velocity(models,&mut velocity);
		let mut target_diff=velocity-body.velocity;
		//remove normal component
		target_diff-=normal.clone()*(normal.dot(target_diff)/normal.dot(normal.clone()));
		if target_diff==Planar64Vec3::ZERO{
			let mut a=Planar64Vec3::ZERO;
			touching.constrain_acceleration(models,&mut a);
			(WalkEnum::Reached,a)
		}else{
			//normal friction acceleration is clippedAcceleration.dot(normal)*friction
			let diff_len=target_diff.length();
			let friction=if diff_len<style.walk_speed{
				style.static_friction
			}else{
				style.kinetic_friction
			};
			let accel=style.walk_accel.min(style.gravity.dot(Planar64Vec3::NEG_Y)*friction);
			let time_delta=diff_len/accel;
			let mut a=target_diff.with_length(accel);
			touching.constrain_acceleration(models,&mut a);
			(WalkEnum::Transient(WalkTarget{velocity,time:body.time+Time::from(time_delta)}),a)
		}
	}
}
impl WalkState{
	fn ground(touching:&TouchingState,body:&Body,style:&StyleModifiers,models:&PhysicsModels,velocity:Planar64Vec3)->(Self,Planar64Vec3){
		let (walk_enum,a)=WalkEnum::with_target_velocity(touching,body,style,models,velocity,&Planar64Vec3::Y);
		(Self{
			state:walk_enum,
			normal:Planar64Vec3::Y,
		},a)
	}
	fn ladder(touching:&TouchingState,body:&Body,style:&StyleModifiers,models:&PhysicsModels,velocity:Planar64Vec3,normal:&Planar64Vec3)->(Self,Planar64Vec3){
		let (walk_enum,a)=WalkEnum::with_target_velocity(touching,body,style,models,velocity,normal);
		(Self{
			state:walk_enum,
			normal:normal.clone(),
		},a)
	}
}

struct Modes{
	modes:Vec<crate::model::ModeDescription>,
	mode_from_mode_id:std::collections::HashMap::<u32,usize>,
}
impl Modes{
	fn clear(&mut self){
		self.modes.clear();
		self.mode_from_mode_id.clear();
	}
	fn get_mode(&self,mode_id:u32)->Option<&crate::model::ModeDescription>{
		self.modes.get(*self.mode_from_mode_id.get(&mode_id)?)
	}
	fn insert(&mut self,temp_map_mode_id:u32,mode:crate::model::ModeDescription){
		let mode_id=self.modes.len();
		self.mode_from_mode_id.insert(temp_map_mode_id,mode_id);
		self.modes.push(mode);
	}
}
impl Default for Modes{
	fn default() -> Self {
		Self{
			modes:Vec::new(),
			mode_from_mode_id:std::collections::HashMap::new(),
		}
	}
}

struct PhysicsModels{
	models:Vec<ModelPhysics>,
	model_id_from_wormhole_id:std::collections::HashMap::<u32,usize>,
}
impl PhysicsModels{
	fn clear(&mut self){
		self.models.clear();
		self.model_id_from_wormhole_id.clear();
	}
	fn get(&self,i:usize)->Option<&ModelPhysics>{
		self.models.get(i)
	}
	fn get_wormhole_model(&self,wormhole_id:u32)->Option<&ModelPhysics>{
		self.models.get(*self.model_id_from_wormhole_id.get(&wormhole_id)?)
	}
	fn push(&mut self,model:ModelPhysics)->usize{
		let model_id=self.models.len();
		self.models.push(model);
		model_id
	}
}
impl Default for PhysicsModels{
	fn default() -> Self {
		Self{
			models:Vec::new(),
			model_id_from_wormhole_id:std::collections::HashMap::new(),
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
	fn simulate_move_rotation(&self,mouse_pos:glam::IVec2)->Planar64Mat3{
		let a=-self.sensitivity.mul_int((mouse_pos-self.mouse.pos+self.clamped_mouse_pos).as_i64vec2());
		let ax=Angle32::wrap_from_i64(a.x);
		let ay=Angle32::clamp_from_i64(a.y)
		//clamp to actual vertical cam limit
		.clamp(self.angle_pitch_lower_limit,self.angle_pitch_upper_limit);
		Planar64Mat3::from_rotation_yx(ax,ay)
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

struct WorldState{}

enum JumpCalculation{
	Capped,//roblox
	Energy,//new
	Linear,//source
}

enum JumpImpulse{
	FromTime(Time),//jump time is invariant across mass and gravity changes
	FromHeight(Planar64),//jump height is invariant across mass and gravity changes
	FromDeltaV(Planar64),//jump velocity is invariant across mass and gravity changes
	FromEnergy(Planar64),// :)
}
//Jumping acts on dot(walks_state.normal,body.velocity)
//Capped means it increases the dot to the cap
//Energy means it adds energy
//Linear means it linearly adds on

struct StyleModifiers{
	controls_mask:u32,//controls which are unable to be activated
	controls_held:u32,//controls which must be active to be able to strafe
	strafe_tick_rate:Ratio64,
	jump_impulse:JumpImpulse,
	jump_calculation:JumpCalculation,
	static_friction:Planar64,
	kinetic_friction:Planar64,
	walk_speed:Planar64,
	walk_accel:Planar64,
	ladder_speed:Planar64,
	ladder_accel:Planar64,
	ladder_dot:Planar64,
	swim_speed:Planar64,
	mass:Planar64,
	mv:Planar64,
	air_accel_limit:Option<Planar64>,
	gravity:Planar64Vec3,
	hitbox_halfsize:Planar64Vec3,
}
impl std::default::Default for StyleModifiers{
	fn default() -> Self {
		Self::roblox_bhop()
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

	fn new()->Self{
		Self{
			controls_mask:!0,//&!(Self::CONTROL_MOVEUP|Self::CONTROL_MOVEDOWN),
			controls_held:0,
			strafe_tick_rate:Ratio64::new(128,Time::ONE_SECOND.nanos() as u64).unwrap(),
			jump_impulse:JumpImpulse::FromEnergy(Planar64::int(512)),
			jump_calculation:JumpCalculation::Energy,
			gravity:Planar64Vec3::int(0,-80,0),
			static_friction:Planar64::int(2),
			kinetic_friction:Planar64::int(3),//unrealistic: kinetic friction is typically lower than static
			mass:Planar64::int(1),
			mv:Planar64::int(2),
			air_accel_limit:None,
			walk_speed:Planar64::int(16),
			walk_accel:Planar64::int(80),
			ladder_speed:Planar64::int(16),
			ladder_accel:Planar64::int(160),
			ladder_dot:(Planar64::int(1)/2).sqrt(),
			swim_speed:Planar64::int(12),
			hitbox_halfsize:Planar64Vec3::int(2,5,2)/2,
		}
	}

	fn roblox_bhop()->Self{
		Self{
			controls_mask:!0,//&!(Self::CONTROL_MOVEUP|Self::CONTROL_MOVEDOWN),
			controls_held:0,
			strafe_tick_rate:Ratio64::new(100,Time::ONE_SECOND.nanos() as u64).unwrap(),
			jump_impulse:JumpImpulse::FromTime(Time::from_micros(715_588)),
			jump_calculation:JumpCalculation::Capped,
			gravity:Planar64Vec3::int(0,-100,0),
			static_friction:Planar64::int(2),
			kinetic_friction:Planar64::int(3),//unrealistic: kinetic friction is typically lower than static
			mass:Planar64::int(1),
			mv:Planar64::int(27)/10,
			air_accel_limit:None,
			walk_speed:Planar64::int(18),
			walk_accel:Planar64::int(90),
			ladder_speed:Planar64::int(18),
			ladder_accel:Planar64::int(180),
			ladder_dot:(Planar64::int(1)/2).sqrt(),
			swim_speed:Planar64::int(12),
			hitbox_halfsize:Planar64Vec3::int(2,5,2)/2,
		}
	}
	fn roblox_surf()->Self{
		Self{
			controls_mask:!0,//&!(Self::CONTROL_MOVEUP|Self::CONTROL_MOVEDOWN),
			controls_held:0,
			strafe_tick_rate:Ratio64::new(100,Time::ONE_SECOND.nanos() as u64).unwrap(),
			jump_impulse:JumpImpulse::FromTime(Time::from_micros(715_588)),
			jump_calculation:JumpCalculation::Capped,
			gravity:Planar64Vec3::int(0,-50,0),
			static_friction:Planar64::int(2),
			kinetic_friction:Planar64::int(3),//unrealistic: kinetic friction is typically lower than static
			mass:Planar64::int(1),
			mv:Planar64::int(27)/10,
			air_accel_limit:None,
			walk_speed:Planar64::int(18),
			walk_accel:Planar64::int(90),
			ladder_speed:Planar64::int(18),
			ladder_accel:Planar64::int(180),
			ladder_dot:(Planar64::int(1)/2).sqrt(),
			swim_speed:Planar64::int(12),
			hitbox_halfsize:Planar64Vec3::int(2,5,2)/2,
		}
	}

	fn source_bhop()->Self{
		//camera_offset=vec3(0,64/16-73/16/2,0),
		Self{
			controls_mask:!0,//&!(Self::CONTROL_MOVEUP|Self::CONTROL_MOVEDOWN),
			controls_held:0,
			strafe_tick_rate:Ratio64::new(100,Time::ONE_SECOND.nanos() as u64).unwrap(),
			jump_impulse:JumpImpulse::FromHeight(Planar64::raw(52<<28)),
			jump_calculation:JumpCalculation::Linear,
			gravity:Planar64Vec3::raw(0,-800<<28,0),
			static_friction:Planar64::int(2),//?
			kinetic_friction:Planar64::int(3),//?
			mass:Planar64::int(1),
			mv:Planar64::raw(30<<28),
			air_accel_limit:Some(Planar64::raw(150<<28)*66),
			walk_speed:Planar64::int(18),//?
			walk_accel:Planar64::int(90),//?
			ladder_speed:Planar64::int(18),//?
			ladder_accel:Planar64::int(180),//?
			ladder_dot:(Planar64::int(1)/2).sqrt(),//?
			swim_speed:Planar64::int(12),//?
			hitbox_halfsize:Planar64Vec3::raw(33<<28,73<<28,33<<28)/2,
		}
	}
	fn source_surf()->Self{
		//camera_offset=vec3(0,64/16-73/16/2,0),
		Self{
			controls_mask:!0,//&!(Self::CONTROL_MOVEUP|Self::CONTROL_MOVEDOWN),
			controls_held:0,
			strafe_tick_rate:Ratio64::new(66,Time::ONE_SECOND.nanos() as u64).unwrap(),
			jump_impulse:JumpImpulse::FromHeight(Planar64::raw(52<<28)),
			jump_calculation:JumpCalculation::Linear,
			gravity:Planar64Vec3::raw(0,-800<<28,0),
			static_friction:Planar64::int(2),//?
			kinetic_friction:Planar64::int(3),//?
			mass:Planar64::int(1),
			mv:Planar64::raw(30<<28),
			air_accel_limit:Some(Planar64::raw(150<<28)*66),
			walk_speed:Planar64::int(18),//?
			walk_accel:Planar64::int(90),//?
			ladder_speed:Planar64::int(18),//?
			ladder_accel:Planar64::int(180),//?
			ladder_dot:(Planar64::int(1)/2).sqrt(),//?
			swim_speed:Planar64::int(12),//?
			hitbox_halfsize:Planar64Vec3::raw(33<<28,73<<28,33<<28)/2,
		}
	}

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

	//fn get_jump_time(&self)->Planar64
	//fn get_jump_height(&self)->Planar64
	//fn get_jump_energy(&self)->Planar64
	fn get_jump_deltav(&self)->Planar64{
		match &self.jump_impulse{
			&JumpImpulse::FromTime(time)=>self.gravity.length()*(time/2),
			&JumpImpulse::FromHeight(height)=>(self.gravity.length()*height*2).sqrt(),
			&JumpImpulse::FromDeltaV(deltav)=>deltav,
			&JumpImpulse::FromEnergy(energy)=>(energy*2/self.mass).sqrt(),
		}
	}

	fn get_walk_target_velocity(&self,camera:&PhysicsCamera,controls:u32,next_mouse:&MouseState,time:Time)->Planar64Vec3{
		let camera_mat=camera.simulate_move_rotation_y(camera.mouse.lerp(&next_mouse,time).x);
		let control_dir=camera_mat*self.get_control_dir(controls);
		control_dir*self.walk_speed
	}
	fn get_ladder_target_velocity(&self,camera:&PhysicsCamera,controls:u32,next_mouse:&MouseState,time:Time)->Planar64Vec3{
		let camera_mat=camera.simulate_move_rotation(camera.mouse.lerp(&next_mouse,time));
		let control_dir=camera_mat*self.get_control_dir(controls);
		// local m=sqrt(ControlDir.length_squared())
		// local d=dot(Normal,ControlDir)/m
		// if d<-LadderDot then
		// 	ControlDir=Up*m
		// 	d=dot(Normal,Up)
		// elseif LadderDot<d then
		// 	ControlDir=Up*-m
		// 	d=-dot(Normal,Up)
		// end
		// return cross(cross(Normal,ControlDir),Normal)/sqrt(1-d*d)
		control_dir*self.walk_speed
	}
	fn get_propulsion_target_velocity(&self,camera:&PhysicsCamera,controls:u32,next_mouse:&MouseState,time:Time)->Planar64Vec3{
		let camera_mat=camera.simulate_move_rotation(camera.mouse.lerp(&next_mouse,time));
		let control_dir=camera_mat*self.get_control_dir(controls);
		control_dir*self.walk_speed
	}
}

enum MoveState{
	Air,
	Walk(WalkState),
	Water,
	Ladder(WalkState),
}

pub struct PhysicsState{
	time:Time,
	body:Body,
	world:WorldState,//currently there is only one state the world can be in
	game:GameMechanicsState,
	style:StyleModifiers,
	touching:TouchingState,
	//camera must exist in state because wormholes modify the camera, also camera punch
	camera:PhysicsCamera,
	pub next_mouse:MouseState,//Where is the mouse headed next
	controls:u32,
	move_state:MoveState,
	//all models
	models:PhysicsModels,
	bvh:crate::bvh::BvhNode,
	
	modes:Modes,
	//the spawn point is where you spawn when you load into the map.
	//This is not the same as Reset which teleports you to Spawn0
	spawn_point:Planar64Vec3,
}
#[derive(Clone)]
pub struct PhysicsOutputState{
	camera:PhysicsCamera,
	body:Body,
}
impl PhysicsOutputState{
	pub fn extrapolate(&self,mouse_pos:glam::IVec2,time:Time)->(glam::Vec3,glam::Vec2){
		((self.body.extrapolated_position(time)+self.camera.offset).into(),self.camera.simulate_move_angles(mouse_pos))
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
	face:TreyMeshFace,//just an id
	model:usize,//using id to avoid lifetimes
}

impl RelativeCollision {
	fn model<'a>(&self,models:&'a PhysicsModels)->Option<&'a ModelPhysics>{
		models.get(self.model)
	}
	// pub fn mesh(&self,models:&Vec<ModelPhysics>) -> TreyMesh {
	// 	return self.model(models).unwrap().face_mesh(self.face).clone()
	// }
	fn normal(&self,models:&PhysicsModels) -> Planar64Vec3 {
		return self.model(models).unwrap().face_normal(self.face)
	}
}

struct TouchingState{
	contacts:std::collections::HashMap::<usize,RelativeCollision>,
	intersects:std::collections::HashMap::<usize,RelativeCollision>,
}
impl TouchingState{
	fn clear(&mut self){
		self.contacts.clear();
		self.intersects.clear();
	}
	fn insert_contact(&mut self,model_id:usize,collision:RelativeCollision)->Option<RelativeCollision>{
		self.contacts.insert(model_id,collision)
	}
	fn remove_contact(&mut self,model_id:usize)->Option<RelativeCollision>{
		self.contacts.remove(&model_id)
	}
	fn insert_intersect(&mut self,model_id:usize,collision:RelativeCollision)->Option<RelativeCollision>{
		self.intersects.insert(model_id,collision)
	}
	fn remove_intersect(&mut self,model_id:usize)->Option<RelativeCollision>{
		self.intersects.remove(&model_id)
	}
	fn constrain_velocity(&self,models:&PhysicsModels,velocity:&mut Planar64Vec3){
		for (_,contact) in &self.contacts {
			let n=contact.normal(models);
			let d=velocity.dot(n);
			if d<Planar64::ZERO{
				(*velocity)-=n*(d/n.dot(n));
			}
		}
	}
	fn constrain_acceleration(&self,models:&PhysicsModels,acceleration:&mut Planar64Vec3){
		for (_,contact) in &self.contacts {
			let n=contact.normal(models);
			let d=acceleration.dot(n);
			if d<Planar64::ZERO{
				(*acceleration)-=n*(d/n.dot(n));
			}
		}
	}
}
impl Default for TouchingState{
	fn default() -> Self {
		Self{
			contacts: std::collections::HashMap::new(),
			intersects: std::collections::HashMap::new(),
		}
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
			touching:TouchingState::default(),
			models:PhysicsModels::default(),
			bvh:crate::bvh::BvhNode::default(),
			move_state: MoveState::Air,
			camera: PhysicsCamera::from_offset(Planar64Vec3::int(0,2,0)),//4.5-2.5=2
			next_mouse: MouseState::default(),
			controls: 0,
			world:WorldState{},
			game:GameMechanicsState::default(),
			modes:Modes::default(),
		}
	}
}

impl PhysicsState {
	pub fn clear(&mut self){
		self.models.clear();
		self.modes.clear();
		self.touching.clear();
	}

	pub fn output(&self)->PhysicsOutputState{
		PhysicsOutputState{
			body:self.body.clone(),
			camera:self.camera.clone(),
		}
	}

	pub fn spawn(&mut self,spawn_point:Planar64Vec3){
		self.game.stage_id=0;
		self.spawn_point=spawn_point;
		self.process_instruction(crate::instruction::TimedInstruction{
			time:self.time,
			instruction: PhysicsInstruction::Input(PhysicsInputInstruction::Reset),
		});
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
					let model_id=self.models.push(model_physics);
					for attr in &model_instance.temp_indexing{
						match attr{
							crate::model::TempIndexedAttributes::Start(s)=>starts.push((model_id,s.clone())),
							crate::model::TempIndexedAttributes::Spawn(s)=>spawns.push((model_id,s.clone())),
							crate::model::TempIndexedAttributes::OrderedCheckpoint(s)=>ordered_checkpoints.push((model_id,s.clone())),
							crate::model::TempIndexedAttributes::UnorderedCheckpoint(s)=>unordered_checkpoints.push((model_id,s.clone())),
							crate::model::TempIndexedAttributes::Wormhole(s)=>{self.models.model_id_from_wormhole_id.insert(s.wormhole_id,model_id);},
						}
					}
				}
			}
		}
		self.bvh=crate::bvh::generate_bvh(self.models.models.iter().map(|m|m.mesh().clone()).collect());
		//I don't wanna write structs for temporary structures
		//this code builds ModeDescriptions from the unsorted lists at the top of the function
		starts.sort_by_key(|tup|tup.1.mode_id);
		let mut mode_id_from_map_mode_id=std::collections::HashMap::new();
		let mut modedatas:Vec<(usize,Vec<(u32,usize)>,Vec<(u32,usize)>,Vec<usize>,u32)>=starts.into_iter().enumerate().map(|(i,(model_id,s))|{
			mode_id_from_map_mode_id.insert(s.mode_id,i);
			(model_id,Vec::new(),Vec::new(),Vec::new(),s.mode_id)
		}).collect();
		for (model_id,s) in spawns{
			if let Some(mode_id)=mode_id_from_map_mode_id.get(&s.mode_id){
				if let Some(modedata)=modedatas.get_mut(*mode_id){
					modedata.1.push((s.stage_id,model_id));
				}
			}
		}
		for (model_id,s) in ordered_checkpoints{
			if let Some(mode_id)=mode_id_from_map_mode_id.get(&s.mode_id){
				if let Some(modedata)=modedatas.get_mut(*mode_id){
					modedata.2.push((s.checkpoint_id,model_id));
				}
			}
		}
		for (model_id,s) in unordered_checkpoints{
			if let Some(mode_id)=mode_id_from_map_mode_id.get(&s.mode_id){
				if let Some(modedata)=modedatas.get_mut(*mode_id){
					modedata.3.push(model_id);
				}
			}
		}
		for mut tup in modedatas.into_iter(){
			tup.1.sort_by_key(|tup|tup.0);
			tup.2.sort_by_key(|tup|tup.0);
			let mut eshmep1=std::collections::HashMap::new();
			let mut eshmep2=std::collections::HashMap::new();
			self.modes.insert(tup.4,crate::model::ModeDescription{
				start:tup.0,
				spawns:tup.1.into_iter().enumerate().map(|(i,tup)|{eshmep1.insert(tup.0,i);tup.1}).collect(),
				ordered_checkpoints:tup.2.into_iter().enumerate().map(|(i,tup)|{eshmep2.insert(tup.0,i);tup.1}).collect(),
				unordered_checkpoints:tup.3,
				spawn_from_stage_id:eshmep1,
				ordered_checkpoint_from_checkpoint_id:eshmep2,
			});
		}
		println!("Physics Objects: {}",self.models.models.len());
	}

	pub fn load_user_settings(&mut self,user_settings:&crate::settings::UserSettings){
		self.camera.sensitivity=user_settings.calculate_sensitivity();
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
		match &self.move_state{
			MoveState::Walk(walk_state)|MoveState::Ladder(walk_state)=>{
				let mut v=self.body.velocity+walk_state.normal*self.style.get_jump_deltav();
				self.touching.constrain_velocity(&self.models,&mut v);
				self.body.velocity=v;
			},
			MoveState::Air|MoveState::Water=>(),
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
		match &mut self.move_state{
			MoveState::Air|MoveState::Water=>(),
			MoveState::Walk(WalkState{normal,state})=>{
				let n=normal;
				(*state,self.body.acceleration)=WalkEnum::with_target_velocity(&self.touching,&self.body,&self.style,&self.models,self.style.get_walk_target_velocity(&self.camera,self.controls,&self.next_mouse,self.time),&n);
			},
			MoveState::Ladder(WalkState{normal,state})=>{
				let n=normal;
				(*state,self.body.acceleration)=WalkEnum::with_target_velocity(&self.touching,&self.body,&self.style,&self.models,self.style.get_ladder_target_velocity(&self.camera,self.controls,&self.next_mouse,self.time),&n);
			},
		}
	}
	fn next_move_instruction(&self)->Option<TimedInstruction<PhysicsInstruction>>{
		//check if you have a valid walk state and create an instruction
		match &self.move_state{
			MoveState::Walk(walk_state)|MoveState::Ladder(walk_state)=>match &walk_state.state{
				WalkEnum::Transient(walk_target)=>Some(TimedInstruction{
					time:walk_target.time,
					instruction:PhysicsInstruction::ReachWalkTargetVelocity
				}),
				WalkEnum::Reached=>None,
			}
			MoveState::Air=>self.next_strafe_instruction(),
			MoveState::Water=>None,//TODO
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
	fn predict_collision_start(&self,time:Time,time_limit:Time,model_id:usize) -> Option<TimedInstruction<PhysicsInstruction>> {
		let mesh0=self.mesh();
		let mesh1=self.models.get(model_id).unwrap().mesh();
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
		if let Some(face)=best_face{
			return Some(TimedInstruction{
				time: best_time,
				instruction:PhysicsInstruction::CollisionStart(RelativeCollision{
					face,
					model:model_id
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
		//TODO: make this into a touching.next_instruction(&mut collector) member function
		for (_,collision_data) in &self.touching.contacts {
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
			if !(self.touching.contacts.contains_key(&id)||self.touching.intersects.contains_key(&id)){
				collector.collect(self.predict_collision_start(self.time,time_limit,id));
			}
		});
		collector.collect(self.next_move_instruction());
		collector.instruction()
	}
}

fn teleport(body:&mut Body,touching:&mut TouchingState,style:&StyleModifiers,point:Planar64Vec3)->MoveState{
	body.position=point;
	//manual clear //for c in contacts{process_instruction(CollisionEnd(c))}
	touching.clear();
	body.acceleration=style.gravity;
	MoveState::Air
	//TODO: calculate contacts and determine the actual state
	//touching.recalculate(body);
}

fn run_teleport_behaviour(teleport_behaviour:&Option<crate::model::TeleportBehaviour>,game:&mut GameMechanicsState,models:&PhysicsModels,modes:&Modes,style:&StyleModifiers,touching:&mut TouchingState,body:&mut Body,model:&ModelPhysics)->Option<MoveState>{
	match teleport_behaviour{
		Some(crate::model::TeleportBehaviour::StageElement(stage_element))=>{
			if stage_element.force||game.stage_id<stage_element.stage_id{
				game.stage_id=stage_element.stage_id;
			}
			match &stage_element.behaviour{
				crate::model::StageElementBehaviour::SpawnAt=>None,
				crate::model::StageElementBehaviour::Trigger
				|crate::model::StageElementBehaviour::Teleport=>{
					//I guess this is correct behaviour when trying to teleport to a non-existent spawn but it's still weird
					let model=models.get(*modes.get_mode(stage_element.mode_id)?.get_spawn_model_id(game.stage_id)? as usize)?;
					let point=model.transform.transform_point3(Planar64Vec3::Y)+Planar64Vec3::Y*(style.hitbox_halfsize.y()+Planar64::ONE/16);
					Some(teleport(body,touching,style,point))
				},
				crate::model::StageElementBehaviour::Platform=>None,
				crate::model::StageElementBehaviour::JumpLimit(_)=>None,//TODO
			}
		},
		Some(crate::model::TeleportBehaviour::Wormhole(wormhole))=>{
			let origin_model=model;
			let destination_model=models.get_wormhole_model(wormhole.destination_model_id)?;
			//ignore the transform for now
			Some(teleport(body,touching,style,body.position-origin_model.transform.translation+destination_model.transform.translation))
		}
		None=>None,
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
			PhysicsInstruction::Input(PhysicsInputInstruction::Idle)=>self.time=ins.time,//idle simply updates time
			PhysicsInstruction::Input(_)
			|PhysicsInstruction::ReachWalkTargetVelocity
			|PhysicsInstruction::CollisionStart(_)
			|PhysicsInstruction::CollisionEnd(_)
			|PhysicsInstruction::StrafeTick=>self.advance_time(ins.time),
		}
		match ins.instruction {
			PhysicsInstruction::CollisionStart(c) => {
				let model=c.model(&self.models).unwrap();
				match &model.attributes{
					PhysicsCollisionAttributes::Contact{contacting,general}=>{
						let mut v=self.body.velocity;
						match &contacting.contact_behaviour{
							Some(crate::model::ContactingBehaviour::Surf)=>println!("I'm surfing!"),
							&Some(crate::model::ContactingBehaviour::Elastic(elasticity))=>{
								let n=c.normal(&self.models);
								let d=n.dot(v)*Planar64::raw(-1-elasticity as i64);
								v-=n*(d/n.dot(n));
							},
							Some(crate::model::ContactingBehaviour::Ladder(contacting_ladder))=>{
								if contacting_ladder.sticky{
									//kill v
									v=Planar64Vec3::ZERO;//model.velocity
								}
								//ladder walkstate
								let (walk_state,a)=WalkState::ladder(&self.touching,&self.body,&self.style,&self.models,self.style.get_ladder_target_velocity(&self.camera,self.controls,&self.next_mouse,self.time),&c.normal(&self.models));
								self.move_state=MoveState::Ladder(walk_state);
								self.body.acceleration=a;
							}
							None=>match &c.face {
								TreyMeshFace::Top => {
									//ground
									let (walk_state,a)=WalkState::ground(&self.touching,&self.body,&self.style,&self.models,self.style.get_walk_target_velocity(&self.camera,self.controls,&self.next_mouse,self.time));
									self.move_state=MoveState::Walk(walk_state);
									self.body.acceleration=a;
								},
								_ => (),
							},
						}
						//check ground
						self.touching.insert_contact(c.model,c);
						//I love making functions with 10 arguments to dodge the borrow checker
						run_teleport_behaviour(&general.teleport_behaviour,&mut self.game,&self.models,&self.modes,&self.style,&mut self.touching,&mut self.body,model);
						//flatten v
						self.touching.constrain_velocity(&self.models,&mut v);
						match &general.booster{
							Some(booster)=>{
								match booster{
									&crate::model::GameMechanicBooster::Affine(transform)=>v=transform.transform_point3(v),
									&crate::model::GameMechanicBooster::Velocity(velocity)=>v+=velocity,
									&crate::model::GameMechanicBooster::Energy{direction,energy}=>todo!(),
								}
								self.touching.constrain_velocity(&self.models,&mut v);
							},
							None=>(),
						}
						match &general.trajectory{
							Some(trajectory)=>{
								match trajectory{
									crate::model::GameMechanicSetTrajectory::AirTime(_) => todo!(),
									crate::model::GameMechanicSetTrajectory::Height(_) => todo!(),
									crate::model::GameMechanicSetTrajectory::TargetPointTime { target_point, time } => todo!(),
									crate::model::GameMechanicSetTrajectory::TrajectoryTargetPoint { target_point, speed, trajectory_choice } => todo!(),
									&crate::model::GameMechanicSetTrajectory::Velocity(velocity)=>v=velocity,
									crate::model::GameMechanicSetTrajectory::DotVelocity { direction, dot } => todo!(),
								}
								self.touching.constrain_velocity(&self.models,&mut v);
							},
							None=>(),
						}
						self.body.velocity=v;
						if self.style.get_control(StyleModifiers::CONTROL_JUMP,self.controls){
							self.jump();
						}
						self.refresh_walk_target();
					},
					PhysicsCollisionAttributes::Intersect{intersecting,general}=>{
						//I think that setting the velocity to 0 was preventing surface contacts from entering an infinite loop
						self.touching.insert_intersect(c.model,c);
						run_teleport_behaviour(&general.teleport_behaviour,&mut self.game,&self.models,&self.modes,&self.style,&mut self.touching,&mut self.body,model);
					},
				}
			},
			PhysicsInstruction::CollisionEnd(c) => {
				let model=c.model(&self.models).unwrap();
				match &model.attributes{
					PhysicsCollisionAttributes::Contact{contacting,general}=>{
						self.touching.remove_contact(c.model);//remove contact before calling contact_constrain_acceleration
						let mut a=self.style.gravity;
						self.touching.constrain_acceleration(&self.models,&mut a);
						self.body.acceleration=a;
						//check ground
						//self.touching.get_move_state();
						match &c.face {
							TreyMeshFace::Top => {
								//TODO: make this more advanced checking contacts
								self.move_state=MoveState::Air;
							},
							_=>self.refresh_walk_target(),
						}
					},
					PhysicsCollisionAttributes::Intersect{intersecting,general}=>{
						self.touching.remove_intersect(c.model);
					},
				}
			},
			PhysicsInstruction::StrafeTick => {
				let camera_mat=self.camera.simulate_move_rotation_y(self.camera.mouse.lerp(&self.next_mouse,self.time).x);
				let control_dir=camera_mat*self.style.get_control_dir(self.controls);
				let d=self.body.velocity.dot(control_dir);
				if d<self.style.mv {
					let mut v=self.body.velocity+control_dir*(self.style.mv-d);
					self.touching.constrain_velocity(&self.models,&mut v);
					self.body.velocity=v;
				}
			}
			PhysicsInstruction::ReachWalkTargetVelocity => {
				match &mut self.move_state{
					MoveState::Air|MoveState::Water=>(),
					MoveState::Walk(walk_state)|MoveState::Ladder(walk_state)=>{
						match &mut walk_state.state{
							WalkEnum::Reached=>(),
							WalkEnum::Transient(walk_target)=>{
								//precisely set velocity
								let mut a=self.style.gravity;
								self.touching.constrain_acceleration(&self.models,&mut a);
								self.body.acceleration=a;
								let mut v=walk_target.velocity;
								self.touching.constrain_velocity(&self.models,&mut v);
								self.body.velocity=v;
								walk_state.state=WalkEnum::Reached;
							},
						}
					}
				}
			},
			PhysicsInstruction::Input(input_instruction) => {
				let mut refresh_walk_target=true;
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
						self.jump();
						refresh_walk_target=false;
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
						self.touching.clear();
						self.body.acceleration=self.style.gravity;
						self.move_state=MoveState::Air;
						refresh_walk_target=false;
					},
					PhysicsInputInstruction::Idle => {refresh_walk_target=false;},//literally idle!
				}
				if refresh_walk_target{
					self.refresh_walk_target();
				}
			},
		}
	}
}

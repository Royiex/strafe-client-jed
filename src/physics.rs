use crate::instruction::{InstructionEmitter,InstructionConsumer,TimedInstruction};
use crate::integer::{Time,Planar64,Planar64Vec3,Planar64Mat3,Angle32,Ratio64,Ratio64Vec2};
use crate::model_physics::{PhysicsMesh,TransformedMesh,MeshQuery};

#[derive(Debug)]
pub enum PhysicsInstruction {
	CollisionStart(Collision),
	CollisionEnd(Collision),
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
		//Idle: there were no input events, but the simulation is safe to advance to this timestep
		//for interpolation / networking / playback reasons, most playback heads will always want
		//to be 1 instruction ahead to generate the next state for interpolation.
}

#[derive(Clone,Hash,Default)]
pub struct Body{
	pub position:Planar64Vec3,//I64 where 2^32 = 1 u
	pub velocity:Planar64Vec3,//I64 where 2^32 = 1 u/s
	pub acceleration:Planar64Vec3,//I64 where 2^32 = 1 u/s/s
	pub time:Time,//nanoseconds x xxxxD!
}
impl std::ops::Neg for Body{
	type Output=Self;
	fn neg(self)->Self::Output{
		Self{
			position:self.position,
			velocity:-self.velocity,
			acceleration:self.acceleration,
			time:-self.time,
		}
	}
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

enum JumpDirection{
	Exactly(Planar64Vec3),
	FromContactNormal,
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
	jump_direction:JumpDirection,
	contact:ContactCollision,
	state:WalkEnum,
}
impl WalkEnum{
	//args going crazy
	//(walk_enum,body.acceleration)=with_target_velocity();
	fn with_target_velocity(body:&Body,style:&StyleModifiers,velocity:Planar64Vec3,normal:&Planar64Vec3,speed:Planar64,normal_accel:Planar64)->(WalkEnum,Planar64Vec3){
		let mut target_diff=velocity-body.velocity;
		//remove normal component
		target_diff-=normal.clone()*(normal.dot(target_diff)/normal.dot(normal.clone()));
		if target_diff==Planar64Vec3::ZERO{
			(WalkEnum::Reached,Planar64Vec3::ZERO)
		}else{
			//normal friction acceleration is clippedAcceleration.dot(normal)*friction
			let diff_len=target_diff.length();
			let friction=if diff_len<speed{
				style.static_friction
			}else{
				style.kinetic_friction
			};
			let accel=style.walk_accel.min(normal_accel*friction);
			let time_delta=diff_len/accel;
			let a=target_diff.with_length(accel);
			(WalkEnum::Transient(WalkTarget{velocity,time:body.time+Time::from(time_delta)}),a)
		}
	}
}
impl WalkState{
	fn ground(body:&Body,style:&StyleModifiers,gravity:Planar64Vec3,velocity:Planar64Vec3,contact:ContactCollision,normal:&Planar64Vec3)->(Self,Planar64Vec3){
		let (walk_enum,a)=WalkEnum::with_target_velocity(body,style,velocity,&Planar64Vec3::Y,style.walk_speed,-normal.dot(gravity));
		(Self{
			state:walk_enum,
			contact,
			jump_direction:JumpDirection::Exactly(Planar64Vec3::Y),
		},a)
	}
	fn ladder(body:&Body,style:&StyleModifiers,gravity:Planar64Vec3,velocity:Planar64Vec3,contact:ContactCollision,normal:&Planar64Vec3)->(Self,Planar64Vec3){
		let (walk_enum,a)=WalkEnum::with_target_velocity(body,style,velocity,normal,style.ladder_speed,style.ladder_accel);
		(Self{
			state:walk_enum,
			contact,
			jump_direction:JumpDirection::FromContactNormal,
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

#[derive(Default)]
struct PhysicsModels{
	meshes:Vec<PhysicsMesh>,
	models:Vec<PhysicsModel>,
	//separate models into Contacting and Intersecting?
	//wrap model id with ContactingModelId and IntersectingModelId
	//attributes can be split into contacting and intersecting (this also saves a bit of memory)
	//can go even further and deduplicate General attributes separately, reconstructing it when queried
	attributes:Vec<PhysicsCollisionAttributes>,
	model_id_from_wormhole_id:std::collections::HashMap::<u32,usize>,
}
impl PhysicsModels{
	fn clear(&mut self){
		self.meshes.clear();
		self.models.clear();
		self.attributes.clear();
		self.model_id_from_wormhole_id.clear();
	}
	fn aabb_list(&self)->Vec<crate::aabb::Aabb>{
		self.models.iter().map(|model|{
			let mut aabb=crate::aabb::Aabb::default();
			for pos in self.meshes[model.mesh_id].verts(){
				aabb.grow(model.transform.transform_point3(pos));
			}
			aabb
		}).collect()
	}
	//TODO: "statically" verify the maps don't refer to any nonexistant data, if they do delete the references.
	//then I can make these getter functions unchecked.
	fn mesh(&self,model_id:usize)->TransformedMesh{
		TransformedMesh::new(
			&self.meshes[self.models[model_id].mesh_id],
			&self.models[model_id].transform,
			&self.models[model_id].normal_transform,
		)
	}
	fn model(&self,model_id:usize)->&PhysicsModel{
		&self.models[model_id]
	}
	fn attr(&self,model_id:usize)->&PhysicsCollisionAttributes{
		&self.attributes[self.models[model_id].attr_id]
	}
	fn get_wormhole_model(&self,wormhole_id:u32)->Option<&PhysicsModel>{
		self.models.get(*self.model_id_from_wormhole_id.get(&wormhole_id)?)
	}
	fn push_mesh(&mut self,mesh:PhysicsMesh){
		self.meshes.push(mesh);
	}
	fn push_model(&mut self,model:PhysicsModel)->usize{
		let model_id=self.models.len();
		self.models.push(model);
		model_id
	}
	fn push_attr(&mut self,attr:PhysicsCollisionAttributes)->usize{
		let attr_id=self.attributes.len();
		self.attributes.push(attr);
		attr_id
	}
}

#[derive(Clone)]
pub struct PhysicsCamera{
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

impl std::default::Default for PhysicsCamera{
	fn default()->Self{
		Self{
			sensitivity:Ratio64Vec2::ONE*200_000,
			mouse:MouseState::default(),//t=0 does not cause divide by zero because it's immediately replaced
			clamped_mouse_pos:glam::IVec2::ZERO,
			angle_pitch_lower_limit:-Angle32::FRAC_PI_2,
			angle_pitch_upper_limit:Angle32::FRAC_PI_2,
		}
	}
}

pub struct GameMechanicsState{
	stage_id:u32,
	jump_counts:std::collections::HashMap<usize,u32>,//model_id -> jump count
	next_ordered_checkpoint_id:u32,//which OrderedCheckpoint model_id you must pass next (if 0 you haven't passed OrderedCheckpoint0)
	unordered_checkpoints:std::collections::HashSet<usize>,//hashset of UnorderedCheckpoint model ids
}
impl std::default::Default for GameMechanicsState{
	fn default()->Self{
		Self{
			stage_id:0,
			next_ordered_checkpoint_id:0,
			unordered_checkpoints:std::collections::HashSet::new(),
			jump_counts:std::collections::HashMap::new(),
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

enum EnableStrafe{
	Always,
	MaskAny(u32),//hsw, shsw
	MaskAll(u32),
	//Function(Box<dyn Fn(u32)->bool>),
}

struct StrafeSettings{
	enable:EnableStrafe,
	air_accel_limit:Option<Planar64>,
	tick_rate:Ratio64,
}

struct Hitbox{
	halfsize:Planar64Vec3,
	mesh:PhysicsMesh,
	transform:crate::integer::Planar64Affine3,
	normal_transform:Planar64Mat3,
}
impl Hitbox{
	fn from_mesh_scale(mesh:PhysicsMesh,scale:Planar64Vec3)->Self{
		Self{
			halfsize:scale,
			mesh,
			transform:crate::integer::Planar64Affine3::new(Planar64Mat3::from_diagonal(scale),Planar64Vec3::ZERO),
			normal_transform:Planar64Mat3::from_diagonal(scale).inverse().transpose(),
		}
	}
	fn from_mesh_scale_offset(mesh:PhysicsMesh,scale:Planar64Vec3,offset:Planar64Vec3)->Self{
		Self{
			halfsize:scale,
			mesh,
			transform:crate::integer::Planar64Affine3::new(Planar64Mat3::from_diagonal(scale),offset),
			normal_transform:Planar64Mat3::from_diagonal(scale).inverse().transpose(),
		}
	}
	fn roblox()->Self{
		Self::from_mesh_scale(PhysicsMesh::from(&crate::primitives::unit_cylinder()),Planar64Vec3::int(2,5,2)/2)
	}
	fn source()->Self{
		Self::from_mesh_scale(PhysicsMesh::from(&crate::primitives::unit_cube()),Planar64Vec3::raw(33<<28,73<<28,33<<28)/2)
	}
	#[inline]
	fn transformed_mesh(&self)->TransformedMesh{
		TransformedMesh::new(&self.mesh,&self.transform,&self.normal_transform)
	}
}

struct StyleModifiers{
	controls_used:u32,//controls which are allowed to pass into gameplay
	controls_mask:u32,//controls which are masked from control state (e.g. jump in scroll style)
	strafe:Option<StrafeSettings>,
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
	surf_slope:Option<Planar64>,
	rocket_force:Option<Planar64>,
	gravity:Planar64Vec3,
	hitbox:Hitbox,
	camera_offset:Planar64Vec3,
}
impl std::default::Default for StyleModifiers{
	fn default()->Self{
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

	fn neo()->Self{
		Self{
			controls_used:!0,
			controls_mask:!0,//&!(Self::CONTROL_MOVEUP|Self::CONTROL_MOVEDOWN),
			strafe:Some(StrafeSettings{
				enable:EnableStrafe::Always,
				air_accel_limit:None,
				tick_rate:Ratio64::new(64,Time::ONE_SECOND.nanos() as u64).unwrap(),
			}),
			jump_impulse:JumpImpulse::FromEnergy(Planar64::int(512)),
			jump_calculation:JumpCalculation::Energy,
			gravity:Planar64Vec3::int(0,-80,0),
			static_friction:Planar64::int(2),
			kinetic_friction:Planar64::int(3),//unrealistic: kinetic friction is typically lower than static
			mass:Planar64::int(1),
			mv:Planar64::int(3),
			rocket_force:None,
			walk_speed:Planar64::int(16),
			walk_accel:Planar64::int(80),
			ladder_speed:Planar64::int(16),
			ladder_accel:Planar64::int(160),
			ladder_dot:(Planar64::int(1)/2).sqrt(),
			swim_speed:Planar64::int(12),
			surf_slope:Some(Planar64::raw(7)/8),
			hitbox:Hitbox::roblox(),
			camera_offset:Planar64Vec3::int(0,2,0),//4.5-2.5=2
		}
	}

	fn roblox_bhop()->Self{
		Self{
			controls_used:!0,
			controls_mask:!0,//&!(Self::CONTROL_MOVEUP|Self::CONTROL_MOVEDOWN),
			strafe:Some(StrafeSettings{
				enable:EnableStrafe::Always,
				air_accel_limit:None,
				tick_rate:Ratio64::new(100,Time::ONE_SECOND.nanos() as u64).unwrap(),
			}),
			jump_impulse:JumpImpulse::FromTime(Time::from_micros(715_588)),
			jump_calculation:JumpCalculation::Capped,
			gravity:Planar64Vec3::int(0,-100,0),
			static_friction:Planar64::int(2),
			kinetic_friction:Planar64::int(3),//unrealistic: kinetic friction is typically lower than static
			mass:Planar64::int(1),
			mv:Planar64::int(27)/10,
			rocket_force:None,
			walk_speed:Planar64::int(18),
			walk_accel:Planar64::int(90),
			ladder_speed:Planar64::int(18),
			ladder_accel:Planar64::int(180),
			ladder_dot:(Planar64::int(1)/2).sqrt(),
			swim_speed:Planar64::int(12),
			surf_slope:Some(Planar64::raw(3787805118)),// normal.y=0.75
			hitbox:Hitbox::roblox(),
			camera_offset:Planar64Vec3::int(0,2,0),//4.5-2.5=2
		}
	}
	fn roblox_surf()->Self{
		Self{
			controls_used:!0,
			controls_mask:!0,//&!(Self::CONTROL_MOVEUP|Self::CONTROL_MOVEDOWN),
			strafe:Some(StrafeSettings{
				enable:EnableStrafe::Always,
				air_accel_limit:None,
				tick_rate:Ratio64::new(100,Time::ONE_SECOND.nanos() as u64).unwrap(),
			}),
			jump_impulse:JumpImpulse::FromTime(Time::from_micros(715_588)),
			jump_calculation:JumpCalculation::Capped,
			gravity:Planar64Vec3::int(0,-50,0),
			static_friction:Planar64::int(2),
			kinetic_friction:Planar64::int(3),//unrealistic: kinetic friction is typically lower than static
			mass:Planar64::int(1),
			mv:Planar64::int(27)/10,
			rocket_force:None,
			walk_speed:Planar64::int(18),
			walk_accel:Planar64::int(90),
			ladder_speed:Planar64::int(18),
			ladder_accel:Planar64::int(180),
			ladder_dot:(Planar64::int(1)/2).sqrt(),
			swim_speed:Planar64::int(12),
			surf_slope:Some(Planar64::raw(3787805118)),// normal.y=0.75
			hitbox:Hitbox::roblox(),
			camera_offset:Planar64Vec3::int(0,2,0),//4.5-2.5=2
		}
	}

	fn source_bhop()->Self{
		Self{
			controls_used:!0,
			controls_mask:!0,//&!(Self::CONTROL_MOVEUP|Self::CONTROL_MOVEDOWN),
			strafe:Some(StrafeSettings{
				enable:EnableStrafe::Always,
				air_accel_limit:Some(Planar64::raw(150<<28)*66),
				tick_rate:Ratio64::new(100,Time::ONE_SECOND.nanos() as u64).unwrap(),
			}),
			jump_impulse:JumpImpulse::FromHeight(Planar64::raw(52<<28)),
			jump_calculation:JumpCalculation::Linear,
			gravity:Planar64Vec3::raw(0,-800<<28,0),
			static_friction:Planar64::int(2),//?
			kinetic_friction:Planar64::int(3),//?
			mass:Planar64::int(1),
			mv:Planar64::raw(30<<28),
			rocket_force:None,
			walk_speed:Planar64::int(18),//?
			walk_accel:Planar64::int(90),//?
			ladder_speed:Planar64::int(18),//?
			ladder_accel:Planar64::int(180),//?
			ladder_dot:(Planar64::int(1)/2).sqrt(),//?
			swim_speed:Planar64::int(12),//?
			surf_slope:Some(Planar64::raw(3787805118)),// normal.y=0.75
			hitbox:Hitbox::source(),
			camera_offset:Planar64Vec3::raw(0,(64<<28)-(73<<27),0),
		}
	}
	fn source_surf()->Self{
		Self{
			controls_used:!0,
			controls_mask:!0,//&!(Self::CONTROL_MOVEUP|Self::CONTROL_MOVEDOWN),
			strafe:Some(StrafeSettings{
				enable:EnableStrafe::Always,
				air_accel_limit:Some(Planar64::raw(150<<28)*66),
				tick_rate:Ratio64::new(66,Time::ONE_SECOND.nanos() as u64).unwrap(),
			}),
			jump_impulse:JumpImpulse::FromHeight(Planar64::raw(52<<28)),
			jump_calculation:JumpCalculation::Linear,
			gravity:Planar64Vec3::raw(0,-800<<28,0),
			static_friction:Planar64::int(2),//?
			kinetic_friction:Planar64::int(3),//?
			mass:Planar64::int(1),
			mv:Planar64::raw(30<<28),
			rocket_force:None,
			walk_speed:Planar64::int(18),//?
			walk_accel:Planar64::int(90),//?
			ladder_speed:Planar64::int(18),//?
			ladder_accel:Planar64::int(180),//?
			ladder_dot:(Planar64::int(1)/2).sqrt(),//?
			swim_speed:Planar64::int(12),//?
			surf_slope:Some(Planar64::raw(3787805118)),// normal.y=0.75
			hitbox:Hitbox::source(),
			camera_offset:Planar64Vec3::raw(0,(64<<28)-(73<<27),0),
		}
	}
	fn roblox_rocket()->Self{
		Self{
			controls_used:!0,
			controls_mask:!0,
			strafe:None,
			jump_impulse:JumpImpulse::FromTime(Time::from_micros(715_588)),
			jump_calculation:JumpCalculation::Capped,
			gravity:Planar64Vec3::int(0,-100,0),
			static_friction:Planar64::int(2),
			kinetic_friction:Planar64::int(3),//unrealistic: kinetic friction is typically lower than static
			mass:Planar64::int(1),
			mv:Planar64::int(27)/10,
			rocket_force:Some(Planar64::int(200)),
			walk_speed:Planar64::int(18),
			walk_accel:Planar64::int(90),
			ladder_speed:Planar64::int(18),
			ladder_accel:Planar64::int(180),
			ladder_dot:(Planar64::int(1)/2).sqrt(),
			swim_speed:Planar64::int(12),
			surf_slope:Some(Planar64::raw(3787805118)),// normal.y=0.75
			hitbox:Hitbox::roblox(),
			camera_offset:Planar64Vec3::int(0,2,0),//4.5-2.5=2
		}
	}

	fn get_control(&self,control:u32,controls:u32)->bool{
		controls&self.controls_mask&control==control
	}

	fn allow_strafe(&self,controls:u32)->bool{
		//disable strafing according to strafe settings
		match &self.strafe{
			Some(StrafeSettings{enable:EnableStrafe::Always,air_accel_limit:_,tick_rate:_})=>true,
			&Some(StrafeSettings{enable:EnableStrafe::MaskAny(mask),air_accel_limit:_,tick_rate:_})=>mask&controls!=0,
			&Some(StrafeSettings{enable:EnableStrafe::MaskAll(mask),air_accel_limit:_,tick_rate:_})=>mask&controls==mask,
			None=>false,
		}
	}

	fn get_control_dir(&self,controls:u32)->Planar64Vec3{
		//don't get fancy just do it
		let mut control_dir:Planar64Vec3 = Planar64Vec3::ZERO;
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

	fn get_walk_target_velocity(&self,camera:&PhysicsCamera,controls:u32,next_mouse:&MouseState,time:Time,normal:&Planar64Vec3)->Planar64Vec3{
		let mut control_dir=self.get_control_dir(controls);
		if control_dir==Planar64Vec3::ZERO{
			return control_dir;
		}
		let camera_mat=camera.simulate_move_rotation_y(camera.mouse.lerp(&next_mouse,time).x);
		control_dir=camera_mat*control_dir;
		let n=normal.length();
		let m=control_dir.length();
		let d=normal.dot(control_dir)/m;
		if d<n{
			let cr=normal.cross(control_dir);
			if cr==Planar64Vec3::ZERO{
				Planar64Vec3::ZERO
			}else{
				cr.cross(*normal)*(self.walk_speed/(n*(n*n-d*d).sqrt()*m))
			}
		}else{
			Planar64Vec3::ZERO
		}
	}
	fn get_ladder_target_velocity(&self,camera:&PhysicsCamera,controls:u32,next_mouse:&MouseState,time:Time,normal:&Planar64Vec3)->Planar64Vec3{
		let mut control_dir=self.get_control_dir(controls);
		if control_dir==Planar64Vec3::ZERO{
			return control_dir;
		}
		let camera_mat=camera.simulate_move_rotation(camera.mouse.lerp(&next_mouse,time));
		control_dir=camera_mat*control_dir;
		let n=normal.length();
		let m=control_dir.length();
		let mut d=normal.dot(control_dir)/m;
		if d< -self.ladder_dot*n{
			control_dir=Planar64Vec3::Y*m;
			d=normal.y();
		}else if self.ladder_dot*n<d{
			control_dir=Planar64Vec3::NEG_Y*m;
			d=-normal.y();
		}
		//n=d if you are standing on top of a ladder and press E.
		//two fixes:
		//- ladder movement is not allowed on walkable surfaces
		//- fix the underlying issue
		if d.get().unsigned_abs()<n.get().unsigned_abs(){
			let cr=normal.cross(control_dir);
			if cr==Planar64Vec3::ZERO{
				Planar64Vec3::ZERO
			}else{
				cr.cross(*normal)*(self.ladder_speed/(n*(n*n-d*d).sqrt()))
			}
		}else{
			Planar64Vec3::ZERO
		}
	}
	fn get_propulsion_control_dir(&self,camera:&PhysicsCamera,controls:u32,next_mouse:&MouseState,time:Time)->Planar64Vec3{
		let camera_mat=camera.simulate_move_rotation(camera.mouse.lerp(&next_mouse,time));
		camera_mat*self.get_control_dir(controls)
	}
	#[inline]
	fn mesh(&self)->TransformedMesh{
		self.hitbox.transformed_mesh()
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
	models:PhysicsModels,
	bvh:crate::bvh::BvhNode,
	
	modes:Modes,
	//the spawn point is where you spawn when you load into the map.
	//This is not the same as Reset which teleports you to Spawn0
	spawn_point:Planar64Vec3,
}
#[derive(Clone,Default)]
pub struct PhysicsOutputState{
	body:Body,
	camera:PhysicsCamera,
	camera_offset:Planar64Vec3,
}
impl PhysicsOutputState{
	pub fn extrapolate(&self,mouse_pos:glam::IVec2,time:Time)->(glam::Vec3,glam::Vec2){
		((self.body.extrapolated_position(time)+self.camera_offset).into(),self.camera.simulate_move_angles(mouse_pos))
	}
}

#[derive(Clone,Hash,Eq,PartialEq)]
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
struct NonPhysicsError;
impl TryFrom<&crate::model::CollisionAttributes> for PhysicsCollisionAttributes{
	type Error=NonPhysicsError;
	fn try_from(value:&crate::model::CollisionAttributes)->Result<Self,Self::Error>{
		match value{
			crate::model::CollisionAttributes::Decoration=>Err(NonPhysicsError),
			crate::model::CollisionAttributes::Contact{contacting,general}=>Ok(Self::Contact{contacting:contacting.clone(),general:general.clone()}),
			crate::model::CollisionAttributes::Intersect{intersecting,general}=>Ok(Self::Intersect{intersecting:intersecting.clone(),general:general.clone()}),
		}
	}
}

pub struct PhysicsModel{
	//A model is a thing that has a hitbox. can be represented by a list of TreyMesh-es
	//in this iteration, all it needs is extents.
	mesh_id:usize,
	attr_id:usize,
	transform:crate::integer::Planar64Affine3,
	normal_transform:crate::integer::Planar64Mat3,
}

impl PhysicsModel{
	pub fn new(mesh_id:usize,attr_id:usize,transform:crate::integer::Planar64Affine3)->Self{
		let normal_transform=transform.matrix3.inverse().transpose();
		Self{
			mesh_id,
			attr_id,
			transform,
			normal_transform,
		}
	}
}

#[derive(Debug,Clone,Eq,Hash,PartialEq)]
struct ContactCollision{
	face_id:crate::model_physics::MinkowskiFace,
	model_id:usize,//using id to avoid lifetimes
}
#[derive(Debug,Clone,Eq,Hash,PartialEq)]
struct IntersectCollision{
	model_id:usize,
}
#[derive(Debug,Clone,Eq,Hash,PartialEq)]
enum Collision{
	Contact(ContactCollision),
	Intersect(IntersectCollision),
}
impl Collision{
	fn model_id(&self)->usize{
		match self{
			&Collision::Contact(ContactCollision{model_id,face_id:_})
			|&Collision::Intersect(IntersectCollision{model_id})=>model_id,
		}
	}
	fn face_id(&self)->Option<crate::model_physics::MinkowskiFace>{
		match self{
			&Collision::Contact(ContactCollision{model_id:_,face_id})=>Some(face_id),
			&Collision::Intersect(IntersectCollision{model_id:_})=>None,
		}
	}
}
#[derive(Default)]
struct TouchingState{
	contacts:std::collections::HashSet::<ContactCollision>,
	intersects:std::collections::HashSet::<IntersectCollision>,
}
impl TouchingState{
	fn clear(&mut self){
		self.contacts.clear();
		self.intersects.clear();
	}
	fn insert(&mut self,collision:Collision)->bool{
		match collision{
			Collision::Contact(collision)=>self.contacts.insert(collision),
			Collision::Intersect(collision)=>self.intersects.insert(collision),
		}
	}
	fn remove(&mut self,collision:&Collision)->bool{
		match collision{
			Collision::Contact(collision)=>self.contacts.remove(collision),
			Collision::Intersect(collision)=>self.intersects.remove(collision),
		}
	}
	fn base_acceleration(&self,models:&PhysicsModels,style:&StyleModifiers,camera:&PhysicsCamera,controls:u32,next_mouse:&MouseState,time:Time)->Planar64Vec3{
		let mut a=style.gravity;
		if let Some(rocket_force)=style.rocket_force{
			a+=style.get_propulsion_control_dir(camera,controls,next_mouse,time)*rocket_force;
		}
		//add accelerators
		for contact in &self.contacts{
			match models.attr(contact.model_id){
				PhysicsCollisionAttributes::Contact{contacting,general}=>{
					match &general.accelerator{
						Some(accelerator)=>a+=accelerator.acceleration,
						None=>(),
					}
				},
				_=>panic!("impossible touching state"),
			}
		}
		for intersect in &self.intersects{
			match models.attr(intersect.model_id){
				PhysicsCollisionAttributes::Intersect{intersecting,general}=>{
					match &general.accelerator{
						Some(accelerator)=>a+=accelerator.acceleration,
						None=>(),
					}
				},
				_=>panic!("impossible touching state"),
			}
		}
		//add water../?
		a
	}
	fn constrain_velocity(&self,models:&PhysicsModels,style_mesh:&TransformedMesh,velocity:&mut Planar64Vec3){
		//TODO: trey push solve
		for contact in &self.contacts{
			let n=contact_normal(models,style_mesh,contact);
			let d=n.dot128(*velocity);
			if d<0{
				*velocity-=n*Planar64::raw(((d<<32)/n.dot128(n)) as i64);
			}
		}
	}
	fn constrain_acceleration(&self,models:&PhysicsModels,style_mesh:&TransformedMesh,acceleration:&mut Planar64Vec3){
		//TODO: trey push solve
		for contact in &self.contacts{
			let n=contact_normal(models,style_mesh,contact);
			let d=n.dot128(*acceleration);
			if d<0{
				*acceleration-=n*Planar64::raw(((d<<32)/n.dot128(n)) as i64);
			}
		}
	}
	fn get_move_state(&self,body:&Body,models:&PhysicsModels,style:&StyleModifiers,camera:&PhysicsCamera,controls:u32,next_mouse:&MouseState,time:Time)->(MoveState,Planar64Vec3){
		//check current move conditions and use heuristics to determine
		//which ladder to climb on, which ground to walk on, etc
		//collect move state affecting objects from contacts (accelerator,water,ladder,ground)
		let style_mesh=style.mesh();
		let gravity=self.base_acceleration(models,style,camera,controls,next_mouse,time);
		let mut move_state=MoveState::Air;
		let mut a=gravity;
		for contact in &self.contacts{
			match models.attr(contact.model_id){
				PhysicsCollisionAttributes::Contact{contacting,general}=>{
					let normal=contact_normal(models,&style_mesh,contact);
					match &contacting.contact_behaviour{
						Some(crate::model::ContactingBehaviour::Ladder(_))=>{
							//ladder walkstate
							let mut target_velocity=style.get_ladder_target_velocity(camera,controls,next_mouse,time,&normal);
							self.constrain_velocity(models,&style_mesh,&mut target_velocity);
							let (walk_state,mut acceleration)=WalkState::ladder(body,style,gravity,target_velocity,contact.clone(),&normal);
							move_state=MoveState::Ladder(walk_state);
							self.constrain_acceleration(models,&style_mesh,&mut acceleration);
							a=acceleration;
						},
						None=>if style.surf_slope.map_or(true,|s|normal.walkable(s,Planar64Vec3::Y)){
							//check ground
							let mut target_velocity=style.get_walk_target_velocity(camera,controls,next_mouse,time,&normal);
							self.constrain_velocity(models,&style_mesh,&mut target_velocity);
							let (walk_state,mut acceleration)=WalkState::ground(body,style,gravity,target_velocity,contact.clone(),&normal);
							move_state=MoveState::Walk(walk_state);
							self.constrain_acceleration(models,&style_mesh,&mut acceleration);
							a=acceleration;
						},
						_=>(),
					}
				},
				_=>panic!("impossible touching state"),
			}
		}
		for intersect in &self.intersects{
			//water
		}
		self.constrain_acceleration(models,&style_mesh,&mut a);
		(move_state,a)
	}
	fn predict_collision_end(&self,collector:&mut crate::instruction::InstructionCollector<PhysicsInstruction>,models:&PhysicsModels,style_mesh:&TransformedMesh,body:&Body,time:Time){
		let relative_body=VirtualBody::relative(&Body::default(),body).body(time);
		for contact in &self.contacts{
			//detect face slide off
			let model_mesh=models.mesh(contact.model_id);
			let minkowski=crate::model_physics::MinkowskiMesh::minkowski_sum(&model_mesh,&style_mesh);
			collector.collect(minkowski.predict_collision_face_out(&relative_body,collector.time(),contact.face_id).map(|(face,time)|{
				TimedInstruction{
					time,
					instruction:PhysicsInstruction::CollisionEnd(
						Collision::Contact(ContactCollision{model_id:contact.model_id,face_id:contact.face_id})
					),
				}
			}));
		}
		for intersect in &self.intersects{
			//detect model collision in reverse
			let model_mesh=models.mesh(intersect.model_id);
			let minkowski=crate::model_physics::MinkowskiMesh::minkowski_sum(&model_mesh,&style_mesh);
			collector.collect(minkowski.predict_collision_out(&relative_body,collector.time()).map(|(face,time)|{
				TimedInstruction{
					time,
					instruction:PhysicsInstruction::CollisionEnd(
						Collision::Intersect(IntersectCollision{model_id:intersect.model_id})
					),
				}
			}));
		}
	}
}

impl Body{
	pub fn new(position:Planar64Vec3,velocity:Planar64Vec3,acceleration:Planar64Vec3,time:Time)->Self{
		Self{
			position,
			velocity,
			acceleration,
			time,
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
	pub fn infinity_dir(&self)->Option<Planar64Vec3>{
		if self.velocity==Planar64Vec3::ZERO{
			if self.acceleration==Planar64Vec3::ZERO{
				None
			}else{
				Some(self.acceleration)
			}
		}else{
			Some(self.velocity)
		}
	}
	pub fn grow_aabb(&self,aabb:&mut crate::aabb::Aabb,t0:Time,t1:Time){
		aabb.grow(self.extrapolated_position(t0));
		aabb.grow(self.extrapolated_position(t1));
		//v+a*t==0
		//goober code
		if self.acceleration.x()!=Planar64::ZERO{
			let t=Time::from(-self.velocity.x()/self.acceleration.x());
			if t0<t&&t<t1{
				aabb.grow(self.extrapolated_position(t));
			}
		}
		if self.acceleration.y()!=Planar64::ZERO{
			let t=Time::from(-self.velocity.y()/self.acceleration.y());
			if t0<t&&t<t1{
				aabb.grow(self.extrapolated_position(t));
			}
		}
		if self.acceleration.z()!=Planar64::ZERO{
			let t=Time::from(-self.velocity.z()/self.acceleration.z());
			if t0<t&&t<t1{
				aabb.grow(self.extrapolated_position(t));
			}
		}
	}

}
impl std::fmt::Display for Body{
	fn fmt(&self,f:&mut std::fmt::Formatter<'_>)->std::fmt::Result{
		write!(f,"p({}) v({}) a({}) t({})",self.position,self.velocity,self.acceleration,self.time)
	}
}

struct VirtualBody<'a>{
	body0:&'a Body,
	body1:&'a Body,
}
impl VirtualBody<'_>{
	fn relative<'a>(body0:&'a Body,body1:&'a Body)->VirtualBody<'a>{
		//(p0,v0,a0,t0)
		//(p1,v1,a1,t1)
		VirtualBody{
			body0,
			body1,
		}
	}
	fn extrapolated_position(&self,time:Time)->Planar64Vec3{
		self.body1.extrapolated_position(time)-self.body0.extrapolated_position(time)
	}
	fn extrapolated_velocity(&self,time:Time)->Planar64Vec3{
		self.body1.extrapolated_velocity(time)-self.body0.extrapolated_velocity(time)
	}
	fn acceleration(&self)->Planar64Vec3{
		self.body1.acceleration-self.body0.acceleration
	}
	fn body(&self,time:Time)->Body{
		Body::new(self.extrapolated_position(time),self.extrapolated_velocity(time),self.acceleration(),time)
	}
}

impl Default for PhysicsState{
	fn default()->Self{
 		Self{
			spawn_point:Planar64Vec3::int(0,50,0),
			body:Body::new(Planar64Vec3::int(0,50,0),Planar64Vec3::int(0,0,0),Planar64Vec3::int(0,-100,0),Time::ZERO),
			time:Time::ZERO,
			style:StyleModifiers::default(),
			touching:TouchingState::default(),
			models:PhysicsModels::default(),
			bvh:crate::bvh::BvhNode::default(),
			move_state: MoveState::Air,
			camera:PhysicsCamera::default(),
			next_mouse:MouseState::default(),
			controls:0,
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
		self.bvh=crate::bvh::BvhNode::default();
	}

	pub fn output(&self)->PhysicsOutputState{
		PhysicsOutputState{
			body:self.body.clone(),
			camera:self.camera.clone(),
			camera_offset:self.style.camera_offset.clone(),
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
		let mut attr_hash=std::collections::HashMap::new();
		for model in &indexed_models.models{
			let mesh_id=self.models.meshes.len();
			let mut make_mesh=false;
			for model_instance in &model.instances{
				if let Ok(physics_attributes)=PhysicsCollisionAttributes::try_from(&model_instance.attributes){
					let attr_id=if let Some(&attr_id)=attr_hash.get(&physics_attributes){
						attr_id
					}else{
						let attr_id=self.models.push_attr(physics_attributes.clone());
						attr_hash.insert(physics_attributes,attr_id);
						attr_id
					};
					let model_physics=PhysicsModel::new(mesh_id,attr_id,model_instance.transform);
					make_mesh=true;
					let model_id=self.models.push_model(model_physics);
					for attr in &model_instance.temp_indexing{
						match attr{
							crate::model::TempIndexedAttributes::Start(s)=>starts.push((model_id,s.clone())),
							crate::model::TempIndexedAttributes::Spawn(s)=>spawns.push((model_id,s.clone())),
							crate::model::TempIndexedAttributes::Wormhole(s)=>{self.models.model_id_from_wormhole_id.insert(s.wormhole_id,model_id);},
						}
					}
				}
			}
			if make_mesh{
				self.models.push_mesh(PhysicsMesh::from(model));
			}
		}
		self.bvh=crate::bvh::generate_bvh(self.models.aabb_list());
		//I don't wanna write structs for temporary structures
		//this code builds ModeDescriptions from the unsorted lists at the top of the function
		starts.sort_by_key(|tup|tup.1.mode_id);
		let mut mode_id_from_map_mode_id=std::collections::HashMap::new();
		let mut modedatas:Vec<(usize,Vec<(u32,usize)>,u32)>=starts.into_iter().enumerate().map(|(i,(model_id,s))|{
			mode_id_from_map_mode_id.insert(s.mode_id,i);
			(model_id,Vec::new(),s.mode_id)
		}).collect();
		for (model_id,s) in spawns{
			if let Some(mode_id)=mode_id_from_map_mode_id.get(&s.mode_id){
				if let Some(modedata)=modedatas.get_mut(*mode_id){
					modedata.1.push((s.stage_id,model_id));
				}
			}
		}
		for mut tup in modedatas.into_iter(){
			tup.1.sort_by_key(|tup|tup.0);
			let mut eshmep1=std::collections::HashMap::new();
			let mut eshmep2=std::collections::HashMap::new();
			self.modes.insert(tup.2,crate::model::ModeDescription{
				start:tup.0,
				spawns:tup.1.into_iter().enumerate().map(|(i,tup)|{eshmep1.insert(tup.0,i);tup.1}).collect(),
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

	fn next_strafe_instruction(&self)->Option<TimedInstruction<PhysicsInstruction>>{
		self.style.strafe.as_ref().map(|strafe|{
			TimedInstruction{
				time:Time::from_nanos(strafe.tick_rate.rhs_div_int(strafe.tick_rate.mul_int(self.time.nanos())+1)),
				//only poll the physics if there is a before and after mouse event
				instruction:PhysicsInstruction::StrafeTick
			}
		})
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

	fn refresh_walk_target(&mut self)->Planar64Vec3{
		match &mut self.move_state{
			MoveState::Air|MoveState::Water=>self.touching.base_acceleration(&self.models,&self.style,&self.camera,self.controls,&self.next_mouse,self.time),
			MoveState::Walk(WalkState{state,contact,jump_direction:_})=>{
				let style_mesh=self.style.mesh();
				let n=contact_normal(&self.models,&style_mesh,contact);
				let gravity=self.touching.base_acceleration(&self.models,&self.style,&self.camera,self.controls,&self.next_mouse,self.time);
				let mut a;
				let mut v=self.style.get_walk_target_velocity(&self.camera,self.controls,&self.next_mouse,self.time,&n);
				self.touching.constrain_velocity(&self.models,&style_mesh,&mut v);
				let normal_accel=-n.dot(gravity)/n.length();
				(*state,a)=WalkEnum::with_target_velocity(&self.body,&self.style,v,&n,self.style.walk_speed,normal_accel);
				a
			},
			MoveState::Ladder(WalkState{state,contact,jump_direction:_})=>{
				let style_mesh=self.style.mesh();
				let n=contact_normal(&self.models,&style_mesh,contact);
				let gravity=self.touching.base_acceleration(&self.models,&self.style,&self.camera,self.controls,&self.next_mouse,self.time);
				let mut a;
				let mut v=self.style.get_ladder_target_velocity(&self.camera,self.controls,&self.next_mouse,self.time,&n);
				self.touching.constrain_velocity(&self.models,&style_mesh,&mut v);
				(*state,a)=WalkEnum::with_target_velocity(&self.body,&self.style,v,&n,self.style.ladder_speed,self.style.ladder_accel);
				a
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
}

impl crate::instruction::InstructionEmitter<PhysicsInstruction> for PhysicsState{
	//this little next instruction function can cache its return value and invalidate the cached value by watching the State.
	fn next_instruction(&self,time_limit:Time)->Option<TimedInstruction<PhysicsInstruction>>{
		//JUST POLLING!!! NO MUTATION
		let mut collector = crate::instruction::InstructionCollector::new(time_limit);

		collector.collect(self.next_move_instruction());

		let style_mesh=self.style.mesh();
		//check for collision ends
		self.touching.predict_collision_end(&mut collector,&self.models,&style_mesh,&self.body,self.time);
		//check for collision starts
		let mut aabb=crate::aabb::Aabb::default();
		self.body.grow_aabb(&mut aabb,self.time,collector.time());
		aabb.inflate(self.style.hitbox.halfsize);
		//common body
		let relative_body=VirtualBody::relative(&Body::default(),&self.body).body(self.time);
		self.bvh.the_tester(&aabb,&mut |id|{
			//no checks are needed because of the time limits.
			let model_mesh=self.models.mesh(id);
			let minkowski=crate::model_physics::MinkowskiMesh::minkowski_sum(&model_mesh,&style_mesh);
			collector.collect(minkowski.predict_collision_in(&relative_body,collector.time())
				//temp (?) code to avoid collision loops
				.map_or(None,|(face,time)|if time==self.time{None}else{Some((face,time))})
				.map(|(face,time)|{
				TimedInstruction{time,instruction:PhysicsInstruction::CollisionStart(match self.models.attr(id){
					PhysicsCollisionAttributes::Contact{contacting:_,general:_}=>Collision::Contact(ContactCollision{model_id:id,face_id:face}),
					PhysicsCollisionAttributes::Intersect{intersecting:_,general:_}=>Collision::Intersect(IntersectCollision{model_id:id}),
				})}
			}));
		});
		collector.instruction()
	}
}

fn get_walk_state(move_state:&MoveState)->Option<&WalkState>{
	match move_state{
		MoveState::Walk(walk_state)|MoveState::Ladder(walk_state)=>Some(walk_state),
		MoveState::Air|MoveState::Water=>None,
	}
}

fn jumped_velocity(models:&PhysicsModels,style:&StyleModifiers,walk_state:&WalkState,v:&mut Planar64Vec3){
	let jump_dir=match &walk_state.jump_direction{
		JumpDirection::FromContactNormal=>contact_normal(models,&style.mesh(),&walk_state.contact),
		&JumpDirection::Exactly(dir)=>dir,
	};
	*v=*v+jump_dir*(style.get_jump_deltav()/jump_dir.length());
}

fn contact_normal(models:&PhysicsModels,style_mesh:&TransformedMesh,contact:&ContactCollision)->Planar64Vec3{
	let model_mesh=models.mesh(contact.model_id);
	let minkowski=crate::model_physics::MinkowskiMesh::minkowski_sum(&model_mesh,style_mesh);
	minkowski.face_nd(contact.face_id).0
}

fn set_position(body:&mut Body,touching:&mut TouchingState,point:Planar64Vec3)->Planar64Vec3{
	//test intersections at new position
	//hovering above the surface 0 units is not intersecting.  you will fall into it just fine
	body.position=point;
	//manual clear //for c in contacts{process_instruction(CollisionEnd(c))}
	touching.clear();
	//TODO: calculate contacts and determine the actual state
	//touching.recalculate(body);
	point
}
fn set_velocity_cull(body:&mut Body,touching:&mut TouchingState,models:&PhysicsModels,style_mesh:&TransformedMesh,v:Planar64Vec3)->bool{
	//This is not correct but is better than what I have
	let mut culled=false;
	touching.contacts.retain(|contact|{
		let n=contact_normal(models,style_mesh,contact);
		let r=n.dot(v)<=Planar64::ZERO;
		if !r{
			culled=true;
			println!("set_velocity_cull contact={:?}",contact);
		}
		r
	});
	set_velocity(body,touching,models,style_mesh,v);
	culled
}
fn set_velocity(body:&mut Body,touching:&TouchingState,models:&PhysicsModels,style_mesh:&TransformedMesh,mut v:Planar64Vec3)->Planar64Vec3{
	touching.constrain_velocity(models,style_mesh,&mut v);
	body.velocity=v;
	v
}
fn set_acceleration_cull(body:&mut Body,touching:&mut TouchingState,models:&PhysicsModels,style_mesh:&TransformedMesh,a:Planar64Vec3)->bool{
	//This is not correct but is better than what I have
	let mut culled=false;
	touching.contacts.retain(|contact|{
		let n=contact_normal(models,style_mesh,contact);
		let r=n.dot(a)<=Planar64::ZERO;
		if !r{
			culled=true;
			println!("set_acceleration_cull contact={:?}",contact);
		}
		r
	});
	set_acceleration(body,touching,models,style_mesh,a);
	culled
}
fn set_acceleration(body:&mut Body,touching:&TouchingState,models:&PhysicsModels,style_mesh:&TransformedMesh,mut a:Planar64Vec3)->Planar64Vec3{
	touching.constrain_acceleration(models,style_mesh,&mut a);
	body.acceleration=a;
	a
}

fn teleport(body:&mut Body,touching:&mut TouchingState,models:&PhysicsModels,style:&StyleModifiers,point:Planar64Vec3)->MoveState{
	set_position(body,touching,point);
	set_acceleration(body,touching,models,&style.mesh(),style.gravity);
	MoveState::Air
}
fn teleport_to_spawn(body:&mut Body,touching:&mut TouchingState,style:&StyleModifiers,mode:&crate::model::ModeDescription,models:&PhysicsModels,stage_id:u32)->Option<MoveState>{
	let model=models.model(*mode.get_spawn_model_id(stage_id)? as usize);
	let point=model.transform.transform_point3(Planar64Vec3::Y)+Planar64Vec3::Y*(style.hitbox.halfsize.y()+Planar64::ONE/16);
	Some(teleport(body,touching,models,style,point))
}

fn run_teleport_behaviour(teleport_behaviour:&Option<crate::model::TeleportBehaviour>,game:&mut GameMechanicsState,models:&PhysicsModels,modes:&Modes,style:&StyleModifiers,touching:&mut TouchingState,body:&mut Body,model_id:usize)->Option<MoveState>{
	//TODO: jump count and checkpoints are always reset on teleport.
	//Map makers are expected to use tools to prevent
	//multi-boosting on JumpLimit boosters such as spawning into a SetVelocity
	match teleport_behaviour{
		Some(crate::model::TeleportBehaviour::StageElement(stage_element))=>{
			if stage_element.force||game.stage_id<stage_element.stage_id{
				//TODO: check if all checkpoints are complete up to destination stage id, otherwise set to checkpoint completion stage it
				game.stage_id=stage_element.stage_id;
			}
			match &stage_element.behaviour{
				crate::model::StageElementBehaviour::SpawnAt=>None,
				crate::model::StageElementBehaviour::Trigger
				|crate::model::StageElementBehaviour::Teleport=>{
					//I guess this is correct behaviour when trying to teleport to a non-existent spawn but it's still weird
					teleport_to_spawn(body,touching,style,modes.get_mode(stage_element.mode_id)?,models,game.stage_id)
				},
				crate::model::StageElementBehaviour::Platform=>None,
				&crate::model::StageElementBehaviour::Checkpoint=>{
					// let mode=modes.get_mode(stage_element.mode_id)?;
					// if mode.ordered_checkpoint_id.map_or(true,|id|id<game.next_ordered_checkpoint_id)
					// 	&&mode.unordered_checkpoint_count<=game.unordered_checkpoints.len() as u32{
					// 	//pass
					 	None
					// }else{
					// 	//fail
					// 	teleport_to_spawn(body,touching,style,modes.get_mode(stage_element.mode_id)?,models,game.stage_id)
					// }
				},
				&crate::model::StageElementBehaviour::Ordered{checkpoint_id}=>{
					if checkpoint_id<game.next_ordered_checkpoint_id{
						//if you hit a checkpoint you already hit, nothing happens
						None
					}else if game.next_ordered_checkpoint_id==checkpoint_id{
						//if you hit the next number in a sequence of ordered checkpoints
						//increment the current checkpoint id
						game.next_ordered_checkpoint_id+=1;
						None
					}else{
						//If you hit an ordered checkpoint after missing a previous one
						teleport_to_spawn(body,touching,style,modes.get_mode(stage_element.mode_id)?,models,game.stage_id)
					}
				},
				crate::model::StageElementBehaviour::Unordered=>{
					//count model id in accumulated unordered checkpoints
					game.unordered_checkpoints.insert(model_id);
					None
				},
				&crate::model::StageElementBehaviour::JumpLimit(jump_limit)=>{
					//let count=game.jump_counts.get(&model.id);
					//TODO
					None
				},
			}
		},
		Some(crate::model::TeleportBehaviour::Wormhole(wormhole))=>{
			let origin_model=models.model(model_id);
			let destination_model=models.get_wormhole_model(wormhole.destination_model_id)?;
			//ignore the transform for now
			Some(teleport(body,touching,models,style,body.position-origin_model.transform.translation+destination_model.transform.translation))
		}
		None=>None,
	}
}

impl crate::instruction::InstructionConsumer<PhysicsInstruction> for PhysicsState {
	fn process_instruction(&mut self, ins:TimedInstruction<PhysicsInstruction>) {
		match &ins.instruction{
			PhysicsInstruction::Input(PhysicsInputInstruction::Idle)
			|PhysicsInstruction::Input(PhysicsInputInstruction::SetNextMouse(_))
			|PhysicsInstruction::Input(PhysicsInputInstruction::ReplaceMouse(_,_))
			|PhysicsInstruction::StrafeTick=>(),
			_=>println!("{}|{:?}",ins.time,ins.instruction),
		}
		//selectively update body
		match &ins.instruction{
			PhysicsInstruction::Input(PhysicsInputInstruction::Idle)=>self.time=ins.time,//idle simply updates time
			PhysicsInstruction::Input(_)
			|PhysicsInstruction::ReachWalkTargetVelocity
			|PhysicsInstruction::CollisionStart(_)
			|PhysicsInstruction::CollisionEnd(_)
			|PhysicsInstruction::StrafeTick=>self.advance_time(ins.time),
		}
		match ins.instruction{
			PhysicsInstruction::CollisionStart(c)=>{
				let style_mesh=self.style.mesh();
				let model_id=c.model_id();
				match (self.models.attr(model_id),&c){
					(PhysicsCollisionAttributes::Contact{contacting,general},Collision::Contact(contact))=>{
						let mut v=self.body.velocity;
						let normal=contact_normal(&self.models,&style_mesh,contact);
						match &contacting.contact_behaviour{
							Some(crate::model::ContactingBehaviour::Surf)=>println!("I'm surfing!"),
							Some(crate::model::ContactingBehaviour::Cling)=>println!("Unimplemented!"),
							&Some(crate::model::ContactingBehaviour::Elastic(elasticity))=>{
								//velocity and normal are facing opposite directions so this is inherently negative.
								let d=normal.dot(v)*(Planar64::ONE+Planar64::raw(elasticity as i64+1));
								v+=normal*(d/normal.dot(normal));
							},
							Some(crate::model::ContactingBehaviour::Ladder(contacting_ladder))=>{
								if contacting_ladder.sticky{
									//kill v
									//actually you could do this with a booster attribute :thinking:
									v=Planar64Vec3::ZERO;//model.velocity
								}
								//ladder walkstate
								let gravity=self.touching.base_acceleration(&self.models,&self.style,&self.camera,self.controls,&self.next_mouse,self.time);
								let mut target_velocity=self.style.get_ladder_target_velocity(&self.camera,self.controls,&self.next_mouse,self.time,&normal);
								self.touching.constrain_velocity(&self.models,&style_mesh,&mut target_velocity);
								let (walk_state,a)=WalkState::ladder(&self.body,&self.style,gravity,target_velocity,contact.clone(),&normal);
								self.move_state=MoveState::Ladder(walk_state);
								set_acceleration(&mut self.body,&self.touching,&self.models,&style_mesh,a);
							}
							None=>if self.style.surf_slope.map_or(true,|s|contact_normal(&self.models,&style_mesh,contact).walkable(s,Planar64Vec3::Y)){
								//ground
								let gravity=self.touching.base_acceleration(&self.models,&self.style,&self.camera,self.controls,&self.next_mouse,self.time);
								let mut target_velocity=self.style.get_walk_target_velocity(&self.camera,self.controls,&self.next_mouse,self.time,&normal);
								self.touching.constrain_velocity(&self.models,&style_mesh,&mut target_velocity);
								let (walk_state,a)=WalkState::ground(&self.body,&self.style,gravity,target_velocity,contact.clone(),&normal);
								self.move_state=MoveState::Walk(walk_state);
								set_acceleration(&mut self.body,&self.touching,&self.models,&style_mesh,a);
							},
						}
						//check ground
						self.touching.insert(c);
						//I love making functions with 10 arguments to dodge the borrow checker
						run_teleport_behaviour(&general.teleport_behaviour,&mut self.game,&self.models,&self.modes,&self.style,&mut self.touching,&mut self.body,model_id);
						//flatten v
						self.touching.constrain_velocity(&self.models,&style_mesh,&mut v);
						match &general.booster{
							Some(booster)=>{
								//DELETE THIS when boosters get converted to height machines
								match booster{
									&crate::model::GameMechanicBooster::Affine(transform)=>v=transform.transform_point3(v),
									&crate::model::GameMechanicBooster::Velocity(velocity)=>v+=velocity,
									&crate::model::GameMechanicBooster::Energy{direction: _,energy: _}=>todo!(),
								}
							},
							None=>(),
						}
						let calc_move=if self.style.get_control(StyleModifiers::CONTROL_JUMP,self.controls){
							if let Some(walk_state)=get_walk_state(&self.move_state){
								jumped_velocity(&self.models,&self.style,walk_state,&mut v);
								set_velocity_cull(&mut self.body,&mut self.touching,&self.models,&style_mesh,v)
							}else{false}
						}else{false};
						match &general.trajectory{
							Some(trajectory)=>{
								match trajectory{
									crate::model::GameMechanicSetTrajectory::AirTime(_) => todo!(),
									crate::model::GameMechanicSetTrajectory::Height(_) => todo!(),
									crate::model::GameMechanicSetTrajectory::TargetPointTime { target_point: _, time: _ } => todo!(),
									crate::model::GameMechanicSetTrajectory::TargetPointSpeed { target_point: _, speed: _, trajectory_choice: _ } => todo!(),
									&crate::model::GameMechanicSetTrajectory::Velocity(velocity)=>v=velocity,
									crate::model::GameMechanicSetTrajectory::DotVelocity { direction: _, dot: _ } => todo!(),
								}
							},
							None=>(),
						}
						set_velocity(&mut self.body,&self.touching,&self.models,&style_mesh,v);
						//not sure if or is correct here
						if calc_move||Planar64::ZERO<normal.dot(v){
							(self.move_state,self.body.acceleration)=self.touching.get_move_state(&self.body,&self.models,&self.style,&self.camera,self.controls,&self.next_mouse,self.time);
						}
						let a=self.refresh_walk_target();
						set_acceleration(&mut self.body,&self.touching,&self.models,&self.style.mesh(),a);
					},
					(PhysicsCollisionAttributes::Intersect{intersecting: _,general},Collision::Intersect(intersect))=>{
						//I think that setting the velocity to 0 was preventing surface contacts from entering an infinite loop
						self.touching.insert(c);
						run_teleport_behaviour(&general.teleport_behaviour,&mut self.game,&self.models,&self.modes,&self.style,&mut self.touching,&mut self.body,model_id);
					},
					_=>panic!("invalid pair"),
				}
			},
			PhysicsInstruction::CollisionEnd(c) => {
				match self.models.attr(c.model_id()){
					PhysicsCollisionAttributes::Contact{contacting:_,general:_}=>{
						self.touching.remove(&c);//remove contact before calling contact_constrain_acceleration
						//check ground
						(self.move_state,self.body.acceleration)=self.touching.get_move_state(&self.body,&self.models,&self.style,&self.camera,self.controls,&self.next_mouse,self.time);
					},
					PhysicsCollisionAttributes::Intersect{intersecting:_,general:_}=>{
						self.touching.remove(&c);
					},
				}
			},
			PhysicsInstruction::StrafeTick => {
				let control_dir=self.style.get_control_dir(self.controls);
				if control_dir!=Planar64Vec3::ZERO{
					let camera_mat=self.camera.simulate_move_rotation_y(self.camera.mouse.lerp(&self.next_mouse,self.time).x);
					let control_dir=camera_mat*control_dir;
					//normalize but careful for zero
					let d=self.body.velocity.dot(control_dir);
					if d<self.style.mv {
						let v=self.body.velocity+control_dir*(self.style.mv-d);
						//this is wrong but will work ig
						//need to note which push planes activate in push solve and keep those
						if set_velocity_cull(&mut self.body,&mut self.touching,&self.models,&self.style.mesh(),v){
							(self.move_state,self.body.acceleration)=self.touching.get_move_state(&self.body,&self.models,&self.style,&self.camera,self.controls,&self.next_mouse,self.time);
						}
					}
				}
			}
			PhysicsInstruction::ReachWalkTargetVelocity => {
				match &mut self.move_state{
					MoveState::Air|MoveState::Water=>(),
					MoveState::Walk(walk_state)|MoveState::Ladder(walk_state)=>{
						match &mut walk_state.state{
							WalkEnum::Reached=>(),
							WalkEnum::Transient(walk_target)=>{
								let style_mesh=self.style.mesh();
								//precisely set velocity
								let a=Planar64Vec3::ZERO;//ignore gravity for now.
								set_acceleration(&mut self.body,&self.touching,&self.models,&style_mesh,a);
								let v=walk_target.velocity;
								set_velocity(&mut self.body,&self.touching,&self.models,&style_mesh,v);
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
						if let Some(walk_state)=get_walk_state(&self.move_state){
							let mut v=self.body.velocity;
							jumped_velocity(&self.models,&self.style,walk_state,&mut v);
							if set_velocity_cull(&mut self.body,&mut self.touching,&self.models,&self.style.mesh(),v){
								(self.move_state,self.body.acceleration)=self.touching.get_move_state(&self.body,&self.models,&self.style,&self.camera,self.controls,&self.next_mouse,self.time);
							}
						}
						refresh_walk_target=false;
					},
					PhysicsInputInstruction::SetZoom(s) => {
						self.set_control(StyleModifiers::CONTROL_ZOOM,s);
						refresh_walk_target=false;
					},
					PhysicsInputInstruction::Reset => {
						//it matters which of these runs first, but I have not thought it through yet as it doesn't matter yet
						set_position(&mut self.body,&mut self.touching,self.spawn_point);
						set_velocity(&mut self.body,&self.touching,&self.models,&self.style.mesh(),Planar64Vec3::ZERO);
						(self.move_state,self.body.acceleration)=self.touching.get_move_state(&self.body,&self.models,&self.style,&self.camera,self.controls,&self.next_mouse,self.time);
						refresh_walk_target=false;
					},
					PhysicsInputInstruction::Idle => {refresh_walk_target=false;},//literally idle!
				}
				if refresh_walk_target{
					let a=self.refresh_walk_target();
					if set_acceleration_cull(&mut self.body,&mut self.touching,&self.models,&self.style.mesh(),a){
						(self.move_state,self.body.acceleration)=self.touching.get_move_state(&self.body,&self.models,&self.style,&self.camera,self.controls,&self.next_mouse,self.time);
					}
				}
			},
		}
	}
}

#[allow(dead_code)]
fn test_collision(relative_body:Body,expected_collision_time:Option<Time>){
	let h0=Hitbox::from_mesh_scale(PhysicsMesh::from(&crate::primitives::unit_cylinder()),Planar64Vec3::int(5,1,5)/2);
	let h1=Hitbox::roblox();
	let hitbox_mesh=h1.transformed_mesh();
	let platform_mesh=h0.transformed_mesh();
	let minkowski=crate::model_physics::MinkowskiMesh::minkowski_sum(&platform_mesh,&hitbox_mesh);
	let collision=minkowski.predict_collision_in(&relative_body,Time::MAX);
	assert_eq!(collision.map(|tup|tup.1),expected_collision_time,"Incorrect time of collision");
}
#[test]
fn test_collision_degenerate(){
	test_collision(Body::new(
		Planar64Vec3::int(0,5,0),
		Planar64Vec3::int(0,-1,0),
		Planar64Vec3::ZERO,
		Time::ZERO
	),Some(Time::from_secs(2)));
}
#[test]
fn test_collision_degenerate_east(){
	test_collision(Body::new(
		Planar64Vec3::int(3,5,0),
		Planar64Vec3::int(0,-1,0),
		Planar64Vec3::ZERO,
		Time::ZERO
	),Some(Time::from_secs(2)));
}
#[test]
fn test_collision_degenerate_south(){
	test_collision(Body::new(
		Planar64Vec3::int(0,5,3),
		Planar64Vec3::int(0,-1,0),
		Planar64Vec3::ZERO,
		Time::ZERO
	),Some(Time::from_secs(2)));
}
#[test]
fn test_collision_degenerate_west(){
	test_collision(Body::new(
		Planar64Vec3::int(-3,5,0),
		Planar64Vec3::int(0,-1,0),
		Planar64Vec3::ZERO,
		Time::ZERO
	),Some(Time::from_secs(2)));
}
#[test]
fn test_collision_degenerate_north(){
	test_collision(Body::new(
		Planar64Vec3::int(0,5,-3),
		Planar64Vec3::int(0,-1,0),
		Planar64Vec3::ZERO,
		Time::ZERO
	),Some(Time::from_secs(2)));
}
#[test]
fn test_collision_parabola_edge_east_from_west(){
	test_collision(VirtualBody::relative(&Body::default(),&Body::new(
		Planar64Vec3::int(3,3,0),
		Planar64Vec3::int(100,-1,0),
		Planar64Vec3::int(0,-1,0),
		Time::ZERO
	)).body(Time::from_secs(-1)),Some(Time::from_secs(0)));
}
#[test]
fn test_collision_parabola_edge_south_from_north(){
	test_collision(VirtualBody::relative(&Body::default(),&Body::new(
		Planar64Vec3::int(0,3,3),
		Planar64Vec3::int(0,-1,100),
		Planar64Vec3::int(0,-1,0),
		Time::ZERO
	)).body(Time::from_secs(-1)),Some(Time::from_secs(0)));
}
#[test]
fn test_collision_parabola_edge_west_from_east(){
	test_collision(VirtualBody::relative(&Body::default(),&Body::new(
		Planar64Vec3::int(-3,3,0),
		Planar64Vec3::int(-100,-1,0),
		Planar64Vec3::int(0,-1,0),
		Time::ZERO
	)).body(Time::from_secs(-1)),Some(Time::from_secs(0)));
}
#[test]
fn test_collision_parabola_edge_north_from_south(){
	test_collision(VirtualBody::relative(&Body::default(),&Body::new(
		Planar64Vec3::int(0,3,-3),
		Planar64Vec3::int(0,-1,-100),
		Planar64Vec3::int(0,-1,0),
		Time::ZERO
	)).body(Time::from_secs(-1)),Some(Time::from_secs(0)));
}
#[test]
fn test_collision_parabola_edge_north_from_ne(){
	test_collision(VirtualBody::relative(&Body::default(),&Body::new(
		Planar64Vec3::int(0,6,-7)/2,
		Planar64Vec3::int(-10,-1,1),
		Planar64Vec3::int(0,-1,0),
		Time::ZERO
	)).body(Time::from_secs(-1)),Some(Time::from_secs(0)));
}
#[test]
fn test_collision_parabola_edge_north_from_nw(){
	test_collision(VirtualBody::relative(&Body::default(),&Body::new(
		Planar64Vec3::int(0,6,-7)/2,
		Planar64Vec3::int(10,-1,1),
		Planar64Vec3::int(0,-1,0),
		Time::ZERO
	)).body(Time::from_secs(-1)),Some(Time::from_secs(0)));
}
#[test]
fn test_collision_parabola_edge_east_from_se(){
	test_collision(VirtualBody::relative(&Body::default(),&Body::new(
		Planar64Vec3::int(7,6,0)/2,
		Planar64Vec3::int(-1,-1,-10),
		Planar64Vec3::int(0,-1,0),
		Time::ZERO
	)).body(Time::from_secs(-1)),Some(Time::from_secs(0)));
}
#[test]
fn test_collision_parabola_edge_east_from_ne(){
	test_collision(VirtualBody::relative(&Body::default(),&Body::new(
		Planar64Vec3::int(7,6,0)/2,
		Planar64Vec3::int(-1,-1,10),
		Planar64Vec3::int(0,-1,0),
		Time::ZERO
	)).body(Time::from_secs(-1)),Some(Time::from_secs(0)));
}
#[test]
fn test_collision_parabola_edge_south_from_se(){
	test_collision(VirtualBody::relative(&Body::default(),&Body::new(
		Planar64Vec3::int(0,6,7)/2,
		Planar64Vec3::int(-10,-1,-1),
		Planar64Vec3::int(0,-1,0),
		Time::ZERO
	)).body(Time::from_secs(-1)),Some(Time::from_secs(0)));
}
#[test]
fn test_collision_parabola_edge_south_from_sw(){
	test_collision(VirtualBody::relative(&Body::default(),&Body::new(
		Planar64Vec3::int(0,6,7)/2,
		Planar64Vec3::int(10,-1,-1),
		Planar64Vec3::int(0,-1,0),
		Time::ZERO
	)).body(Time::from_secs(-1)),Some(Time::from_secs(0)));
}
#[test]
fn test_collision_parabola_edge_west_from_se(){
	test_collision(VirtualBody::relative(&Body::default(),&Body::new(
		Planar64Vec3::int(-7,6,0)/2,
		Planar64Vec3::int(1,-1,-10),
		Planar64Vec3::int(0,-1,0),
		Time::ZERO
	)).body(Time::from_secs(-1)),Some(Time::from_secs(0)));
}
#[test]
fn test_collision_parabola_edge_west_from_ne(){
	test_collision(VirtualBody::relative(&Body::default(),&Body::new(
		Planar64Vec3::int(-7,6,0)/2,
		Planar64Vec3::int(1,-1,10),
		Planar64Vec3::int(0,-1,0),
		Time::ZERO
	)).body(Time::from_secs(-1)),Some(Time::from_secs(0)));
}
#[test]
fn test_collision_oblique(){
	test_collision(Body::new(
		Planar64Vec3::int(0,5,0),
		Planar64Vec3::int(1,-64,2)/64,
		Planar64Vec3::ZERO,
		Time::ZERO
	),Some(Time::from_secs(2)));
}
#[test]
fn zoom_hit_nothing(){
	test_collision(Body::new(
		Planar64Vec3::int(0,10,0),
		Planar64Vec3::int(1,0,0),
		Planar64Vec3::int(0,1,0),
		Time::ZERO
	),None);
}
#[test]
fn already_inside_hit_nothing(){
	test_collision(Body::new(
		Planar64Vec3::ZERO,
		Planar64Vec3::int(1,0,0),
		Planar64Vec3::int(0,1,0),
		Time::ZERO
	),None);
}
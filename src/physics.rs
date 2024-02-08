use std::collections::HashMap;
use std::collections::HashSet;

use crate::model_physics::{self,PhysicsMesh,PhysicsMeshTransform,TransformedMesh,MeshQuery,PhysicsMeshId,PhysicsSubmeshId};
use strafesnet_common::bvh;
use strafesnet_common::map;
use strafesnet_common::aabb;
use strafesnet_common::gameplay_modes;
use strafesnet_common::gameplay_attributes;
use strafesnet_common::model::ModelId;
use strafesnet_common::gameplay_style::{self,StyleModifiers};
use strafesnet_common::instruction::{self,InstructionEmitter,InstructionConsumer,TimedInstruction};
use strafesnet_common::integer::{self,Time,Planar64,Planar64Vec3,Planar64Mat3,Angle32,Ratio64Vec2};

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

#[derive(Default)]
struct PhysicsModels{
	meshes:HashMap<PhysicsMeshId,PhysicsMesh>,
	models:HashMap<PhysicsModelId,PhysicsModel>,
	//separate models into Contacting and Intersecting?
	//wrap model id with ContactingModelId and IntersectingModelId
	//attributes can be split into contacting and intersecting (this also saves a bit of memory)
	//can go even further and deduplicate General attributes separately, reconstructing it when queried
	attributes:HashMap<PhysicsAttributesId,PhysicsCollisionAttributes>,
}
impl PhysicsModels{
	fn clear(&mut self){
		self.meshes.clear();
		self.models.clear();
		self.attributes.clear();
	}
	//TODO: "statically" verify the maps don't refer to any nonexistant data, if they do delete the references.
	//then I can make these getter functions unchecked.
	fn mesh(&self,convex_mesh_id:ConvexMeshId)->TransformedMesh{
		let model_idx=convex_mesh_id.model_id.get() as usize;
		TransformedMesh::new(
			self.meshes[model_idx].submesh_view(convex_mesh_id.submesh_id),
			&self.models[model_idx].transform
		)
	}
	fn model(&self,model_id:PhysicsModelId)->&PhysicsModel{
		&self.models[model_id.get() as usize]
	}
	fn attr(&self,model_id:PhysicsModelId)->&PhysicsCollisionAttributes{
		&self.attributes[self.models[model_id.get() as usize].attr_id.get() as usize]
	}
	fn push_mesh(&mut self,mesh:PhysicsMesh){
		self.meshes.push(mesh);
	}
	fn push_model(&mut self,model:PhysicsModel)->PhysicsModelId{
		let model_id=PhysicsModelId::new(self.models.len() as u32);
		self.models.push(model);
		model_id
	}
	fn push_attr(&mut self,attr:PhysicsCollisionAttributes)->PhysicsAttributesId{
		let attr_id=PhysicsAttributesId::new(self.attributes.len() as u32);
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
	//angle limits could be an enum + struct that defines whether it's limited and selects clamp or wrap depending
	// enum AngleLimit{
	// 	Unlimited,
	// 	Limited{lower:Angle32,upper:Angle32},
	// }
	//pitch_limit:AngleLimit,
	//yaw_limit:AngleLimit,
}

impl PhysicsCamera {
	const ANGLE_PITCH_LOWER_LIMIT:Angle32=-Angle32::FRAC_PI_2;
	const ANGLE_PITCH_UPPER_LIMIT:Angle32=Angle32::FRAC_PI_2;
	pub fn move_mouse(&mut self,mouse_pos:glam::IVec2){
		let mut unclamped_mouse_pos=mouse_pos-self.mouse.pos+self.clamped_mouse_pos;
		unclamped_mouse_pos.y=unclamped_mouse_pos.y.clamp(
			self.sensitivity.y.rhs_div_int(Self::ANGLE_PITCH_LOWER_LIMIT.get() as i64) as i32,
			self.sensitivity.y.rhs_div_int(Self::ANGLE_PITCH_UPPER_LIMIT.get() as i64) as i32,
		);
		self.clamped_mouse_pos=unclamped_mouse_pos;
	}
	pub fn simulate_move_angles(&self,mouse_pos:glam::IVec2)->glam::Vec2 {
		let a=-self.sensitivity.mul_int((mouse_pos-self.mouse.pos+self.clamped_mouse_pos).as_i64vec2());
		let ax=Angle32::wrap_from_i64(a.x);
		let ay=Angle32::clamp_from_i64(a.y)
		//clamp to actual vertical cam limit
		.clamp(Self::ANGLE_PITCH_LOWER_LIMIT,Self::ANGLE_PITCH_UPPER_LIMIT);
		return glam::vec2(ax.into(),ay.into());
	}
	fn simulate_move_rotation(&self,mouse_pos:glam::IVec2)->Planar64Mat3{
		let a=-self.sensitivity.mul_int((mouse_pos-self.mouse.pos+self.clamped_mouse_pos).as_i64vec2());
		let ax=Angle32::wrap_from_i64(a.x);
		let ay=Angle32::clamp_from_i64(a.y)
		//clamp to actual vertical cam limit
		.clamp(Self::ANGLE_PITCH_LOWER_LIMIT,Self::ANGLE_PITCH_UPPER_LIMIT);
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
		}
	}
}

pub struct ModeState{
	mode_id:gameplay_modes::ModeId,
	stage_id:gameplay_modes::StageId,
	next_ordered_checkpoint_id:gameplay_modes::CheckpointId,//which OrderedCheckpoint model_id you must pass next (if 0 you haven't passed OrderedCheckpoint0)
	unordered_checkpoints:HashSet<ModelId>,
	jump_counts:HashMap<ModelId,u32>,//model_id -> jump count
}
impl std::default::Default for ModeState{
	fn default()->Self{
		Self{
			mode_id:gameplay_modes::ModeId::new(0),
			stage_id:gameplay_modes::StageId::new(0),
			next_ordered_checkpoint_id:gameplay_modes::CheckpointId::new(0),
			unordered_checkpoints:HashSet::new(),
			jump_counts:HashMap::new(),
		}
	}
}

struct WorldState{}

struct HitboxMesh{
	halfsize:Planar64Vec3,
	mesh:PhysicsMesh,
	transform:PhysicsMeshTransform,
}
impl HitboxMesh{
	fn new(mesh:PhysicsMesh,transform:integer::Planar64Affine3)->Self{
		//calculate extents
		let mut aabb=aabb::Aabb::default();
		for vert in mesh.complete_mesh_view().verts(){
			aabb.grow(transform.transform_point3(vert));
		}
		Self{
			halfsize:aabb.size()/2,
			mesh,
			transform:PhysicsMeshTransform::new(transform)
		}
	}
	#[inline]
	fn transformed_mesh(&self)->TransformedMesh{
		TransformedMesh::new(&self.mesh.complete_mesh_view(),&self.transform)
	}
}

trait StyleHelper{
	fn get_control(&self,control:u32,controls:u32)->bool;
	fn allow_strafe(&self,controls:u32)->bool;
	fn get_control_dir(&self,controls:u32)->Planar64Vec3;
	//fn get_jump_time(&self)->Planar64;
	//fn get_jump_height(&self)->Planar64;
	//fn get_jump_energy(&self)->Planar64;
	fn get_jump_deltav(&self)->Planar64;
	fn get_walk_target_velocity(&self,camera:&PhysicsCamera,controls:u32,next_mouse:&MouseState,time:Time,normal:&Planar64Vec3)->Planar64Vec3;
	fn get_ladder_target_velocity(&self,camera:&PhysicsCamera,controls:u32,next_mouse:&MouseState,time:Time,normal:&Planar64Vec3)->Planar64Vec3;
	fn get_propulsion_control_dir(&self,camera:&PhysicsCamera,controls:u32,next_mouse:&MouseState,time:Time)->Planar64Vec3;
	fn calculate_mesh(&self)->HitboxMesh;
}
impl StyleHelper for StyleModifiers{
	fn get_control(&self,control:u32,controls:u32)->bool{
		controls&self.controls_mask&control==control
	}

	fn allow_strafe(&self,controls:u32)->bool{
		//disable strafing according to strafe settings
		self.strafe.is_some_and(|s|s.mask(controls))
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
			&gameplay_style::JumpImpulse::FromTime(time)=>self.gravity.length()*(time/2),
			&gameplay_style::JumpImpulse::FromHeight(height)=>(self.gravity.length()*height*2).sqrt(),
			&gameplay_style::JumpImpulse::FromDeltaV(deltav)=>deltav,
			&gameplay_style::JumpImpulse::FromEnergy(energy)=>(energy*2/self.mass).sqrt(),
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
	fn calculate_mesh(&self)->HitboxMesh{
		let mesh=match self.hitbox.mesh{
			gameplay_style::HitboxMesh::Box=>PhysicsMesh::unit_cube(),
			gameplay_style::HitboxMesh::Cylinder=>PhysicsMesh::unit_cylinder(),
		};
		let transform=integer::Planar64Affine3::new(Planar64Mat3::from_diagonal(self.hitbox.halfsize),Planar64Vec3::ZERO);
		HitboxMesh::new(mesh,transform)
	}
}

enum MoveState{
	Air,
	Walk(WalkState),
	Water,
	Ladder(WalkState),
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
		contacting:gameplay_attributes::ContactingAttributes,
		general:gameplay_attributes::GeneralAttributes,
	},
	Intersect{//track whether you are intersecting the object
		intersecting:gameplay_attributes::IntersectingAttributes,
		general:gameplay_attributes::GeneralAttributes,
	},
}
struct NonPhysicsError;
impl TryFrom<&gameplay_attributes::CollisionAttributes> for PhysicsCollisionAttributes{
	type Error=NonPhysicsError;
	fn try_from(value:&gameplay_attributes::CollisionAttributes)->Result<Self,Self::Error>{
		match value{
			gameplay_attributes::CollisionAttributes::Decoration=>Err(NonPhysicsError),
			gameplay_attributes::CollisionAttributes::Contact{contacting,general}=>Ok(Self::Contact{contacting:contacting.clone(),general:general.clone()}),
			gameplay_attributes::CollisionAttributes::Intersect{intersecting,general}=>Ok(Self::Intersect{intersecting:intersecting.clone(),general:general.clone()}),
		}
	}
}
#[derive(id::Id)]
struct PhysicsAttributesId(u32);

//unique physics meshes indexed by this
#[derive(Debug,Clone,Copy,Eq,Hash,PartialEq)]
struct ConvexMeshId{
	model_id:PhysicsModelId,
	submesh_id:PhysicsSubmeshId,
}
#[derive(Debug,Clone,Copy,Hash,id::Id,Eq,PartialEq)]
struct PhysicsModelId(u32);
impl Into<ModelId> for PhysicsModelId{
	fn into(self)->ModelId{
		ModelId::new(self.0)
	}
}
impl From<ModelId> for PhysicsModelId{
	fn from(value:ModelId)->Self{
		Self::new(value.get())
	}
}
pub struct PhysicsModel{
	//A model is a thing that has a hitbox. can be represented by a list of TreyMesh-es
	//in this iteration, all it needs is extents.
	mesh_id:PhysicsMeshId,
	//put these up on the Model (data normalization)
	attr_id:PhysicsAttributesId,
	transform:PhysicsMeshTransform,
}

impl PhysicsModel{
	pub const fn new(mesh_id:PhysicsMeshId,attr_id:PhysicsAttributesId,transform:PhysicsMeshTransform)->Self{
		Self{
			mesh_id,
			attr_id,
			transform,
		}
	}
	const fn transform(&self)->&PhysicsMeshTransform{
		&self.transform
	}
}

#[derive(Debug,Clone,Eq,Hash,PartialEq)]
struct ContactCollision{
	face_id:model_physics::MinkowskiFace,
	convex_mesh_id:ConvexMeshId,
}
#[derive(Debug,Clone,Eq,Hash,PartialEq)]
struct IntersectCollision{
	convex_mesh_id:ConvexMeshId,
}
#[derive(Debug,Clone,Eq,Hash,PartialEq)]
enum Collision{
	Contact(ContactCollision),
	Intersect(IntersectCollision),
}
impl Collision{
	fn convex_mesh_id(&self)->ConvexMeshId{
		match self{
			&Collision::Contact(ContactCollision{convex_mesh_id,face_id:_})
			|&Collision::Intersect(IntersectCollision{convex_mesh_id})=>convex_mesh_id,
		}
	}
	fn face_id(&self)->Option<model_physics::MinkowskiFace>{
		match self{
			&Collision::Contact(ContactCollision{convex_mesh_id:_,face_id})=>Some(face_id),
			&Collision::Intersect(IntersectCollision{convex_mesh_id:_})=>None,
		}
	}
}
#[derive(Default)]
struct TouchingState{
	contacts:HashSet::<ContactCollision>,
	intersects:HashSet::<IntersectCollision>,
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
			match models.attr(contact.convex_mesh_id.model_id){
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
			match models.attr(intersect.convex_mesh_id.model_id){
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
	fn constrain_velocity(&self,models:&PhysicsModels,hitbox_mesh:&HitboxMesh,velocity:&mut Planar64Vec3){
		//TODO: trey push solve
		for contact in &self.contacts{
			let n=contact_normal(models,hitbox_mesh,contact);
			let d=n.dot128(*velocity);
			if d<0{
				*velocity-=n*Planar64::raw(((d<<32)/n.dot128(n)) as i64);
			}
		}
	}
	fn constrain_acceleration(&self,models:&PhysicsModels,hitbox_mesh:&HitboxMesh,acceleration:&mut Planar64Vec3){
		//TODO: trey push solve
		for contact in &self.contacts{
			let n=contact_normal(models,hitbox_mesh,contact);
			let d=n.dot128(*acceleration);
			if d<0{
				*acceleration-=n*Planar64::raw(((d<<32)/n.dot128(n)) as i64);
			}
		}
	}
	fn get_move_state(&self,body:&Body,models:&PhysicsModels,style:&StyleModifiers,hitbox_mesh:&HitboxMesh,camera:&PhysicsCamera,controls:u32,next_mouse:&MouseState,time:Time)->(MoveState,Planar64Vec3){
		//check current move conditions and use heuristics to determine
		//which ladder to climb on, which ground to walk on, etc
		//collect move state affecting objects from contacts (accelerator,water,ladder,ground)
		let gravity=self.base_acceleration(models,style,camera,controls,next_mouse,time);
		let mut move_state=MoveState::Air;
		let mut a=gravity;
		for contact in &self.contacts{
			match models.attr(contact.convex_mesh_id.model_id){
				PhysicsCollisionAttributes::Contact{contacting,general}=>{
					let normal=contact_normal(models,hitbox_mesh,contact);
					match &contacting.contact_behaviour{
						Some(gameplay_attributes::ContactingBehaviour::Ladder(_))=>{
							//ladder walkstate
							let mut target_velocity=style.get_ladder_target_velocity(camera,controls,next_mouse,time,&normal);
							self.constrain_velocity(models,hitbox_mesh,&mut target_velocity);
							let (walk_state,mut acceleration)=WalkState::ladder(body,style,gravity,target_velocity,contact.clone(),&normal);
							move_state=MoveState::Ladder(walk_state);
							self.constrain_acceleration(models,hitbox_mesh,&mut acceleration);
							a=acceleration;
						},
						None=>if style.surf_slope.map_or(true,|s|normal.walkable(s,Planar64Vec3::Y)){
							//check ground
							let mut target_velocity=style.get_walk_target_velocity(camera,controls,next_mouse,time,&normal);
							self.constrain_velocity(models,hitbox_mesh,&mut target_velocity);
							let (walk_state,mut acceleration)=WalkState::ground(body,style,gravity,target_velocity,contact.clone(),&normal);
							move_state=MoveState::Walk(walk_state);
							self.constrain_acceleration(models,hitbox_mesh,&mut acceleration);
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
		self.constrain_acceleration(models,hitbox_mesh,&mut a);
		(move_state,a)
	}
	fn predict_collision_end(&self,collector:&mut instruction::InstructionCollector<PhysicsInstruction>,models:&PhysicsModels,hitbox_mesh:&HitboxMesh,body:&Body,time:Time){
		let relative_body=VirtualBody::relative(&Body::default(),body).body(time);
		for contact in &self.contacts{
			//detect face slide off
			let model_mesh=models.mesh(contact.convex_mesh_id);
			let minkowski=model_physics::MinkowskiMesh::minkowski_sum(model_mesh,hitbox_mesh.transformed_mesh());
			collector.collect(minkowski.predict_collision_face_out(&relative_body,collector.time(),contact.face_id).map(|(face,time)|{
				TimedInstruction{
					time,
					instruction:PhysicsInstruction::CollisionEnd(
						Collision::Contact(ContactCollision{convex_mesh_id:contact.convex_mesh_id,face_id:contact.face_id})
					),
				}
			}));
		}
		for intersect in &self.intersects{
			//detect model collision in reverse
			let model_mesh=models.mesh(intersect.convex_mesh_id);
			let minkowski=model_physics::MinkowskiMesh::minkowski_sum(model_mesh,hitbox_mesh.transformed_mesh());
			collector.collect(minkowski.predict_collision_out(&relative_body,collector.time()).map(|(face,time)|{
				TimedInstruction{
					time,
					instruction:PhysicsInstruction::CollisionEnd(
						Collision::Intersect(IntersectCollision{convex_mesh_id:intersect.convex_mesh_id})
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
	pub fn grow_aabb(&self,aabb:&mut aabb::Aabb,t0:Time,t1:Time){
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

pub struct PhysicsState{
	time:Time,
	body:Body,
	world:WorldState,//currently there is only one state the world can be in
	mode_state:ModeState,
	style:StyleModifiers,//mode style with custom style updates applied
	touching:TouchingState,
	//camera must exist in state because wormholes modify the camera, also camera punch
	camera:PhysicsCamera,
	pub next_mouse:MouseState,//Where is the mouse headed next
	controls:u32,//TODO this should be a struct
	move_state:MoveState,
}
//random collection of contextual data that doesn't belong in PhysicsState
pub struct PhysicsData{
	//permanent map data
	bvh:bvh::BvhNode<ConvexMeshId>,
	modes:gameplay_modes::Modes,
	//transient map/environment data (open world may load/unload)
	models:PhysicsModels,
	//cached calculations
	hitbox_mesh:HitboxMesh,
}
impl Default for PhysicsState{
	fn default()->Self{
 		Self{
			body:Body::new(Planar64Vec3::int(0,50,0),Planar64Vec3::int(0,0,0),Planar64Vec3::int(0,-100,0),Time::ZERO),
			time:Time::ZERO,
			style:StyleModifiers::default(),
			touching:TouchingState::default(),
			move_state: MoveState::Air,
			camera:PhysicsCamera::default(),
			next_mouse:MouseState::default(),
			controls:0,
			world:WorldState{},
			mode_state:ModeState::default(),
		}
	}
}

impl PhysicsState {
	pub fn clear(&mut self){
		self.touching.clear();
	}

	pub fn output(&self)->PhysicsOutputState{
		PhysicsOutputState{
			body:self.body.clone(),
			camera:self.camera.clone(),
			camera_offset:self.style.camera_offset.clone(),
		}
	}

	pub fn load_user_settings(&mut self,user_settings:&crate::settings::UserSettings){
		self.camera.sensitivity=user_settings.calculate_sensitivity();
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
				time:strafe.next_tick(self.time),
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
pub struct PhysicsContext{
	pub state:PhysicsState,//this captures the entire state of the physics.
	data:PhysicsData,//data currently loaded into memory which is needded for physics to run, but is not part of the state.
}
impl instruction::InstructionConsumer<PhysicsInstruction> for PhysicsContext{
	fn process_instruction(&mut self,ins:TimedInstruction<PhysicsInstruction>){
		atomic_state_update(&mut self.state,&self.data,ins)
	}
}
impl instruction::InstructionEmitter<PhysicsInstruction> for PhysicsContext{
	//this little next instruction function can cache its return value and invalidate the cached value by watching the State.
	fn next_instruction(&self,time_limit:Time)->Option<TimedInstruction<PhysicsInstruction>>{
		literally_next_instruction_but_with_context(&self.state,&self.data,time_limit)
	}
}
impl PhysicsContext{
	pub fn spawn(&mut self){
		self.state.mode_state.stage_id=gameplay_modes::StageId::new(0);
		self.process_instruction(instruction::TimedInstruction{
			time:self.state.time,
			instruction: PhysicsInstruction::Input(PhysicsInputInstruction::Reset),
		});
	}

	pub fn generate_models(&mut self,map:&map::CompleteMap){
		let mut starts=Vec::new();
		let mut spawns=Vec::new();
		let mut attr_hash=HashMap::new();
		for model in &map.models{
			let mesh_id=self.data.models.meshes.len();
			let mut make_mesh=false;
			for model_instance in &model.instances{
				if let Ok(physics_attributes)=PhysicsCollisionAttributes::try_from(&model_instance.attributes){
					let attr_id=if let Some(&attr_id)=attr_hash.get(&physics_attributes){
						attr_id
					}else{
						let attr_id=self.data.models.push_attr(physics_attributes.clone());
						attr_hash.insert(physics_attributes,attr_id);
						attr_id
					};
					let model_physics=PhysicsModel::new(mesh_id,attr_id,model_instance.transform);
					make_mesh=true;
					self.data.models.push_model(model_physics);
				}
			}
			if make_mesh{
				self.data.models.push_mesh(PhysicsMesh::from(model));
			}
		}
		let convex_mesh_aabb_list=self.data.models.models.iter()
		.enumerate().flat_map(|(model_id,model)|{
			self.data.models.meshes[model.mesh_id.get() as usize].submesh_views()
			.enumerate().map(|(submesh_id,view)|{
				let mut aabb=aabb::Aabb::default();
				for v in view.verts(){
					aabb.grow(v)
				}
				(ConvexMeshId{
					model_id:PhysicsModelId::new(model_id as u32),
					submesh_id:PhysicsSubmeshId::new(submesh_id as u32),
				},aabb)
			})
		}).collect();
		self.data.bvh=bvh::generate_bvh_node(convex_mesh_aabb_list);
		println!("Physics Objects: {}",self.data.models.models.len());
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

	//TODO get rid of this trash
	fn refresh_walk_target(&mut self)->Planar64Vec3{
		match &mut self.state.move_state{
			MoveState::Air|MoveState::Water=>self.state.touching.base_acceleration(&self.data.models,&self.state.style,&self.state.camera,self.state.controls,&self.state.next_mouse,self.state.time),
			MoveState::Walk(WalkState{state,contact,jump_direction:_})=>{
				let n=contact_normal(&self.data.models,&self.data.hitbox_mesh,contact);
				let gravity=self.state.touching.base_acceleration(&self.data.models,&self.state.style,&self.state.camera,self.state.controls,&self.state.next_mouse,self.state.time);
				let mut a;
				let mut v=self.state.style.get_walk_target_velocity(&self.state.camera,self.state.controls,&self.state.next_mouse,self.state.time,&n);
				self.state.touching.constrain_velocity(&self.data.models,&self.data.hitbox_mesh,&mut v);
				let normal_accel=-n.dot(gravity)/n.length();
				(*state,a)=WalkEnum::with_target_velocity(&self.state.body,&self.state.style,v,&n,self.state.style.walk_speed,normal_accel);
				a
			},
			MoveState::Ladder(WalkState{state,contact,jump_direction:_})=>{
				let n=contact_normal(&self.data.models,&self.data.hitbox_mesh,contact);
				let gravity=self.state.touching.base_acceleration(&self.data.models,&self.state.style,&self.state.camera,self.state.controls,&self.state.next_mouse,self.state.time);
				let mut a;
				let mut v=self.state.style.get_ladder_target_velocity(&self.state.camera,self.state.controls,&self.state.next_mouse,self.state.time,&n);
				self.state.touching.constrain_velocity(&self.data.models,&self.data.hitbox_mesh,&mut v);
				(*state,a)=WalkEnum::with_target_velocity(&self.state.body,&self.state.style,v,&n,self.state.style.ladder_speed,self.state.style.ladder_accel);
				a
			},
		}
	}
}

	fn literally_next_instruction_but_with_context(state:&PhysicsState,data:&PhysicsData,time_limit:Time)->Option<TimedInstruction<PhysicsInstruction>>{
		//JUST POLLING!!! NO MUTATION
		let mut collector = instruction::InstructionCollector::new(time_limit);

		collector.collect(state.next_move_instruction());

		//check for collision ends
		state.touching.predict_collision_end(&mut collector,&data.models,&data.hitbox_mesh,&state.body,state.time);
		//check for collision starts
		let mut aabb=aabb::Aabb::default();
		state.body.grow_aabb(&mut aabb,state.time,collector.time());
		aabb.inflate(state.style.hitbox.halfsize);
		//common body
		let relative_body=VirtualBody::relative(&Body::default(),&state.body).body(state.time);
		let hitbox_mesh=data.hitbox_mesh.transformed_mesh();
		data.bvh.the_tester(&aabb,&mut |convex_mesh_id|{
			//no checks are needed because of the time limits.
			let model_mesh=data.models.mesh(convex_mesh_id);
			let minkowski=model_physics::MinkowskiMesh::minkowski_sum(model_mesh,hitbox_mesh);
			collector.collect(minkowski.predict_collision_in(&relative_body,collector.time())
				//temp (?) code to avoid collision loops
				.map_or(None,|(face,time)|if time==state.time{None}else{Some((face,time))})
				.map(|(face,time)|{
				TimedInstruction{time,instruction:PhysicsInstruction::CollisionStart(match data.models.attr(convex_mesh_id.model_id){
					PhysicsCollisionAttributes::Contact{contacting:_,general:_}=>Collision::Contact(ContactCollision{convex_mesh_id,face_id:face}),
					PhysicsCollisionAttributes::Intersect{intersecting:_,general:_}=>Collision::Intersect(IntersectCollision{convex_mesh_id}),
				})}
			}));
		});
		collector.instruction()
	}

fn get_walk_state(move_state:&MoveState)->Option<&WalkState>{
	match move_state{
		MoveState::Walk(walk_state)|MoveState::Ladder(walk_state)=>Some(walk_state),
		MoveState::Air|MoveState::Water=>None,
	}
}

fn jumped_velocity(models:&PhysicsModels,style:&StyleModifiers,hitbox_mesh:&HitboxMesh,walk_state:&WalkState,v:&mut Planar64Vec3){
	let jump_dir=match &walk_state.jump_direction{
		JumpDirection::FromContactNormal=>contact_normal(models,hitbox_mesh,&walk_state.contact),
		&JumpDirection::Exactly(dir)=>dir,
	};
	*v=*v+jump_dir*(style.get_jump_deltav()/jump_dir.length());
}

fn contact_normal(models:&PhysicsModels,hitbox_mesh:&HitboxMesh,contact:&ContactCollision)->Planar64Vec3{
	let model_mesh=models.mesh(contact.convex_mesh_id);
	let minkowski=model_physics::MinkowskiMesh::minkowski_sum(model_mesh,hitbox_mesh.transformed_mesh());
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
fn set_velocity_cull(body:&mut Body,touching:&mut TouchingState,models:&PhysicsModels,hitbox_mesh:&HitboxMesh,v:Planar64Vec3)->bool{
	//This is not correct but is better than what I have
	let mut culled=false;
	touching.contacts.retain(|contact|{
		let n=contact_normal(models,hitbox_mesh,contact);
		let r=n.dot(v)<=Planar64::ZERO;
		if !r{
			culled=true;
			println!("set_velocity_cull contact={:?}",contact);
		}
		r
	});
	set_velocity(body,touching,models,hitbox_mesh,v);
	culled
}
fn set_velocity(body:&mut Body,touching:&TouchingState,models:&PhysicsModels,hitbox_mesh:&HitboxMesh,mut v:Planar64Vec3)->Planar64Vec3{
	touching.constrain_velocity(models,hitbox_mesh,&mut v);
	body.velocity=v;
	v
}
fn set_acceleration_cull(body:&mut Body,touching:&mut TouchingState,models:&PhysicsModels,hitbox_mesh:&HitboxMesh,a:Planar64Vec3)->bool{
	//This is not correct but is better than what I have
	let mut culled=false;
	touching.contacts.retain(|contact|{
		let n=contact_normal(models,hitbox_mesh,contact);
		let r=n.dot(a)<=Planar64::ZERO;
		if !r{
			culled=true;
			println!("set_acceleration_cull contact={:?}",contact);
		}
		r
	});
	set_acceleration(body,touching,models,hitbox_mesh,a);
	culled
}
fn set_acceleration(body:&mut Body,touching:&TouchingState,models:&PhysicsModels,hitbox_mesh:&HitboxMesh,mut a:Planar64Vec3)->Planar64Vec3{
	touching.constrain_acceleration(models,hitbox_mesh,&mut a);
	body.acceleration=a;
	a
}

fn teleport(body:&mut Body,touching:&mut TouchingState,models:&PhysicsModels,style:&StyleModifiers,hitbox_mesh:&HitboxMesh,point:Planar64Vec3)->MoveState{
	set_position(body,touching,point);
	set_acceleration(body,touching,models,hitbox_mesh,style.gravity);
	MoveState::Air
}
fn teleport_to_spawn(body:&mut Body,touching:&mut TouchingState,style:&StyleModifiers,hitbox_mesh:&HitboxMesh,mode:&gameplay_modes::Mode,models:&PhysicsModels,stage_id:gameplay_modes::StageId)->Option<MoveState>{
	let model=models.model(mode.get_spawn_model_id(stage_id)?.into());
	let point=model.transform.vertex.transform_point3(Planar64Vec3::Y)+Planar64Vec3::Y*(style.hitbox.halfsize.y()+Planar64::ONE/16);
	Some(teleport(body,touching,models,style,hitbox_mesh,point))
}

fn run_teleport_behaviour(wormhole:&Option<gameplay_attributes::Wormhole>,game:&mut ModeState,models:&PhysicsModels,mode:&gameplay_modes::Mode,style:&StyleModifiers,hitbox_mesh:&HitboxMesh,touching:&mut TouchingState,body:&mut Body,convex_mesh_id:ConvexMeshId)->Option<MoveState>{
	//TODO: jump count and checkpoints are always reset on teleport.
	//Map makers are expected to use tools to prevent
	//multi-boosting on JumpLimit boosters such as spawning into a SetVelocity
	if let Some(stage_element)=mode.get_element(convex_mesh_id.model_id.into()){
		let stage=mode.get_stage(stage_element.stage_id)?;
		if stage_element.force||game.stage_id<stage_element.stage_id{
			//TODO: check if all checkpoints are complete up to destination stage id, otherwise set to checkpoint completion stage it
			game.stage_id=stage_element.stage_id;
		}
		match &stage_element.behaviour{
			gameplay_modes::StageElementBehaviour::SpawnAt=>(),
			gameplay_modes::StageElementBehaviour::Trigger
			|gameplay_modes::StageElementBehaviour::Teleport=>{
				//I guess this is correct behaviour when trying to teleport to a non-existent spawn but it's still weird
				return teleport_to_spawn(body,touching,style,hitbox_mesh,mode,models,game.stage_id);
			},
			gameplay_modes::StageElementBehaviour::Platform=>(),
			&gameplay_modes::StageElementBehaviour::Checkpoint=>{
				//checkpoint check
				//TODO: need to check all stages
				if stage.ordered_checkpoint_id.map_or(true,|id|id<game.next_ordered_checkpoint_id)
					&&stage.unordered_checkpoint_count<=game.unordered_checkpoints.len() as u32{
					//pass
				}else{
					//fail
					return teleport_to_spawn(body,touching,style,hitbox_mesh,mode,models,game.stage_id);
				}
			},
		}
		if let Some(next_checkpoint)=stage.ordered_checkpoints.get(&game.next_ordered_checkpoint_id){
			if convex_mesh_id==next_checkpoint{
				//if you hit the next number in a sequence of ordered checkpoints
				//increment the current checkpoint id
				game.next_ordered_checkpoint_id=gameplay_modes::CheckpointId::new(game.next_ordered_checkpoint_id.get()+1);
			}
		}
		if stage.unordered_checkpoints.contains(convex_mesh_id.model_id.into()){
			//count model id in accumulated unordered checkpoints
			game.unordered_checkpoints.insert(convex_mesh_id.model_id.into());
		}
	}
	match wormhole{
		&Some(gameplay_attributes::Wormhole{destination_model})=>{
			let origin_model=models.model(convex_mesh_id.model_id);
			let destination_model=models.model(destination_model.into());
			//ignore the transform for now
			Some(teleport(body,touching,models,style,hitbox_mesh,body.position-origin_model.transform.vertex.translation+destination_model.transform.vertex.translation))
		}
		None=>None,
	}
}

	fn atomic_state_update(state:&mut PhysicsState,data:&PhysicsData,ins:TimedInstruction<PhysicsInstruction>){
		match &ins.instruction{
			PhysicsInstruction::Input(PhysicsInputInstruction::Idle)
			|PhysicsInstruction::Input(PhysicsInputInstruction::SetNextMouse(_))
			|PhysicsInstruction::Input(PhysicsInputInstruction::ReplaceMouse(_,_))
			|PhysicsInstruction::StrafeTick=>(),
			_=>println!("{}|{:?}",ins.time,ins.instruction),
		}
		//selectively update body
		match &ins.instruction{
			PhysicsInstruction::Input(PhysicsInputInstruction::Idle)=>state.time=ins.time,//idle simply updates time
			PhysicsInstruction::Input(_)
			|PhysicsInstruction::ReachWalkTargetVelocity
			|PhysicsInstruction::CollisionStart(_)
			|PhysicsInstruction::CollisionEnd(_)
			|PhysicsInstruction::StrafeTick=>state.advance_time(ins.time),
		}
		match ins.instruction{
			PhysicsInstruction::CollisionStart(c)=>{
				let convex_mesh_id=c.convex_mesh_id();
				match (data.models.attr(convex_mesh_id.model_id),&c){
					(PhysicsCollisionAttributes::Contact{contacting,general},Collision::Contact(contact))=>{
						let mut v=state.body.velocity;
						let normal=contact_normal(&data.models,&data.hitbox_mesh,contact);
						match &contacting.contact_behaviour{
							Some(gameplay_attributes::ContactingBehaviour::Surf)=>println!("I'm surfing!"),
							Some(gameplay_attributes::ContactingBehaviour::Cling)=>println!("Unimplemented!"),
							&Some(gameplay_attributes::ContactingBehaviour::Elastic(elasticity))=>{
								//velocity and normal are facing opposite directions so this is inherently negative.
								let d=normal.dot(v)*(Planar64::ONE+Planar64::raw(elasticity as i64+1));
								v+=normal*(d/normal.dot(normal));
							},
							Some(gameplay_attributes::ContactingBehaviour::Ladder(contacting_ladder))=>{
								if contacting_ladder.sticky{
									//kill v
									//actually you could do this with a booster attribute :thinking:
									v=Planar64Vec3::ZERO;//model.velocity
								}
								//ladder walkstate
								let gravity=state.touching.base_acceleration(&data.models,&state.style,&state.camera,state.controls,&state.next_mouse,state.time);
								let mut target_velocity=state.style.get_ladder_target_velocity(&state.camera,state.controls,&state.next_mouse,state.time,&normal);
								state.touching.constrain_velocity(&data.models,&data.hitbox_mesh,&mut target_velocity);
								let (walk_state,a)=WalkState::ladder(&state.body,&state.style,gravity,target_velocity,contact.clone(),&normal);
								state.move_state=MoveState::Ladder(walk_state);
								set_acceleration(&mut state.body,&state.touching,&data.models,&data.hitbox_mesh,a);
							}
							None=>if state.style.surf_slope.map_or(true,|s|contact_normal(&data.models,&data.hitbox_mesh,contact).walkable(s,Planar64Vec3::Y)){
								//ground
								let gravity=state.touching.base_acceleration(&data.models,&state.style,&state.camera,state.controls,&state.next_mouse,state.time);
								let mut target_velocity=state.style.get_walk_target_velocity(&state.camera,state.controls,&state.next_mouse,state.time,&normal);
								state.touching.constrain_velocity(&data.models,&data.hitbox_mesh,&mut target_velocity);
								let (walk_state,a)=WalkState::ground(&state.body,&state.style,gravity,target_velocity,contact.clone(),&normal);
								state.move_state=MoveState::Walk(walk_state);
								set_acceleration(&mut state.body,&state.touching,&data.models,&data.hitbox_mesh,a);
							},
						}
						//check ground
						state.touching.insert(c);
						//I love making functions with 10 arguments to dodge the borrow checker
						run_teleport_behaviour(&general.wormhole,&mut state.mode_state,&data.models,&data.modes.get_mode(state.mode_state.mode_id).unwrap(),&state.style,&data.hitbox_mesh,&mut state.touching,&mut state.body,convex_mesh_id);
						//flatten v
						state.touching.constrain_velocity(&data.models,&data.hitbox_mesh,&mut v);
						match &general.booster{
							Some(booster)=>{
								//DELETE THIS when boosters get converted to height machines
								match booster{
									//&gameplay_attributes::Booster::Affine(transform)=>v=transform.transform_point3(v),
									&gameplay_attributes::Booster::Velocity(velocity)=>v+=velocity,
									&gameplay_attributes::Booster::Energy{direction: _,energy: _}=>todo!(),
								}
							},
							None=>(),
						}
						let calc_move=if state.style.get_control(StyleModifiers::CONTROL_JUMP,state.controls){
							if let Some(walk_state)=get_walk_state(&state.move_state){
								jumped_velocity(&data.models,&state.style,&data.hitbox_mesh,walk_state,&mut v);
								set_velocity_cull(&mut state.body,&mut state.touching,&data.models,&data.hitbox_mesh,v)
							}else{false}
						}else{false};
						match &general.trajectory{
							Some(trajectory)=>{
								match trajectory{
									gameplay_attributes::SetTrajectory::AirTime(_) => todo!(),
									gameplay_attributes::SetTrajectory::Height(_) => todo!(),
									gameplay_attributes::SetTrajectory::TargetPointTime { target_point: _, time: _ } => todo!(),
									gameplay_attributes::SetTrajectory::TargetPointSpeed { target_point: _, speed: _, trajectory_choice: _ } => todo!(),
									&gameplay_attributes::SetTrajectory::Velocity(velocity)=>v=velocity,
									gameplay_attributes::SetTrajectory::DotVelocity { direction: _, dot: _ } => todo!(),
								}
							},
							None=>(),
						}
						set_velocity(&mut state.body,&state.touching,&data.models,&data.hitbox_mesh,v);
						//not sure if or is correct here
						if calc_move||Planar64::ZERO<normal.dot(v){
							(state.move_state,state.body.acceleration)=state.touching.get_move_state(&state.body,&data.models,&state.style,&data.hitbox_mesh,&state.camera,state.controls,&state.next_mouse,state.time);
						}
						let a=state.refresh_walk_target();
						set_acceleration(&mut state.body,&state.touching,&data.models,&data.hitbox_mesh,a);
					},
					(PhysicsCollisionAttributes::Intersect{intersecting: _,general},Collision::Intersect(intersect))=>{
						//I think that setting the velocity to 0 was preventing surface contacts from entering an infinite loop
						state.touching.insert(c);
						run_teleport_behaviour(&general.wormhole,&mut state.mode_state,&data.models,&data.modes.get_mode(state.mode_state.mode_id).unwrap(),&state.style,&data.hitbox_mesh,&mut state.touching,&mut state.body,convex_mesh_id);
					},
					_=>panic!("invalid pair"),
				}
			},
			PhysicsInstruction::CollisionEnd(c) => {
				match data.models.attr(c.convex_mesh_id().model_id){
					PhysicsCollisionAttributes::Contact{contacting:_,general:_}=>{
						state.touching.remove(&c);//remove contact before calling contact_constrain_acceleration
						//check ground
						(state.move_state,state.body.acceleration)=state.touching.get_move_state(&state.body,&data.models,&state.style,&data.hitbox_mesh,&state.camera,state.controls,&state.next_mouse,state.time);
					},
					PhysicsCollisionAttributes::Intersect{intersecting:_,general:_}=>{
						state.touching.remove(&c);
					},
				}
			},
			PhysicsInstruction::StrafeTick => {
				let control_dir=state.style.get_control_dir(state.controls);
				if control_dir!=Planar64Vec3::ZERO{
					let camera_mat=state.camera.simulate_move_rotation_y(state.camera.mouse.lerp(&state.next_mouse,state.time).x);
					let control_dir=camera_mat*control_dir;
					//normalize but careful for zero
					let d=state.body.velocity.dot(control_dir);
					if d<state.style.mv {
						let v=state.body.velocity+control_dir*(state.style.mv-d);
						//this is wrong but will work ig
						//need to note which push planes activate in push solve and keep those
						if set_velocity_cull(&mut state.body,&mut state.touching,&data.models,&data.hitbox_mesh,v){
							(state.move_state,state.body.acceleration)=state.touching.get_move_state(&state.body,&data.models,&state.style,&data.hitbox_mesh,&state.camera,state.controls,&state.next_mouse,state.time);
						}
					}
				}
			}
			PhysicsInstruction::ReachWalkTargetVelocity => {
				match &mut state.move_state{
					MoveState::Air|MoveState::Water=>(),
					MoveState::Walk(walk_state)|MoveState::Ladder(walk_state)=>{
						match &mut walk_state.state{
							WalkEnum::Reached=>(),
							WalkEnum::Transient(walk_target)=>{
								//precisely set velocity
								let a=Planar64Vec3::ZERO;//ignore gravity for now.
								set_acceleration(&mut state.body,&state.touching,&data.models,&data.hitbox_mesh,a);
								let v=walk_target.velocity;
								set_velocity(&mut state.body,&state.touching,&data.models,&data.hitbox_mesh,v);
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
						state.camera.move_mouse(state.next_mouse.pos);
						(state.camera.mouse,state.next_mouse)=(state.next_mouse.clone(),m);
					},
					PhysicsInputInstruction::ReplaceMouse(m0,m1) => {
						state.camera.move_mouse(m0.pos);
						(state.camera.mouse,state.next_mouse)=(m0,m1);
					},
					PhysicsInputInstruction::SetMoveForward(s) => state.set_control(StyleModifiers::CONTROL_MOVEFORWARD,s),
					PhysicsInputInstruction::SetMoveLeft(s) => state.set_control(StyleModifiers::CONTROL_MOVELEFT,s),
					PhysicsInputInstruction::SetMoveBack(s) => state.set_control(StyleModifiers::CONTROL_MOVEBACK,s),
					PhysicsInputInstruction::SetMoveRight(s) => state.set_control(StyleModifiers::CONTROL_MOVERIGHT,s),
					PhysicsInputInstruction::SetMoveUp(s) => state.set_control(StyleModifiers::CONTROL_MOVEUP,s),
					PhysicsInputInstruction::SetMoveDown(s) => state.set_control(StyleModifiers::CONTROL_MOVEDOWN,s),
					PhysicsInputInstruction::SetJump(s) => {
						state.set_control(StyleModifiers::CONTROL_JUMP,s);
						if let Some(walk_state)=get_walk_state(&state.move_state){
							let mut v=state.body.velocity;
							jumped_velocity(&data.models,&state.style,&data.hitbox_mesh,walk_state,&mut v);
							if set_velocity_cull(&mut state.body,&mut state.touching,&data.models,&data.hitbox_mesh,v){
								(state.move_state,state.body.acceleration)=state.touching.get_move_state(&state.body,&data.models,&state.style,&data.hitbox_mesh,&state.camera,state.controls,&state.next_mouse,state.time);
							}
						}
						refresh_walk_target=false;
					},
					PhysicsInputInstruction::SetZoom(s) => {
						state.set_control(StyleModifiers::CONTROL_ZOOM,s);
						refresh_walk_target=false;
					},
					PhysicsInputInstruction::Reset => {
						//it matters which of these runs first, but I have not thought it through yet as it doesn't matter yet
						let spawn_point={
							let mode=data.modes.get_mode(state.mode_state.mode_id).unwrap();
							let stage=mode.get_stage(gameplay_modes::StageId::FIRST).unwrap();
							data.models.model(stage.spawn().into()).transform.vertex.translation
						};
						set_position(&mut state.body,&mut state.touching,spawn_point);
						set_velocity(&mut state.body,&state.touching,&data.models,&data.hitbox_mesh,Planar64Vec3::ZERO);
						(state.move_state,state.body.acceleration)=state.touching.get_move_state(&state.body,&data.models,&state.style,&data.hitbox_mesh,&state.camera,state.controls,&state.next_mouse,state.time);
						refresh_walk_target=false;
					},
					PhysicsInputInstruction::Idle => {refresh_walk_target=false;},//literally idle!
				}
				if refresh_walk_target{
					let a=state.refresh_walk_target();
					if set_acceleration_cull(&mut state.body,&mut state.touching,&data.models,&data.hitbox_mesh,a){
						(state.move_state,state.body.acceleration)=state.touching.get_move_state(&state.body,&data.models,&state.style,&data.hitbox_mesh,&state.camera,state.controls,&state.next_mouse,state.time);
					}
				}
			},
		}
	}

#[allow(dead_code)]
fn test_collision_axis_aligned(relative_body:Body,expected_collision_time:Option<Time>){
	let h0=HitboxMesh::from_mesh_scale(PhysicsMesh::unit_cube(),Planar64Vec3::int(5,1,5)/2);
	let h1=HitboxMesh::roblox();
	let hitbox_mesh=h1.transformed_mesh();
	let platform_mesh=h0.transformed_mesh();
	let minkowski=model_physics::MinkowskiMesh::minkowski_sum(&platform_mesh,&hitbox_mesh);
	let collision=minkowski.predict_collision_in(&relative_body,Time::MAX);
	assert_eq!(collision.map(|tup|tup.1),expected_collision_time,"Incorrect time of collision");
}
#[allow(dead_code)]
fn test_collision_rotated(relative_body:Body,expected_collision_time:Option<Time>){
	let h0=HitboxMesh::new(PhysicsMesh::unit_cube(),
		integer::Planar64Affine3::new(
			integer::Planar64Mat3::from_cols(
				Planar64Vec3::int(5,0,1)/2,
				Planar64Vec3::int(0,1,0)/2,
				Planar64Vec3::int(-1,0,5)/2,
			),
			Planar64Vec3::ZERO,
		)
	);
	let h1=HitboxMesh::roblox();
	let hitbox_mesh=h1.transformed_mesh();
	let platform_mesh=h0.transformed_mesh();
	let minkowski=model_physics::MinkowskiMesh::minkowski_sum(platform_mesh,hitbox_mesh);
	let collision=minkowski.predict_collision_in(&relative_body,Time::MAX);
	assert_eq!(collision.map(|tup|tup.1),expected_collision_time,"Incorrect time of collision");
}
#[allow(dead_code)]
fn test_collision(relative_body:Body,expected_collision_time:Option<Time>){
	test_collision_axis_aligned(relative_body.clone(),expected_collision_time);
	test_collision_rotated(relative_body,expected_collision_time);
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
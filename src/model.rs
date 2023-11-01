use crate::integer::{Time,Planar64,Planar64Vec3,Planar64Affine3};
pub type TextureCoordinate=glam::Vec2;
pub type Color4=glam::Vec4;
#[derive(Clone,Hash,PartialEq,Eq)]
pub struct IndexedVertex{
	pub pos:u32,
	pub tex:u32,
	pub normal:u32,
	pub color:u32,
}
pub struct IndexedPolygon{
	pub vertices:Vec<u32>,
}
pub struct IndexedGroup{
	pub texture:Option<u32>,//RenderPattern? material/texture/shader/flat color
	pub polys:Vec<IndexedPolygon>,
}
pub struct IndexedModel{
	pub unique_pos:Vec<Planar64Vec3>,
	pub unique_normal:Vec<Planar64Vec3>,
	pub unique_tex:Vec<TextureCoordinate>,
	pub unique_color:Vec<Color4>,
	pub unique_vertices:Vec<IndexedVertex>,
	pub groups: Vec<IndexedGroup>,
	pub instances:Vec<ModelInstance>,
}
pub struct ModelInstance{
	//pub id:u64,//this does not actually help with map fixes resimulating bots, they must always be resimulated
	pub transform:Planar64Affine3,
	pub color:Color4,//transparency is in here
	pub attributes:CollisionAttributes,
	pub temp_indexing:Vec<TempIndexedAttributes>,
}
impl std::default::Default for ModelInstance{
	fn default() -> Self {
		Self{
			color:Color4::ONE,
			transform:Default::default(),
			attributes:Default::default(),
			temp_indexing:Default::default(),
		}
	}
}
pub struct IndexedModelInstances{
	pub textures:Vec<String>,//RenderPattern
	pub models:Vec<IndexedModel>,
	//may make this into an object later.
	pub modes:Vec<ModeDescription>,
	pub spawn_point:Planar64Vec3,
}
//stage description referencing flattened ids is spooky, but the map loading is meant to be deterministic.
pub struct ModeDescription{
	//TODO: put "default" style modifiers in mode
	//pub style:StyleModifiers,
	pub start:usize,//start=model_id
	pub spawns:Vec<usize>,//spawns[spawn_id]=model_id
	pub spawn_from_stage_id:std::collections::HashMap::<u32,usize>,
	pub ordered_checkpoint_from_checkpoint_id:std::collections::HashMap::<u32,usize>,
}
impl ModeDescription{
	pub fn get_spawn_model_id(&self,stage_id:u32)->Option<&usize>{
		self.spawns.get(*self.spawn_from_stage_id.get(&stage_id)?)
	}
}
//I don't want this code to exist!
#[derive(Clone)]
pub struct TempAttrStart{
	pub mode_id:u32,
}
#[derive(Clone)]
pub struct TempAttrSpawn{
	pub mode_id:u32,
	pub stage_id:u32,
}
#[derive(Clone)]
pub struct TempAttrWormhole{
	pub wormhole_id:u32,
}
pub enum TempIndexedAttributes{
	Start(TempAttrStart),
	Spawn(TempAttrSpawn),
	Wormhole(TempAttrWormhole),
}

//you have this effect while in contact
#[derive(Clone)]
pub struct ContactingLadder{
	pub sticky:bool
}
#[derive(Clone)]
pub enum ContactingBehaviour{
    Surf,
    Ladder(ContactingLadder),
    Elastic(u32),//[1/2^32,1] 0=None (elasticity+1)/2^32
}
//you have this effect while intersecting
#[derive(Clone)]
pub struct IntersectingWater{
	pub viscosity:Planar64,
	pub density:Planar64,
	pub current:Planar64Vec3,
}
//All models can be given these attributes
#[derive(Clone)]
pub struct GameMechanicAccelerator{
	pub acceleration:Planar64Vec3
}
#[derive(Clone)]
pub enum GameMechanicBooster{
	Affine(Planar64Affine3),//capable of SetVelocity,DotVelocity,normal booster,bouncy part,redirect velocity, and much more
	Velocity(Planar64Vec3),//straight up boost velocity adds to your current velocity
	Energy{direction:Planar64Vec3,energy:Planar64},//increase energy in direction
}
#[derive(Clone)]
pub enum GameMechanicCheckpoint{
	Ordered{
		mode_id:u32,
		checkpoint_id:u32,
	},
	Unordered{
		mode_id:u32,
	},
}
#[derive(Clone)]
pub enum TrajectoryChoice{
	HighArcLongDuration,//underhand lob at target: less horizontal speed and more air time
	LowArcShortDuration,//overhand throw at target: more horizontal speed and less air time
}
#[derive(Clone)]
pub enum GameMechanicSetTrajectory{
	AirTime(Time),//air time (relative to gravity direction) is invariant across mass and gravity changes
	Height(Planar64),//boost height (relative to gravity direction) is invariant across mass and gravity changes
	TargetPointTime{//launch on a trajectory that will land at a target point in a set amount of time
		target_point:Planar64Vec3,
		time:Time,//short time = fast and direct, long time = launch high in the air, negative time = wrong way
	},
	TrajectoryTargetPoint{//launch at a fixed speed and land at a target point
		target_point:Planar64Vec3,
		speed:Planar64,//if speed is too low this will fail to reach the target.  The closest-passing trajectory will be chosen instead
		trajectory_choice:TrajectoryChoice,
	},
	Velocity(Planar64Vec3),//SetVelocity
	DotVelocity{direction:Planar64Vec3,dot:Planar64},//set your velocity in a specific direction without touching other directions
}
#[derive(Clone)]
pub enum ZoneBehaviour{
	//Start is indexed
	//Checkpoints are indexed
	Finish,
	Anitcheat,
}
#[derive(Clone)]
pub struct GameMechanicZone{
	pub mode_id:u32,
	pub behaviour:ZoneBehaviour,
}
// enum TrapCondition{
// 	FasterThan(Planar64),
// 	SlowerThan(Planar64),
// 	InRange(Planar64,Planar64),
// 	OutsideRange(Planar64,Planar64),
// }
#[derive(Clone)]
pub enum StageElementBehaviour{
 	//Spawn,//The behaviour of stepping on a spawn setting the spawnid
 	SpawnAt,
 	Trigger,
 	Teleport,
 	Platform,
 	//Acts like a trigger if you haven't hit all the checkpoints.
 	Checkpoint{
 		//if this is 2 you must have hit OrderedCheckpoint(0) OrderedCheckpoint(1) OrderedCheckpoint(2) to pass
 		ordered_checkpoint_id:Option<u32>,
 		//if this is 2 you must have hit at least 2 UnorderedCheckpoints to pass
 		unordered_checkpoint_count:u32,
 	},
 	JumpLimit(u32),
 	//Speedtrap(TrapCondition),//Acts as a trigger with a speed condition
}
#[derive(Clone)]
pub struct GameMechanicStageElement{
	pub mode_id:u32,
	pub stage_id:u32,//which spawn to send to
	pub force:bool,//allow setting to lower spawn id i.e. 7->3
	pub behaviour:StageElementBehaviour
}
#[derive(Clone)]
pub struct GameMechanicWormhole{
	//destination does not need to be another wormhole
	//this defines a one way portal to a destination model transform
	//two of these can create a two way wormhole
	pub destination_model_id:u32,
	//(position,angles)*=origin.transform.inverse()*destination.transform
}
#[derive(Clone)]
pub enum TeleportBehaviour{
	StageElement(GameMechanicStageElement),
	Wormhole(GameMechanicWormhole),
}
//attributes listed in order of handling
#[derive(Default,Clone)]
pub struct GameMechanicAttributes{
	pub zone:Option<GameMechanicZone>,
	pub booster:Option<GameMechanicBooster>,
	pub checkpoint:Option<GameMechanicCheckpoint>,
	pub trajectory:Option<GameMechanicSetTrajectory>,
	pub teleport_behaviour:Option<TeleportBehaviour>,
	pub accelerator:Option<GameMechanicAccelerator>,
}
impl GameMechanicAttributes{
	pub fn any(&self)->bool{
		self.zone.is_some()
		||self.booster.is_some()
		||self.checkpoint.is_some()
		||self.trajectory.is_some()
		||self.teleport_behaviour.is_some()
		||self.accelerator.is_some()
	}
}
#[derive(Default,Clone)]
pub struct ContactingAttributes{
	//friction?
	pub contact_behaviour:Option<ContactingBehaviour>,
}
impl ContactingAttributes{
	pub fn any(&self)->bool{
		self.contact_behaviour.is_some()
	}
}
#[derive(Default,Clone)]
pub struct IntersectingAttributes{
	pub water:Option<IntersectingWater>,
}
impl IntersectingAttributes{
	pub fn any(&self)->bool{
		self.water.is_some()
	}
}
//Spawn(u32) NO! spawns are indexed in the map header instead of marked with attibutes
//TODO: deduplicate attributes
pub enum CollisionAttributes{
	Decoration,//visual only
	Contact{//track whether you are contacting the object
		contacting:ContactingAttributes,
		general:GameMechanicAttributes,
	},
	Intersect{//track whether you are intersecting the object
		intersecting:IntersectingAttributes,
		general:GameMechanicAttributes,
	},
}
impl std::default::Default for CollisionAttributes{
	fn default() -> Self {
		Self::Contact{
			contacting:ContactingAttributes::default(),
			general:GameMechanicAttributes::default()
		}
	}
}

pub fn generate_indexed_model_list_from_obj(data:obj::ObjData,color:Color4)->Vec<IndexedModel>{
	let mut unique_vertex_index = std::collections::HashMap::<obj::IndexTuple,u32>::new();
	return data.objects.iter().map(|object|{
		unique_vertex_index.clear();
		let mut unique_vertices = Vec::new();
		let groups = object.groups.iter().map(|group|{
			IndexedGroup{
				texture:None,
				polys:group.polys.iter().map(|poly|{
					IndexedPolygon{
						vertices:poly.0.iter().map(|&tup|{
							if let Some(&i)=unique_vertex_index.get(&tup){
								i
							}else{
								let i=unique_vertices.len() as u32;
								unique_vertices.push(IndexedVertex{
									pos: tup.0 as u32,
									tex: tup.1.unwrap() as u32,
									normal: tup.2.unwrap() as u32,
									color: 0,
								});
								unique_vertex_index.insert(tup,i);
								i
							}
						}).collect()
					}
				}).collect()
			}
		}).collect();
		IndexedModel{
			unique_pos: data.position.iter().map(|&v|Planar64Vec3::try_from(v).unwrap()).collect(),
			unique_tex: data.texture.iter().map(|&v|TextureCoordinate::from_array(v)).collect(),
			unique_normal: data.normal.iter().map(|&v|Planar64Vec3::try_from(v).unwrap()).collect(),
			unique_color: vec![color],
			unique_vertices,
			groups,
			instances:Vec::new(),
		}
	}).collect()
}

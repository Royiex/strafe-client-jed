use crate::integer::{Planar64,Planar64Vec3,Planar64Affine3};
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
	pub start:u32,//start=model_id
	pub spawns:Vec<u32>,//spawns[spawn_id]=model_id
	pub ordered_checkpoints:Vec<u32>,//ordered_checkpoints[checkpoint_id]=model_id
	pub unordered_checkpoints:Vec<u32>,//unordered_checkpoints[checkpoint_id]=model_id
	pub spawn_from_stage_id:std::collections::HashMap::<u32,usize>,
	pub ordered_checkpoint_from_checkpoint_id:std::collections::HashMap::<u32,usize>,
}
impl ModeDescription{
	pub fn get_spawn_model_id(&self,stage_id:u32)->Option<&u32>{
		if let Some(&spawn)=self.spawn_from_stage_id.get(&stage_id){
			self.spawns.get(spawn)
		}else{
			None
		}
	}
	pub fn get_ordered_checkpoint_model_id(&self,checkpoint_id:u32)->Option<&u32>{
		if let Some(&checkpoint)=self.ordered_checkpoint_from_checkpoint_id.get(&checkpoint_id){
			self.ordered_checkpoints.get(checkpoint)
		}else{
			None
		}
	}
}
pub enum TempIndexedAttributes{
	Start{
		mode_id:u32,
	},
	Spawn{
		mode_id:u32,
		stage_id:u32,
	},
	OrderedCheckpoint{
		mode_id:u32,
		checkpoint_id:u32,
	},
	UnorderedCheckpoint{
		mode_id:u32,
	},
}

//you have this effect while in contact
#[derive(Clone)]
pub struct ContactingSurf{}
#[derive(Clone)]
pub struct ContactingLadder{
	pub sticky:bool
}
//you have this effect while intersecting
#[derive(Clone)]
pub struct IntersectingWater{
	pub viscosity:Planar64,
	pub density:Planar64,
	pub current:Planar64Vec3,
}
#[derive(Clone)]
pub struct IntersectingAccelerator{
	pub acceleration:Planar64Vec3
}
//All models can be given these attributes
#[derive(Clone)]
pub struct GameMechanicJumpLimit{
	pub count:u32,
}
#[derive(Clone)]
pub struct GameMechanicBooster{
	pub velocity:Planar64Vec3,
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
// 	FasterThan(i64),
// 	SlowerThan(i64),
// 	InRange(i64,i64),
// 	OutsideRange(i64,i64),
// }
#[derive(Clone)]
pub enum StageElementBehaviour{
 	//Spawn,//The behaviour of stepping on a spawn setting the spawnid
 	SpawnAt,
 	Trigger,
 	Teleport,
 	Platform,
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
#[derive(Default,Clone)]
pub struct GameMechanicAttributes{
	pub jump_limit:Option<GameMechanicJumpLimit>,
	pub booster:Option<GameMechanicBooster>,
	pub zone:Option<GameMechanicZone>,
	pub teleport_behaviour:Option<TeleportBehaviour>,
}
#[derive(Default,Clone)]
pub struct ContactingAttributes{
	pub elasticity:Option<u32>,//[1/2^32,1] 0=None (elasticity+1)/2^32
	//friction?
	pub surf:Option<ContactingSurf>,
	pub ladder:Option<ContactingLadder>,
}
#[derive(Default,Clone)]
pub struct IntersectingAttributes{
	pub water:Option<IntersectingWater>,
	pub accelerator:Option<IntersectingAccelerator>,
}
//Spawn(u32) NO! spawns are indexed in the map header instead of marked with attibutes
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

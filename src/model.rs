use bytemuck::{Pod, Zeroable};
#[derive(Clone, Copy, Pod, Zeroable)]
#[repr(C)]
pub struct Vertex {
	pub pos: [f32; 3],
	pub tex: [f32; 2],
	pub normal: [f32; 3],
	pub color: [f32; 4],
}
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
	pub unique_pos:Vec<[f32; 3]>,
	pub unique_tex:Vec<[f32; 2]>,
	pub unique_normal:Vec<[f32; 3]>,
	pub unique_color:Vec<[f32; 4]>,
	pub unique_vertices:Vec<IndexedVertex>,
	pub groups: Vec<IndexedGroup>,
	pub instances:Vec<ModelInstance>,
}
pub struct IndexedGroupFixedTexture{
	pub polys:Vec<IndexedPolygon>,
}
pub struct IndexedModelSingleTexture{
	pub unique_pos:Vec<[f32; 3]>,
	pub unique_tex:Vec<[f32; 2]>,
	pub unique_normal:Vec<[f32; 3]>,
	pub unique_color:Vec<[f32; 4]>,
	pub unique_vertices:Vec<IndexedVertex>,
	pub texture:Option<u32>,//RenderPattern? material/texture/shader/flat color
	pub groups: Vec<IndexedGroupFixedTexture>,
	pub instances:Vec<ModelGraphicsInstance>,
}
pub struct ModelSingleTexture{
	pub instances: Vec<ModelGraphicsInstance>,
	pub vertices: Vec<Vertex>,
	pub entities: Vec<Vec<u16>>,
	pub texture: Option<u32>,
}
#[derive(Clone)]
pub struct ModelGraphicsInstance{
	pub transform:glam::Mat4,
	pub normal_transform:glam::Mat4,
	pub color:glam::Vec4,
}
pub struct ModelInstance{
	pub transform:glam::Affine3A,
	pub color:glam::Vec4,
}
pub struct IndexedModelInstances{
	pub textures:Vec<String>,//RenderPattern
	pub models:Vec<IndexedModel>,
	//object_index for spawns, triggers etc?
	pub spawn_point:glam::Vec3,
}
//stage description referencing flattened ids is spooky, but the map loading is meant to be deterministic.
pub struct StageDescription{
	pub start:u32,//start=model_id
	pub spawns:Vec<u32>,//spawns[spawn_id]=model_id
	pub ordered_checkpoints:Vec<u32>,//ordered_checkpoints[checkpoint_id]=model_id
	pub unordered_checkpoints:Vec<u32>,//unordered_checkpoints[checkpoint_id]=model_id
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
	pub viscosity:i64,
	pub density:i64,
	pub current:glam::Vec3,
}
#[derive(Clone)]
pub struct IntersectingAccelerator{
	pub acceleration:glam::Vec3
}
//All models can be given these attributes
#[derive(Clone)]
pub struct GameMechanicJumpLimit{
	pub count:u32,
}
#[derive(Clone)]
pub struct GameMechanicBooster{
	pub velocity:glam::Vec3,
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
pub struct GameMechanicWormhole{//(position,angles)*=origin.transform.inverse()*destination.transform
	pub model_id:u32,
}
#[derive(Default,Clone)]
pub struct GameMechanicAttributes{
	pub jump_limit:Option<GameMechanicJumpLimit>,
	pub booster:Option<GameMechanicBooster>,
	pub zone:Option<GameMechanicZone>,
	pub stage_element:Option<GameMechanicStageElement>,
	pub wormhole:Option<GameMechanicWormhole>,//stage_element and wormhole are in conflict
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

pub fn generate_indexed_model_list_from_obj(data:obj::ObjData,color:[f32;4]) -> Vec<IndexedModel>{
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
			unique_pos: data.position.clone(),
			unique_tex: data.texture.clone(),
			unique_normal: data.normal.clone(),
			unique_color: vec![color],
			unique_vertices,
			groups,
			instances:Vec::new(),
		}
	}).collect()
}

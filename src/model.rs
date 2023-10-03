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

//you have this effect while in contact
struct ContactingSurf{}
struct ContactingLadder{
	sticky:bool
}
//you have this effect while intersecting
struct IntersectingWater{
	viscosity:i64,
	density:i64,
}
struct IntersectingAccelerator{
	acceleration:glam::I64Vec3
}
//All models can be given these attributes
struct GameMechanicJumpLimit{
	count:u32,
}
struct GameMechanicBooster{
	velocity:glam::I64Vec3,
}
enum ZoneBehaviour{
	//Start is indexed
	//Checkpoints are indexed
	Finish,
	Anitcheat,
}
struct GameMechanicZone{
	mode_id:u32,
	behaviour:ZoneBehaviour
}
// enum TrapCondition{
// 	FasterThan(i64),
// 	SlowerThan(i64),
// 	InRange(i64,i64),
// 	OutsideRange(i64,i64),
// }
enum StageElementBehaviour{
	SpawnAt,
	Trigger,
	Teleport,
	Platform,
	//Speedtrap(TrapCondition),//Acts as a trigger with a speed condition
}
struct GameMechanicStageElement{
	mode_id:u32,
	stage_id:u32,//which spawn to send to
	force:bool,//allow setting to lower spawn id i.e. 7->3
	behaviour:StageElementBehaviour
}
struct GameMechanicWormhole{//(position,angles)*=origin.transform.inverse()*destination.transform
	model_id:u32,
}
struct GameMechanicAttributes{
	jump_limit:Option<GameMechanicJumpLimit>,
	booster:Option<GameMechanicBooster>,
	zone:Option<GameMechanicZone>,
	stage_element:Option<GameMechanicStageElement>,
	wormhole:Option<GameMechanicWormhole>,
}
struct ContactingAttributes{
	surf:Option<ContactingSurf>,
	ladder:Option<ContactingLadder>,
}
struct IntersectingAttibutes{
	water:Option<IntersectingWater>,
	accelerator:Option<IntersectingAccelerator>,
}
//Spawn(u32) NO! spawns are indexed in the map header instead of marked with attibutes
pub enum CollisionAttributes{
	Decoration,//visual only
	Contact{//track whether you are contacting the object
		contacting:ContactingAttributes,
		general:GameMechanicAttributes,
	},
	Intersect{//track whether you are intersecting the object
		intersecting:IntersectingAttibutes,
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

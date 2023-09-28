use bytemuck::{Pod, Zeroable};
#[derive(Clone, Copy, Pod, Zeroable)]
#[repr(C)]
pub struct Vertex {
	pub pos: [f32; 3],
	pub texture: [f32; 2],
	pub normal: [f32; 3],
	pub color: [f32; 4],
}
#[derive(Hash)]
pub struct IndexedVertex{
	pub pos:u32,
	pub texture:u32,
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
	pub unique_texture:Vec<[f32; 2]>,
	pub unique_normal:Vec<[f32; 3]>,
	pub unique_color:Vec<[f32; 4]>,
	pub unique_vertices:Vec<IndexedVertex>,
	pub groups: Vec<IndexedGroup>,
}
#[derive(Clone)]
pub struct ModelInstance{
	pub model:u32,
	pub model_transform:glam::Affine3A,
	pub color:glam::Vec4,
}
pub struct IndexedModelInstances{
	pub textures:Vec<String>,//RenderPattern
	pub models:Vec<IndexedModel>,
	pub instances:Vec<ModelInstance>,
	//object_index for spawns, triggers etc?
}

pub fn generate_indexed_model_from_obj(data:obj::ObjData,color:[f32;4]) -> Vec<IndexedModel>{
	let mut unique_vertex_index = std::collections::HashMap::<obj::IndexTuple,u32>::new();
	return data.objects.iter().map(|&object|{
		unique_vertex_index.clear();
		let mut unique_vertices = Vec::new();
		let groups = object.groups.iter().map(|&group|{
			IndexedGroup{
				texture:None,
				polys:group.polys.iter().map(|&poly|{
					IndexedPolygon{
						vertices:poly.0.iter().map(|&tup|{
							if let Some(&i)=unique_vertex_index.get(&tup){
								i
							}else{
								let i=unique_vertices.len() as u32;
								unique_vertices.push(IndexedVertex{
									pos: tup.0 as u32,
									texture: tup.1.unwrap() as u32,
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
			unique_pos: data.position,
			unique_texture: data.texture,
			unique_normal: data.normal,
			unique_color: vec![color],
			unique_vertices,
			groups,
		}
	}).collect()
}
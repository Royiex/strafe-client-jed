use bytemuck::{Pod, Zeroable};
#[derive(Clone, Copy, Pod, Zeroable)]
#[repr(C)]
pub struct Vertex {
	pub pos: [f32; 3],
	pub texture: [f32; 2],
	pub normal: [f32; 3],
	pub color: [f32; 4],
}
#[derive(Clone)]
pub struct ModelInstance {
	pub model_transform: glam::Affine3A,
	pub color: glam::Vec4,
}

#[derive(Clone)]
pub struct ModelData {
	pub instances: Vec<ModelInstance>,
	pub vertices: Vec<Vertex>,
	pub entities: Vec<Vec<u16>>,
	pub texture: Option<u32>,
}

impl ModelData {
	pub const COLOR_FLOATS_WHITE: [f32;4] = [1.0,1.0,1.0,1.0];
	pub const COLOR_VEC4_WHITE: glam::Vec4 = glam::vec4(1.0,1.0,1.0,1.0);
}

pub fn generate_modeldatas(data:obj::ObjData,color:[f32;4]) -> Vec<ModelData>{
	let mut modeldatas=Vec::new();
	let mut vertices = Vec::new();
	let mut vertex_index = std::collections::HashMap::<obj::IndexTuple,u16>::new();
	for object in data.objects {
		vertices.clear();
		vertex_index.clear();
		let mut entities = Vec::new();
		for group in object.groups {
			let mut indices = Vec::new();
			for poly in group.polys {
				for end_index in 2..poly.0.len() {
					for &index in &[0, end_index - 1, end_index] {
						let vert = poly.0[index];
						if let Some(&i)=vertex_index.get(&vert){
							indices.push(i);
						}else{
							let i=vertices.len() as u16;
							vertices.push(Vertex {
								pos: data.position[vert.0],
								texture: data.texture[vert.1.unwrap()],
								normal: data.normal[vert.2.unwrap()],
								color,
							});
							vertex_index.insert(vert,i);
							indices.push(i);
						}
					}
				}
			}
			entities.push(indices);
		}
		modeldatas.push(ModelData {
			instances: Vec::new(),
			vertices:vertices.clone(),
			entities,
			texture: None,
		});
	}
	modeldatas
}
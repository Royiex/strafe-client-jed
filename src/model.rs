use bytemuck::{Pod, Zeroable};
#[derive(Clone, Copy, Pod, Zeroable)]
#[repr(C)]
pub struct Vertex {
	pub pos: [f32; 3],
	pub texture: [f32; 2],
	pub normal: [f32; 3],
	pub color: [f32; 4],
}

pub struct ModelInstance {
	pub transform: glam::Mat4,
	pub color: glam::Vec4,
}

pub struct ModelData {
	pub instances: Vec<ModelInstance>,
	pub vertices: Vec<Vertex>,
	pub entities: Vec<Vec<u16>>,
}

impl ModelData {
	pub const COLOR_FLOATS_WHITE: [f32;4] = [1.0,1.0,1.0,1.0];
	pub const COLOR_VEC4_WHITE: glam::Vec4 = glam::vec4(1.0,1.0,1.0,1.0);
}
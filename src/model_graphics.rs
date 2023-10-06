use bytemuck::{Pod, Zeroable};
use crate::model::{IndexedVertex,IndexedPolygon};
#[derive(Clone, Copy, Pod, Zeroable)]
#[repr(C)]
pub struct GraphicsVertex {
	pub pos: [f32; 3],
	pub tex: [f32; 2],
	pub normal: [f32; 3],
	pub color: [f32; 4],
}
pub struct IndexedGroupFixedTexture{
	pub polys:Vec<IndexedPolygon>,
}
pub struct IndexedModelGraphicsSingleTexture{
	pub unique_pos:Vec<[f32; 3]>,
	pub unique_tex:Vec<[f32; 2]>,
	pub unique_normal:Vec<[f32; 3]>,
	pub unique_color:Vec<[f32; 4]>,
	pub unique_vertices:Vec<IndexedVertex>,
	pub texture:Option<u32>,//RenderPattern? material/texture/shader/flat color
	pub groups: Vec<IndexedGroupFixedTexture>,
	pub instances:Vec<ModelGraphicsInstance>,
}
pub struct ModelGraphicsSingleTexture{
	pub instances: Vec<ModelGraphicsInstance>,
	pub vertices: Vec<GraphicsVertex>,
	pub entities: Vec<Vec<u16>>,
	pub texture: Option<u32>,
}
#[derive(Clone,PartialEq)]
pub struct ModelGraphicsColor4(glam::Vec4);
impl ModelGraphicsColor4{
	pub const fn get(&self)->glam::Vec4{
		self.0
	}
}
impl From<glam::Vec4> for ModelGraphicsColor4{
	fn from(value:glam::Vec4)->Self{
		Self(value)
	}
}
impl std::hash::Hash for ModelGraphicsColor4{
	fn hash<H: std::hash::Hasher>(&self,state:&mut H) {
		for &f in self.0.as_ref(){
			u32::from_ne_bytes(f.to_ne_bytes()).hash(state);
		}
	}
}
impl Eq for ModelGraphicsColor4{}
#[derive(Clone)]
pub struct ModelGraphicsInstance{
	pub transform:glam::Mat4,
	pub normal_transform:glam::Mat3,
	pub color:ModelGraphicsColor4,
}
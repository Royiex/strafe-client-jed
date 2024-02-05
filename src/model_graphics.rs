use bytemuck::{Pod, Zeroable};
use strafesnet_common::model::{IndexedVertex,IndexedVertexList};
#[derive(Clone,Copy,Pod,Zeroable)]
#[repr(C)]
pub struct GraphicsVertex{
	pub pos:[f32;3],
	pub tex:[f32;2],
	pub normal:[f32;3],
	pub color:[f32;4],
}
pub struct IndexedGroupFixedTexture{
	pub polys:Vec<IndexedVertexList>,
}
pub struct IndexedGraphicsModelSingleTexture{
	pub unique_pos:Vec<[f32;3]>,
	pub unique_tex:Vec<[f32;2]>,
	pub unique_normal:Vec<[f32;3]>,
	pub unique_color:Vec<[f32;4]>,
	pub unique_vertices:Vec<IndexedVertex>,
	pub texture:Option<u32>,//RenderPattern? material/texture/shader/flat color
	pub groups:Vec<IndexedGroupFixedTexture>,
	pub instances:Vec<GraphicsModelInstance>,
}
pub enum Entities{
	U32(Vec<Vec<u32>>),
	U16(Vec<Vec<u16>>),
}
pub struct GraphicsModelSingleTexture{
	pub instances:Vec<GraphicsModelInstance>,
	pub vertices:Vec<GraphicsVertex>,
	pub entities:Entities,
	pub texture:Option<u32>,
}
#[derive(Clone,PartialEq)]
pub struct GraphicsModelColor4(glam::Vec4);
impl GraphicsModelColor4{
	pub const fn get(&self)->glam::Vec4{
		self.0
	}
}
impl From<glam::Vec4> for GraphicsModelColor4{
	fn from(value:glam::Vec4)->Self{
		Self(value)
	}
}
impl std::hash::Hash for GraphicsModelColor4{
	fn hash<H: std::hash::Hasher>(&self,state:&mut H) {
		for &f in self.0.as_ref(){
			bytemuck::cast::<f32,u32>(f).hash(state);
		}
	}
}
impl Eq for GraphicsModelColor4{}
#[derive(Clone)]
pub struct GraphicsModelInstance{
	pub transform:glam::Mat4,
	pub normal_transform:glam::Mat3,
	pub color:GraphicsModelColor4,
}

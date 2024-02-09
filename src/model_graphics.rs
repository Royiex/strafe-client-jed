use bytemuck::{Pod,Zeroable};
use strafesnet_common::model::{IndexedVertex,PolygonGroup,RenderConfigId};
#[derive(Clone,Copy,Pod,Zeroable)]
#[repr(C)]
pub struct GraphicsVertex{
	pub pos:[f32;3],
	pub tex:[f32;2],
	pub normal:[f32;3],
	pub color:[f32;4],
}
#[derive(Clone,Copy,id::Id)]
pub struct IndexedGraphicsMeshOwnedRenderConfigId(u32);
pub struct IndexedGraphicsMeshOwnedRenderConfig{
	pub unique_pos:Vec<[f32;3]>,
	pub unique_tex:Vec<[f32;2]>,
	pub unique_normal:Vec<[f32;3]>,
	pub unique_color:Vec<[f32;4]>,
	pub unique_vertices:Vec<IndexedVertex>,
	pub render_config:RenderConfigId,
	pub polys:PolygonGroup,
	pub instances:Vec<GraphicsModelOwned>,
}
pub enum Indices{
	U32(Vec<u32>),
	U16(Vec<u16>),
}
pub struct GraphicsMeshOwnedRenderConfig{
	pub vertices:Vec<GraphicsVertex>,
	pub indices:Indices,
	pub render_config:RenderConfigId,
	pub instances:Vec<GraphicsModelOwned>,
}
#[derive(Clone,Copy,PartialEq,id::Id)]
pub struct GraphicsModelColor4(glam::Vec4);
impl std::hash::Hash for GraphicsModelColor4{
	fn hash<H:std::hash::Hasher>(&self,state:&mut H) {
		for &f in self.0.as_ref(){
			bytemuck::cast::<f32,u32>(f).hash(state);
		}
	}
}
impl Eq for GraphicsModelColor4{}
#[derive(Clone)]
pub struct GraphicsModelOwned{
	pub transform:glam::Mat4,
	pub normal_transform:glam::Mat3,
	pub color:GraphicsModelColor4,
}

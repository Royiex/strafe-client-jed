use crate::model::{ModelData,ModelInstance};

use crate::primitives;

fn class_is_a(class: &str, superclass: &str) -> bool {
	if class==superclass {
		return true
	}
	let class_descriptor=rbx_reflection_database::get().classes.get(class);
	if let Some(descriptor) = &class_descriptor {
		if let Some(class_super) = &descriptor.superclass {
			return class_is_a(&class_super, superclass)
		}
	}
	return false
}
fn recursive_collect_superclass(objects: &mut std::vec::Vec<rbx_dom_weak::types::Ref>,dom: &rbx_dom_weak::WeakDom, instance: &rbx_dom_weak::Instance, superclass: &str){
	for &referent in instance.children() {
		if let Some(c) = dom.get_by_ref(referent) {
			if class_is_a(c.class.as_str(), superclass) {
				objects.push(c.referent());//copy ref
			}
			recursive_collect_superclass(objects,dom,c,superclass);
		}
	}
}
fn get_texture_refs(dom:&rbx_dom_weak::WeakDom) -> Vec<rbx_dom_weak::types::Ref>{
	let mut objects = std::vec::Vec::new();
	recursive_collect_superclass(&mut objects, dom, dom.root(),"Decal");
	//get ids
	//clear vec
	//next class
	objects
}

struct RobloxAssetId(u64);
struct RobloxAssetIdParseErr;
impl std::str::FromStr for RobloxAssetId {
	type Err=RobloxAssetIdParseErr;
	fn from_str(s: &str) -> Result<Self, Self::Err>{
		let regman=regex::Regex::new(r"(\d+)$").unwrap();
		if let Some(captures) = regman.captures(s) {
			if captures.len()==2{//captures[0] is all captures concatenated, and then each individual capture
				if let Ok(id) = captures[0].parse::<u64>() {
					return Ok(Self(id));
				}
			}
		}
		Err(RobloxAssetIdParseErr)
	}
}
#[derive(Clone,Copy,PartialEq)]
struct RobloxTextureTransform{
	offset_u:f32,
	offset_v:f32,
	scale_u:f32,
	scale_v:f32,
}
impl std::cmp::Eq for RobloxTextureTransform{}//????
impl std::default::Default for RobloxTextureTransform{
    fn default() -> Self {
        Self{offset_u:0.0,offset_v:0.0,scale_u:1.0,scale_v:1.0}
    }
}
impl std::hash::Hash for RobloxTextureTransform {
	fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
		self.offset_u.to_ne_bytes().hash(state);
		self.offset_v.to_ne_bytes().hash(state);
		self.scale_u.to_ne_bytes().hash(state);
		self.scale_v.to_ne_bytes().hash(state);
	}
}
#[derive(Hash)]
struct PartFaceTextureDescription{
	texture:u32,
	transform:RobloxTextureTransform,
}
type PartTextureDescription=[Option<PartFaceTextureDescription>;6];
#[derive(Hash,Eq,PartialEq)]
struct RobloxUnitCubeGenerationData{
	texture:Option<u32>,
	faces:[Option<RobloxTextureTransform>;6],
}
impl std::default::Default for RobloxUnitCubeGenerationData{
    fn default() -> Self {
    	Self{
			texture:None,
			faces:[Some(RobloxTextureTransform::default());6],
		}
    }
}
impl RobloxUnitCubeGenerationData{
    fn empty() -> Self {
    	Self{
			texture:None,
			faces:[None,None,None,None,None,None],
		}
    }
}
pub fn generate_modeldatas_roblox(dom:rbx_dom_weak::WeakDom) -> Result<(Vec<ModelData>,Vec<String>,glam::Vec3), Box<dyn std::error::Error>>{
	//ModelData includes texture dds
	let mut spawn_point=glam::Vec3::ZERO;

	//TODO: generate unit Block, Wedge, etc. after based on part shape lists
	let mut modeldatas=Vec::new();

	let mut texture_id_from_asset_id=std::collections::HashMap::<u64,u32>::new();
	let mut asset_id_from_texture_id=Vec::new();

	let mut object_refs=Vec::new();
	let mut temp_objects=Vec::new();
	let mut model_id_from_ucgd=std::collections::HashMap::<RobloxUnitCubeGenerationData,usize>::new();
	recursive_collect_superclass(&mut object_refs, &dom, dom.root(),"BasePart");
	for object_ref in object_refs {
		if let Some(object)=dom.get_by_ref(object_ref){
			if let (
					Some(rbx_dom_weak::types::Variant::CFrame(cf)),
					Some(rbx_dom_weak::types::Variant::Vector3(size)),
					Some(rbx_dom_weak::types::Variant::Float32(transparency)),
					Some(rbx_dom_weak::types::Variant::Color3uint8(color3)),
					Some(rbx_dom_weak::types::Variant::Enum(shape)),
				) = (
					object.properties.get("CFrame"),
					object.properties.get("Size"),
					object.properties.get("Transparency"),
					object.properties.get("Color"),
					object.properties.get("Shape"),//this will also skip unions
				)
			{
				let model_transform=glam::Affine3A::from_translation(
					glam::Vec3::new(cf.position.x,cf.position.y,cf.position.z)
				)
				* glam::Affine3A::from_mat3(
					glam::Mat3::from_cols(
						glam::Vec3::new(cf.orientation.x.x,cf.orientation.y.x,cf.orientation.z.x),
						glam::Vec3::new(cf.orientation.x.y,cf.orientation.y.y,cf.orientation.z.y),
						glam::Vec3::new(cf.orientation.x.z,cf.orientation.y.z,cf.orientation.z.z),
					),
				)
				* glam::Affine3A::from_scale(
					glam::Vec3::new(size.x,size.y,size.z)/2.0
				);
				if object.name=="MapStart"{
					spawn_point=model_transform.transform_point3(glam::Vec3::Y)+glam::vec3(0.0,2.5,0.0);
					println!("Found MapStart{:?}",spawn_point);
				}
				if *transparency==1.0||shape.to_u32()!=1 {
					continue;
				}
				temp_objects.clear();
				recursive_collect_superclass(&mut temp_objects, &dom, object,"Decal");

				let mut part_texture_description:PartTextureDescription=[None,None,None,None,None,None];
				for &decal_ref in &temp_objects{
					if let Some(decal)=dom.get_by_ref(decal_ref){
						if let (
							Some(rbx_dom_weak::types::Variant::Content(content)),
							Some(rbx_dom_weak::types::Variant::Enum(normalid)),
						) = (
							decal.properties.get("Texture"),
							decal.properties.get("Face"),
						) {
							if let Ok(asset_id)=content.clone().into_string().parse::<RobloxAssetId>(){
								let texture_id=if let Some(&texture_id)=texture_id_from_asset_id.get(&asset_id.0){
									texture_id
								}else{
									let texture_id=asset_id_from_texture_id.len() as u32;
									texture_id_from_asset_id.insert(asset_id.0,texture_id);
									asset_id_from_texture_id.push(asset_id.0);
									texture_id
								};
								let face=normalid.to_u32();
								if face<6{
									let mut roblox_texture_transform=RobloxTextureTransform::default();
									if decal.class=="Texture"{
										//generate tranform
										if let (
												Some(rbx_dom_weak::types::Variant::Float32(ox)),
												Some(rbx_dom_weak::types::Variant::Float32(oy)),
												Some(rbx_dom_weak::types::Variant::Float32(sx)),
												Some(rbx_dom_weak::types::Variant::Float32(sy)),
											) = (
												decal.properties.get("OffsetStudsU"),
												decal.properties.get("OffsetStudsV"),
												decal.properties.get("StudsPerTileU"),
												decal.properties.get("StudsPerTileV"),
											)
										{
											let (size_u,size_v)=match face{
												0=>(size.z,size.y),//right
												1=>(size.x,size.z),//top
												2=>(size.x,size.y),//back
												3=>(size.z,size.y),//left
												4=>(size.x,size.z),//bottom
												5=>(size.x,size.y),//front
												_=>(1.,1.),
											};
											roblox_texture_transform=RobloxTextureTransform{
												offset_u:*ox/(*sx),offset_v:*oy/(*sy),
												scale_u:size_u/(*sx),scale_v:size_v/(*sy),
											};
										}
									}
									//I can alos put the color into here and generate the vertices with the color
									part_texture_description[face as usize]=Some(PartFaceTextureDescription{
										texture:texture_id,
										transform:roblox_texture_transform,
									});
								}else{
									println!("goofy ahh roblox gave NormalId {}", face);
								}
							}
						}
					}
				}
				let mut unit_cube_generation_data_list=Vec::new();
				let mut unit_cube_from_texture_id=std::collections::HashMap::<u32,usize>::new();
				//use part_texture_description to extract unique texture faces
				let mut add_negative_cube=false;
				let mut negative_cube=RobloxUnitCubeGenerationData::empty();
				for (i,maybe_part_face) in part_texture_description.iter().enumerate(){
					if let Some(part_face)=maybe_part_face{
						let unit_cube_id=if let Some(&unit_cube_id)=unit_cube_from_texture_id.get(&part_face.texture){
							unit_cube_id
						}else{
							let unit_cube_id=unit_cube_generation_data_list.len();
							unit_cube_generation_data_list.push(RobloxUnitCubeGenerationData::empty());
							unit_cube_from_texture_id.insert(part_face.texture,unit_cube_id);
							unit_cube_generation_data_list[unit_cube_id].texture=Some(part_face.texture);
							unit_cube_id
						};
						unit_cube_generation_data_list[unit_cube_id].faces[i]=Some(part_face.transform);
					}else{
						add_negative_cube=true;
						negative_cube.faces[i]=Some(RobloxTextureTransform::default());
					}
				}
				//must add the rest of the cube to complete the faces!
				if add_negative_cube{
					unit_cube_generation_data_list.push(negative_cube);
				}
				for roblox_unit_cube_generation_data in unit_cube_generation_data_list.drain(..){
					//make new model if unit cube has not been crated before
					let model_id=if let Some(&model_id)=model_id_from_ucgd.get(&roblox_unit_cube_generation_data){
						//push to existing texture model
						model_id
					}else{
						let unit_cube_generation_data=roblox_unit_cube_generation_data.faces.map(|face|{
							match face{
								Some(roblox_texture_transform)=>Some(
									glam::Affine2::from_translation(
										glam::vec2(roblox_texture_transform.offset_u,roblox_texture_transform.offset_v)
									)
									*glam::Affine2::from_scale(
										glam::vec2(roblox_texture_transform.scale_u,roblox_texture_transform.scale_v)
									)
								),
								None=>None,
							}
						});
						let mut new_modeldatas=crate::model::generate_modeldatas(primitives::generate_partial_unit_cube(unit_cube_generation_data),ModelData::COLOR_FLOATS_WHITE);
						new_modeldatas[0].texture=roblox_unit_cube_generation_data.texture;
						let model_id=modeldatas.len();
						modeldatas.append(&mut new_modeldatas);
						model_id_from_ucgd.insert(roblox_unit_cube_generation_data,model_id);
						model_id
					};
					modeldatas[model_id].instances.push(ModelInstance {
						model_transform,
						color: glam::Vec4::ONE,//glam::vec4(color3.r as f32/255f32, color3.g as f32/255f32, color3.b as f32/255f32, 1.0-*transparency),
					});
				}
			}
		}
	}
	Ok((modeldatas,asset_id_from_texture_id.iter().map(|t|t.to_string()).collect(),spawn_point))
}

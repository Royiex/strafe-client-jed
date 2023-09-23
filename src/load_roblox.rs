use std::todo;

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
pub fn generate_modeldatas_roblox(dom:&rbx_dom_weak::WeakDom) -> Result<(Vec<ModelData>,Vec<std::fs::File>), Box<dyn std::error::Error>>{
	//ModelData includes texture dds

	//TODO: generate unit Block, Wedge, etc. after based on part shape lists
	let mut modeldatas=crate::model::generate_modeldatas(primitives::the_unit_cube_lol(),ModelData::COLOR_FLOATS_WHITE);
	let unit_cube_modeldata=modeldatas[0].clone();

	let mut texture_id_from_asset_id=std::collections::HashMap::<u64,u32>::new();
	let mut asset_id_from_texture_id=Vec::new();

	let mut object_refs = std::vec::Vec::new();
	let mut temp_objects = std::vec::Vec::new();
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
				if *transparency==1.0||shape.to_u32()!=1 {
					continue;
				}
				temp_objects.clear();
				recursive_collect_superclass(&mut temp_objects, &dom, object,"Decal");

				let mut i_can_only_load_one_texture_per_model=None;
				for &decal_ref in &temp_objects{
					if let Some(decal)=dom.get_by_ref(decal_ref){
						if let Some(rbx_dom_weak::types::Variant::Content(content)) = decal.properties.get("Texture") {
							if let Ok(asset_id)=content.clone().into_string().parse::<RobloxAssetId>(){
								if let Some(&texture_id)=texture_id_from_asset_id.get(&asset_id.0){
									i_can_only_load_one_texture_per_model=Some(texture_id);
								}else{
									let texture_id=asset_id_from_texture_id.len();
									texture_id_from_asset_id.insert(asset_id.0,texture_id as u32);
									asset_id_from_texture_id.push(asset_id.0);
									//make new model
									let mut unit_cube_texture=unit_cube_modeldata.clone();
									unit_cube_texture.texture=Some(texture_id as u32);
									modeldatas.push(unit_cube_texture);
								}
							}
						}
					}
				}
				let model_instance=ModelInstance {
						transform:glam::Mat4::from_translation(
								glam::Vec3::new(cf.position.x,cf.position.y,cf.position.z)
							)
							* glam::Mat4::from_mat3(
								glam::Mat3::from_cols(
									glam::Vec3::new(cf.orientation.x.x,cf.orientation.y.x,cf.orientation.z.x),
									glam::Vec3::new(cf.orientation.x.y,cf.orientation.y.y,cf.orientation.z.y),
									glam::Vec3::new(cf.orientation.x.z,cf.orientation.y.z,cf.orientation.z.z),
								),
							)
							* glam::Mat4::from_scale(
								glam::Vec3::new(size.x,size.y,size.z)/2.0
							),
						color: glam::vec4(color3.r as f32/255f32, color3.g as f32/255f32, color3.b as f32/255f32, 1.0-*transparency),
					};
				match i_can_only_load_one_texture_per_model{
					//push to existing texture model
					Some(texture_id)=>modeldatas[(texture_id+1) as usize].instances.push(model_instance),
					//push instance to big unit cube in the sky
					None=>modeldatas[0].instances.push(model_instance),
				}
			}
		}
	}
	let texture_files:Result<Vec<std::fs::File>,_>=asset_id_from_texture_id.iter().map(|asset_id|{
		std::fs::File::open(std::path::Path::new(&format!("textures/{}.dds",asset_id)))
	}).collect();
	Ok((modeldatas,texture_files?))
}

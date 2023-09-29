use crate::model::{IndexedModelInstances,ModelInstance};

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
#[derive(Clone,PartialEq)]
struct RobloxFaceTextureDescription{
	texture:u32,
	color:glam::Vec4,
	transform:RobloxTextureTransform,
}
impl std::cmp::Eq for RobloxFaceTextureDescription{}//????
impl std::hash::Hash for RobloxFaceTextureDescription {
	fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
		self.texture.hash(state);
		self.transform.hash(state);
        for &el in self.color.as_ref().iter() {
            el.to_ne_bytes().hash(state);
        }
    }
}
type RobloxPartDescription=[Option<RobloxFaceTextureDescription>;6];
//type RobloxWedgeDescription=[Option<RobloxFaceTextureDescription>;5];
#[derive(Clone,Eq,Hash,PartialEq)]
enum RobloxBasePartDescription{
	Part(RobloxPartDescription),
	//Wedge(RobloxWedgeDescription),
}
pub fn generate_indexed_models_roblox(dom:rbx_dom_weak::WeakDom) -> Result<(IndexedModelInstances,glam::Vec3), Box<dyn std::error::Error>>{
	//IndexedModelInstances includes textures
	let mut spawn_point=glam::Vec3::ZERO;

	let mut indexed_models=Vec::new();
	let mut model_id_from_description=std::collections::HashMap::<RobloxBasePartDescription,usize>::new();

	let mut texture_id_from_asset_id=std::collections::HashMap::<u64,u32>::new();
	let mut asset_id_from_texture_id=Vec::new();

	let mut object_refs=Vec::new();
	let mut temp_objects=Vec::new();
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

				let mut part_texture_description:RobloxPartDescription=[None,None,None,None,None,None];
				for &decal_ref in &temp_objects{
					if let Some(decal)=dom.get_by_ref(decal_ref){
						if let (
							Some(rbx_dom_weak::types::Variant::Content(content)),
							Some(rbx_dom_weak::types::Variant::Enum(normalid)),
							Some(rbx_dom_weak::types::Variant::Color3(decal_color3)),
							Some(rbx_dom_weak::types::Variant::Float32(decal_transparency)),
						) = (
							decal.properties.get("Texture"),
							decal.properties.get("Face"),
							decal.properties.get("Color3"),
							decal.properties.get("Transparency"),
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
									let mut roblox_texture_color=glam::Vec4::ONE;
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
											roblox_texture_color=glam::vec4(decal_color3.r,decal_color3.g,decal_color3.b,1.0-*decal_transparency);
										}
									}
									part_texture_description[face as usize]=Some(RobloxFaceTextureDescription{
										texture:texture_id,
										color:roblox_texture_color,
										transform:roblox_texture_transform,
									});
								}else{
									println!("goofy ahh roblox gave NormalId {}", face);
								}
							}
						}
					}
				}
				//TODO: generate unit Block, Wedge, etc. based on part shape lists
				let basepart_texture_description=RobloxBasePartDescription::Part(part_texture_description);
				//make new model if unit cube has not been crated before
				let model_id=if let Some(&model_id)=model_id_from_description.get(&basepart_texture_description){
					//push to existing texture model
					model_id
				}else{
					let model_id=indexed_models.len();
					model_id_from_description.insert(basepart_texture_description.clone(),model_id);//borrow checker going crazy
					match basepart_texture_description{
						RobloxBasePartDescription::Part(part_texture_description)=>{
							let unit_cube_faces=part_texture_description.map(|face|{
								match face{
									Some(roblox_texture_transform)=>Some(
										primitives::FaceDescription{
											texture:Some(roblox_texture_transform.texture),
											transform:glam::Affine2::from_translation(
												glam::vec2(roblox_texture_transform.transform.offset_u,roblox_texture_transform.transform.offset_v)
											)
											*glam::Affine2::from_scale(
												glam::vec2(roblox_texture_transform.transform.scale_u,roblox_texture_transform.transform.scale_v)
											),
											color:roblox_texture_transform.color,
										}
									),
									None=>Some(primitives::FaceDescription::default()),
								}
							});
							let indexed_model=primitives::generate_partial_unit_cube(unit_cube_faces);
							indexed_models.push(indexed_model);
							model_id
						},
					}
				};
				indexed_models[model_id].instances.push(ModelInstance {
					model_transform,
					color:glam::vec4(color3.r as f32/255f32, color3.g as f32/255f32, color3.b as f32/255f32, 1.0-*transparency),
				});
			}
		}
	}
	Ok((IndexedModelInstances{
		textures:asset_id_from_texture_id.iter().map(|t|t.to_string()).collect(),
		models:indexed_models,
	},spawn_point))
}

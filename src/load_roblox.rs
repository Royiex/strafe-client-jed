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
		let regman=lazy_regex::regex!(r"(\d+)$");
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
impl RobloxFaceTextureDescription{
	fn to_face_description(&self)->primitives::FaceDescription{
		primitives::FaceDescription{
			texture:Some(self.texture),
			transform:glam::Affine2::from_translation(
				glam::vec2(self.transform.offset_u,self.transform.offset_v)
			)
			*glam::Affine2::from_scale(
				glam::vec2(self.transform.scale_u,self.transform.scale_v)
			),
			color:self.color,
		}
	}
}
type RobloxPartDescription=[Option<RobloxFaceTextureDescription>;6];
type RobloxWedgeDescription=[Option<RobloxFaceTextureDescription>;5];
type RobloxCornerWedgeDescription=[Option<RobloxFaceTextureDescription>;4];
#[derive(Clone,Eq,Hash,PartialEq)]
enum RobloxBasePartDescription{
	Sphere,
	Part(RobloxPartDescription),
	Cylinder,
	Wedge(RobloxWedgeDescription),
	CornerWedge(RobloxCornerWedgeDescription),
}
pub fn generate_indexed_models(dom:rbx_dom_weak::WeakDom) -> crate::model::IndexedModelInstances{
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
				) = (
					object.properties.get("CFrame"),
					object.properties.get("Size"),
					object.properties.get("Transparency"),
					object.properties.get("Color"),
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
					spawn_point=model_transform.transform_point3(-glam::Vec3::Y)+glam::vec3(0.0,2.5+0.1,0.0);
					println!("Found MapStart{:?}",spawn_point);
				}

				//TODO: also detect "CylinderMesh" etc here
				let shape=match &object.class[..]{
					"Part"=>{
						if let Some(rbx_dom_weak::types::Variant::Enum(shape))=object.properties.get("Shape"){
							match shape.to_u32(){
								0=>primitives::Primitives::Sphere,
								1=>primitives::Primitives::Cube,
								2=>primitives::Primitives::Cylinder,
								3=>primitives::Primitives::Wedge,
								4=>primitives::Primitives::CornerWedge,
								_=>panic!("Funky roblox PartType={};",shape.to_u32()),
							}
						}else{
							panic!("Part has no Shape!");
						}
					},
					"WedgePart"=>primitives::Primitives::Wedge,
					"CornerWedgePart"=>primitives::Primitives::CornerWedge,
					_=>{
						println!("Unsupported BasePart ClassName={}; defaulting to cube",object.class);
						primitives::Primitives::Cube
					}
				};

				//use the biggest one and cut it down later...
				let mut part_texture_description:RobloxPartDescription=[None,None,None,None,None,None];
				temp_objects.clear();
				recursive_collect_superclass(&mut temp_objects, &dom, object,"Decal");
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
								let normal_id=normalid.to_u32();
								if normal_id<6{
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
											let (size_u,size_v)=match normal_id{
												0=>(size.z,size.y),//right
												1=>(size.x,size.z),//top
												2=>(size.x,size.y),//back
												3=>(size.z,size.y),//left
												4=>(size.x,size.z),//bottom
												5=>(size.x,size.y),//front
												_=>panic!("unreachable"),
											};
											roblox_texture_transform=RobloxTextureTransform{
												offset_u:*ox/(*sx),offset_v:*oy/(*sy),
												scale_u:size_u/(*sx),scale_v:size_v/(*sy),
											};
											roblox_texture_color=glam::vec4(decal_color3.r,decal_color3.g,decal_color3.b,1.0-*decal_transparency);
										}
									}
									part_texture_description[normal_id as usize]=Some(RobloxFaceTextureDescription{
										texture:texture_id,
										color:roblox_texture_color,
										transform:roblox_texture_transform,
									});
								}else{
									println!("NormalId={} unsupported for shape={:?}",normal_id,shape);
								}
							}
						}
					}
				}
				//obscure rust syntax "slice pattern"
				let [f0,f1,f2,f3,f4,f5]=part_texture_description;
				let basepart_texture_description=match shape{
					primitives::Primitives::Sphere=>RobloxBasePartDescription::Sphere,
					primitives::Primitives::Cube=>RobloxBasePartDescription::Part([f0,f1,f2,f3,f4,f5]),
					primitives::Primitives::Cylinder=>RobloxBasePartDescription::Cylinder,
					//use front face texture first and use top face texture as a fallback
					primitives::Primitives::Wedge=>RobloxBasePartDescription::Wedge([f0,if f2.is_some(){f2}else{f1},f3,f4,f5]),
					primitives::Primitives::CornerWedge=>RobloxBasePartDescription::CornerWedge([f1,f3,f4,f5]),
				};
				//make new model if unit cube has not been created before
				let model_id=if let Some(&model_id)=model_id_from_description.get(&basepart_texture_description){
					//push to existing texture model
					model_id
				}else{
					let model_id=indexed_models.len();
					model_id_from_description.insert(basepart_texture_description.clone(),model_id);//borrow checker going crazy
					indexed_models.push(match basepart_texture_description{
						RobloxBasePartDescription::Sphere=>primitives::unit_sphere(),
						RobloxBasePartDescription::Part(part_texture_description)=>{
							let mut cube_face_description=primitives::CubeFaceDescription::new();
							for (face_id,roblox_face_description) in part_texture_description.iter().enumerate(){
								cube_face_description.insert(
								match face_id{
									0=>primitives::CubeFace::Right,
									1=>primitives::CubeFace::Top,
									2=>primitives::CubeFace::Back,
									3=>primitives::CubeFace::Left,
									4=>primitives::CubeFace::Bottom,
									5=>primitives::CubeFace::Front,
									_=>panic!("unreachable"),
								},
								match roblox_face_description{
									Some(roblox_texture_transform)=>roblox_texture_transform.to_face_description(),
									None=>primitives::FaceDescription::default(),
								});
							}
							primitives::generate_partial_unit_cube(cube_face_description)
						},
						RobloxBasePartDescription::Cylinder=>primitives::unit_cylinder(),
						RobloxBasePartDescription::Wedge(wedge_texture_description)=>{
							let mut wedge_face_description=primitives::WedgeFaceDescription::new();
							for (face_id,roblox_face_description) in wedge_texture_description.iter().enumerate(){
								wedge_face_description.insert(
								match face_id{
									0=>primitives::WedgeFace::Right,
									1=>primitives::WedgeFace::TopFront,
									2=>primitives::WedgeFace::Back,
									3=>primitives::WedgeFace::Left,
									4=>primitives::WedgeFace::Bottom,
									_=>panic!("unreachable"),
								},
								match roblox_face_description{
									Some(roblox_texture_transform)=>roblox_texture_transform.to_face_description(),
									None=>primitives::FaceDescription::default(),
								});
							}
							primitives::generate_partial_unit_wedge(wedge_face_description)
						},
						RobloxBasePartDescription::CornerWedge(cornerwedge_texture_description)=>{
							let mut cornerwedge_face_description=primitives::CornerWedgeFaceDescription::new();
							for (face_id,roblox_face_description) in cornerwedge_texture_description.iter().enumerate(){
								cornerwedge_face_description.insert(
								match face_id{
									0=>primitives::CornerWedgeFace::Top,
									1=>primitives::CornerWedgeFace::Right,
									2=>primitives::CornerWedgeFace::Bottom,
									3=>primitives::CornerWedgeFace::Front,
									_=>panic!("unreachable"),
								},
								match roblox_face_description{
									Some(roblox_texture_transform)=>roblox_texture_transform.to_face_description(),
									None=>primitives::FaceDescription::default(),
								});
							}
							primitives::generate_partial_unit_cornerwedge(cornerwedge_face_description)
						},
					});
					model_id
				};
				indexed_models[model_id].instances.push(crate::model::ModelInstance {
					transform:model_transform,
					color:glam::vec4(color3.r as f32/255f32, color3.g as f32/255f32, color3.b as f32/255f32, 1.0-*transparency),
				});
			}
		}
	}
	crate::model::IndexedModelInstances{
		textures:asset_id_from_texture_id.iter().map(|t|t.to_string()).collect(),
		models:indexed_models,
		spawn_point,
	}
}

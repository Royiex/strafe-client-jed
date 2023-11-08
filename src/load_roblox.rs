use crate::primitives;
use crate::integer::{Planar64,Planar64Vec3,Planar64Mat3,Planar64Affine3};

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
fn planar64_affine3_from_roblox(cf:&rbx_dom_weak::types::CFrame,size:&rbx_dom_weak::types::Vector3)->Planar64Affine3{
	Planar64Affine3::new(
		Planar64Mat3::from_cols(
			Planar64Vec3::try_from([cf.orientation.x.x,cf.orientation.y.x,cf.orientation.z.x]).unwrap()
			*Planar64::try_from(size.x/2.0).unwrap(),
			Planar64Vec3::try_from([cf.orientation.x.y,cf.orientation.y.y,cf.orientation.z.y]).unwrap()
			*Planar64::try_from(size.y/2.0).unwrap(),
			Planar64Vec3::try_from([cf.orientation.x.z,cf.orientation.y.z,cf.orientation.z.z]).unwrap()
			*Planar64::try_from(size.z/2.0).unwrap(),
		),
		Planar64Vec3::try_from([cf.position.x,cf.position.y,cf.position.z]).unwrap()
	)
}
fn get_attributes(name:&str,can_collide:bool,velocity:Planar64Vec3,force_intersecting:bool)->crate::model::CollisionAttributes{
	let mut general=crate::model::GameMechanicAttributes::default();
	let mut intersecting=crate::model::IntersectingAttributes::default();
	let mut contacting=crate::model::ContactingAttributes::default();
	let mut force_can_collide=can_collide;
	match name{
		"Water"=>{
			force_can_collide=false;
			//TODO: read stupid CustomPhysicalProperties
			intersecting.water=Some(crate::model::IntersectingWater{density:Planar64::ONE,viscosity:Planar64::ONE/10,current:velocity});
		},
		"Accelerator"=>{
			//although the new game supports collidable accelerators, this is a roblox compatability map loader
			force_can_collide=false;
			general.accelerator=Some(crate::model::GameMechanicAccelerator{acceleration:velocity});
		},
		"UnorderedCheckpoint"=>general.checkpoint=Some(crate::model::GameMechanicCheckpoint::Unordered{mode_id:0}),
		"SetVelocity"=>general.trajectory=Some(crate::model::GameMechanicSetTrajectory::Velocity(velocity)),
		"MapFinish"=>{force_can_collide=false;general.zone=Some(crate::model::GameMechanicZone{mode_id:0,behaviour:crate::model::ZoneBehaviour::Finish})},
		"MapAnticheat"=>{force_can_collide=false;general.zone=Some(crate::model::GameMechanicZone{mode_id:0,behaviour:crate::model::ZoneBehaviour::Anitcheat})},
		"Platform"=>general.teleport_behaviour=Some(crate::model::TeleportBehaviour::StageElement(crate::model::GameMechanicStageElement{
			mode_id:0,
			stage_id:0,
			force:false,
			behaviour:crate::model::StageElementBehaviour::Platform,
		})),
		other=>{
			if let Some(captures)=lazy_regex::regex!(r"^(Force)?(Spawn|SpawnAt|Trigger|Teleport|Platform)(\d+)$")
			.captures(other){
				general.teleport_behaviour=Some(crate::model::TeleportBehaviour::StageElement(crate::model::GameMechanicStageElement{
					mode_id:0,
					stage_id:captures[3].parse::<u32>().unwrap(),
					force:match captures.get(1){
						Some(m)=>m.as_str()=="Force",
						None=>false,
					},
					behaviour:match &captures[2]{
						"Spawn"|"SpawnAt"=>crate::model::StageElementBehaviour::SpawnAt,
						//cancollide false so you don't hit the side
						//NOT a decoration
						"Trigger"=>{force_can_collide=false;crate::model::StageElementBehaviour::Trigger},
						"Teleport"=>{force_can_collide=false;crate::model::StageElementBehaviour::Teleport},
						"Platform"=>crate::model::StageElementBehaviour::Platform,
						_=>panic!("regex1[2] messed up bad"),
					}
				}));
			}else if let Some(captures)=lazy_regex::regex!(r"^(Force)?(Jump)(\d+)$")
			.captures(other){
				general.teleport_behaviour=Some(crate::model::TeleportBehaviour::StageElement(crate::model::GameMechanicStageElement{
					mode_id:0,
					stage_id:0,
					force:match captures.get(1){
						Some(m)=>m.as_str()=="Force",
						None=>false,
					},
					behaviour:match &captures[2]{
						"Jump"=>crate::model::StageElementBehaviour::JumpLimit(captures[3].parse::<u32>().unwrap()),
						_=>panic!("regex4[1] messed up bad"),
					}
				}));
			}else if let Some(captures)=lazy_regex::regex!(r"^Bonus(Finish|Anticheat)(\d+)$")
			.captures(other){
				force_can_collide=false;
				match &captures[1]{
					"Finish"=>general.zone=Some(crate::model::GameMechanicZone{mode_id:captures[2].parse::<u32>().unwrap(),behaviour:crate::model::ZoneBehaviour::Finish}),
					"Anticheat"=>general.zone=Some(crate::model::GameMechanicZone{mode_id:captures[2].parse::<u32>().unwrap(),behaviour:crate::model::ZoneBehaviour::Anitcheat}),
					_=>panic!("regex2[1] messed up bad"),
				}
			}else if let Some(captures)=lazy_regex::regex!(r"^(WormholeIn)(\d+)$")
			.captures(other){
				force_can_collide=false;
				match &captures[1]{
					"WormholeIn"=>general.teleport_behaviour=Some(crate::model::TeleportBehaviour::Wormhole(crate::model::GameMechanicWormhole{destination_model_id:captures[2].parse::<u32>().unwrap()})),
					_=>panic!("regex3[1] messed up bad"),
				}
			}else if let Some(captures)=lazy_regex::regex!(r"^(OrderedCheckpoint)(\d+)$")
			.captures(other){
				match &captures[1]{
					"OrderedCheckpoint"=>general.checkpoint=Some(crate::model::GameMechanicCheckpoint::Ordered{mode_id:0,checkpoint_id:captures[2].parse::<u32>().unwrap()}),
					_=>panic!("regex3[1] messed up bad"),
				}
			}
		}
	}
	//need some way to skip this
	if velocity!=Planar64Vec3::ZERO{
		general.booster=Some(crate::model::GameMechanicBooster::Velocity(velocity));
	}
	match force_can_collide{
		true=>{
			match name{
				"Bounce"=>contacting.contact_behaviour=Some(crate::model::ContactingBehaviour::Elastic(u32::MAX)),
				"Surf"=>contacting.contact_behaviour=Some(crate::model::ContactingBehaviour::Surf),
				"Ladder"=>contacting.contact_behaviour=Some(crate::model::ContactingBehaviour::Ladder(crate::model::ContactingLadder{sticky:true})),
				_=>(),
			}
			crate::model::CollisionAttributes::Contact{contacting,general}
		},
		false=>if force_intersecting
		||general.any()
		||intersecting.any()
		{
			crate::model::CollisionAttributes::Intersect{intersecting,general}
		}else{
			crate::model::CollisionAttributes::Decoration
		},
	}
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
type RobloxCornerWedgeDescription=[Option<RobloxFaceTextureDescription>;5];
#[derive(Clone,Eq,Hash,PartialEq)]
enum RobloxBasePartDescription{
	Sphere(RobloxPartDescription),
	Part(RobloxPartDescription),
	Cylinder(RobloxPartDescription),
	Wedge(RobloxWedgeDescription),
	CornerWedge(RobloxCornerWedgeDescription),
}
pub fn generate_indexed_models(dom:rbx_dom_weak::WeakDom) -> crate::model::IndexedModelInstances{
	//IndexedModelInstances includes textures
	let mut spawn_point=Planar64Vec3::ZERO;

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
					Some(rbx_dom_weak::types::Variant::Vector3(velocity)),
					Some(rbx_dom_weak::types::Variant::Float32(transparency)),
					Some(rbx_dom_weak::types::Variant::Color3uint8(color3)),
					Some(rbx_dom_weak::types::Variant::Bool(can_collide)),
				) = (
					object.properties.get("CFrame"),
					object.properties.get("Size"),
					object.properties.get("Velocity"),
					object.properties.get("Transparency"),
					object.properties.get("Color"),
					object.properties.get("CanCollide"),
				)
			{
				let model_transform=planar64_affine3_from_roblox(cf,size);

				//push TempIndexedAttributes
				let mut force_intersecting=false;
				let mut temp_indexing_attributes=Vec::new();
				if let Some(attr)=match &object.name[..]{
					"MapStart"=>{
						spawn_point=model_transform.transform_point3(Planar64Vec3::ZERO)+Planar64Vec3::Y*5/2;
						Some(crate::model::TempIndexedAttributes::Start(crate::model::TempAttrStart{mode_id:0}))
					},
					other=>{
						let regman=lazy_regex::regex!(r"^(BonusStart|Spawn|ForceSpawn|WormholeOut)(\d+)$");
						if let Some(captures) = regman.captures(other) {
							match &captures[1]{
								"BonusStart"=>Some(crate::model::TempIndexedAttributes::Start(crate::model::TempAttrStart{mode_id:captures[2].parse::<u32>().unwrap()})),
								"Spawn"|"ForceSpawn"=>Some(crate::model::TempIndexedAttributes::Spawn(crate::model::TempAttrSpawn{mode_id:0,stage_id:captures[2].parse::<u32>().unwrap()})),
								"WormholeOut"=>Some(crate::model::TempIndexedAttributes::Wormhole(crate::model::TempAttrWormhole{wormhole_id:captures[2].parse::<u32>().unwrap()})),
								_=>None,
							}
						}else{
							None
						}
					}
				}{
					force_intersecting=true;
					temp_indexing_attributes.push(attr);
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
					"TrussPart"=>primitives::Primitives::Cube,
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
									let (roblox_texture_color,roblox_texture_transform)=if decal.class=="Texture"{
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
											(
												glam::vec4(decal_color3.r,decal_color3.g,decal_color3.b,1.0-*decal_transparency),
												RobloxTextureTransform{
													offset_u:*ox/(*sx),offset_v:*oy/(*sy),
													scale_u:size_u/(*sx),scale_v:size_v/(*sy),
												}
											)
										}else{
											(glam::Vec4::ONE,RobloxTextureTransform::default())
										}
									}else{
										(glam::Vec4::ONE,RobloxTextureTransform::default())
									};
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
				let [
					f0,//Cube::Right
					f1,//Cube::Top
					f2,//Cube::Back
					f3,//Cube::Left
					f4,//Cube::Bottom
					f5,//Cube::Front
				]=part_texture_description;
				let basepart_texture_description=match shape{
					primitives::Primitives::Sphere=>RobloxBasePartDescription::Sphere([f0,f1,f2,f3,f4,f5]),
					primitives::Primitives::Cube=>RobloxBasePartDescription::Part([f0,f1,f2,f3,f4,f5]),
					primitives::Primitives::Cylinder=>RobloxBasePartDescription::Cylinder([f0,f1,f2,f3,f4,f5]),
					//use front face texture first and use top face texture as a fallback
					primitives::Primitives::Wedge=>RobloxBasePartDescription::Wedge([
						f0,//Cube::Right->Wedge::Right
						if f5.is_some(){f5}else{f1},//Cube::Front|Cube::Top->Wedge::TopFront
						f2,//Cube::Back->Wedge::Back
						f3,//Cube::Left->Wedge::Left
						f4,//Cube::Bottom->Wedge::Bottom
					]),
					//TODO: fix Left+Back texture coordinates to match roblox when not overwridden by Top
					primitives::Primitives::CornerWedge=>RobloxBasePartDescription::CornerWedge([
						f0,//Cube::Right->CornerWedge::Right
						if f2.is_some(){f2}else{f1.clone()},//Cube::Back|Cube::Top->CornerWedge::TopBack
						if f3.is_some(){f3}else{f1},//Cube::Left|Cube::Top->CornerWedge::TopLeft
						f4,//Cube::Bottom->CornerWedge::Bottom
						f5,//Cube::Front->CornerWedge::Front
					]),
				};
				//make new model if unit cube has not been created before
				let model_id=if let Some(&model_id)=model_id_from_description.get(&basepart_texture_description){
					//push to existing texture model
					model_id
				}else{
					let model_id=indexed_models.len();
					model_id_from_description.insert(basepart_texture_description.clone(),model_id);//borrow checker going crazy
					indexed_models.push(match basepart_texture_description{
						RobloxBasePartDescription::Sphere(part_texture_description)
						|RobloxBasePartDescription::Cylinder(part_texture_description)
						|RobloxBasePartDescription::Part(part_texture_description)=>{
							let mut cube_face_description=primitives::CubeFaceDescription::default();
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
						RobloxBasePartDescription::Wedge(wedge_texture_description)=>{
							let mut wedge_face_description=primitives::WedgeFaceDescription::default();
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
							let mut cornerwedge_face_description=primitives::CornerWedgeFaceDescription::default();
							for (face_id,roblox_face_description) in cornerwedge_texture_description.iter().enumerate(){
								cornerwedge_face_description.insert(
								match face_id{
									0=>primitives::CornerWedgeFace::Right,
									1=>primitives::CornerWedgeFace::TopBack,
									2=>primitives::CornerWedgeFace::TopLeft,
									3=>primitives::CornerWedgeFace::Bottom,
									4=>primitives::CornerWedgeFace::Front,
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
					attributes:get_attributes(&object.name,*can_collide,Planar64Vec3::try_from([velocity.x,velocity.y,velocity.z]).unwrap(),force_intersecting),
					temp_indexing:temp_indexing_attributes,
				});
			}
		}
	}
	crate::model::IndexedModelInstances{
		textures:asset_id_from_texture_id.iter().map(|t|t.to_string()).collect(),
		models:indexed_models,
		spawn_point,
		modes:Vec::new(),
	}
}

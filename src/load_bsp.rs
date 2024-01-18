const VALVE_SCALE:f32=1.0/16.0;
fn valve_transform(v:[f32;3])->crate::integer::Planar64Vec3{
	crate::integer::Planar64Vec3::try_from([v[0]*VALVE_SCALE,v[2]*VALVE_SCALE,-v[1]*VALVE_SCALE]).unwrap()
}
pub fn generate_indexed_models<R:std::io::Read+std::io::Seek>(input:&mut R)->Result<crate::model::IndexedModelInstances,vbsp::BspError>{
	let mut s=Vec::new();

	match input.read_to_end(&mut s){
		Ok(_)=>(),
		Err(e)=>println!("load_bsp::generate_indexed_models read_to_end failed: {:?}",e),
	}

	match vbsp::Bsp::read(s.as_slice()){
		Ok(bsp)=>{
			let mut spawn_point=crate::integer::Planar64Vec3::ZERO;

			let vertices: Vec<_> = bsp
				.vertices
				.iter()
				.map(|vertex|<[f32;3]>::from(vertex.position))
				.collect();

			let mut name_from_texture_id=Vec::new();
			let mut texture_id_from_name=std::collections::HashMap::new();

			let mut models=bsp.models().map(|world_model|{
				//non-deduplicated
				let mut spam_pos=Vec::new();
				let mut spam_tex=Vec::new();
				let mut spam_normal=Vec::new();
				let mut spam_vertices=Vec::new();
				let groups=world_model.faces()
				.filter(|face| face.is_visible())//TODO: look at this
				.map(|face|{
					let face_texture=face.texture();
					let face_texture_data=face_texture.texture_data();
					let (texture_u,texture_v)=(glam::Vec3A::from_slice(&face_texture.texture_transforms_u[0..3]),glam::Vec3A::from_slice(&face_texture.texture_transforms_v[0..3]));
					let texture_offset=glam::vec2(face_texture.texture_transforms_u[3],face_texture.texture_transforms_v[3]);
					let texture_size=glam::vec2(face_texture_data.width as f32,face_texture_data.height as f32);

					//texture
					let texture_id=if let Some(&texture_id)=texture_id_from_name.get(face_texture_data.name()){
						texture_id
					}else{
						let texture_id=name_from_texture_id.len() as u32;
						texture_id_from_name.insert(face_texture_data.name().to_string(),texture_id);
						name_from_texture_id.push(face_texture_data.name().to_string());
						texture_id
					};

					//normal
					let normal=face.normal();
					let normal_idx=spam_normal.len() as u32;
					spam_normal.push(valve_transform(<[f32;3]>::from(normal)));
					let mut vertices:Vec<u32>=face.vertex_indexes().map(|vertex_index|{
						let pos=glam::Vec3A::from_array(vertices[vertex_index as usize]);
						let pos_idx=spam_pos.len();
						spam_pos.push(valve_transform(vertices[vertex_index as usize]));

						//calculate texture coordinates
						let tex=(glam::vec2(pos.dot(texture_u),pos.dot(texture_v))+texture_offset)/texture_size;
						let tex_idx=spam_tex.len() as u32;
						spam_tex.push(tex);

						let i=spam_vertices.len() as u32;
						spam_vertices.push(crate::model::IndexedVertex{
							pos: pos_idx as u32,
							tex: tex_idx as u32,
							normal: normal_idx,
							color: 0,
						});
						i
					}).collect();
					vertices.reverse();
					crate::model::IndexedGroup{
						texture:Some(texture_id),
						polys:vec![crate::model::IndexedPolygon{vertices}],
					}
				}).collect();
				crate::model::IndexedModel{
					unique_pos:spam_pos,
					unique_tex:spam_tex,
					unique_normal:spam_normal,
					unique_color:vec![glam::Vec4::ONE],
					unique_vertices:spam_vertices,
					groups,
					instances:vec![crate::model::ModelInstance{
						attributes:crate::model::CollisionAttributes::Decoration,
						transform:crate::integer::Planar64Affine3::new(
							crate::integer::Planar64Mat3::default(),
							valve_transform(<[f32;3]>::from(world_model.origin))
						),
						..Default::default()
					}],
				}
			}).collect();

			//dedupe prop models
			let mut model_dedupe=std::collections::HashSet::new();
			for prop in bsp.static_props(){
				model_dedupe.insert(prop.model());
			}

			//generate unique meshes
			let mut model_map=std::collections::HashMap::with_capacity(model_dedupe.len());
			let mut prop_models=Vec::new();
			for model_name in model_dedupe{
				let model_name_lower=model_name.to_lowercase();
				//.mdl, .vvd, .dx90.vtx
				let mut path=std::path::PathBuf::from(model_name_lower.as_str());
				let file_name=std::path::PathBuf::from(path.file_stem().unwrap());
				path.pop();
				path.push(file_name);
				let mut vvd_path=path.clone();
				let mut vtx_path=path.clone();
				vvd_path.set_extension("vvd");
				vtx_path.set_extension("dx90.vtx");
				match (bsp.pack.get(model_name_lower.as_str()),bsp.pack.get(vvd_path.as_os_str().to_str().unwrap()),bsp.pack.get(vtx_path.as_os_str().to_str().unwrap())){
					(Ok(Some(mdl_file)),Ok(Some(vvd_file)),Ok(Some(vtx_file)))=>{
						match (vmdl::mdl::Mdl::read(mdl_file.as_ref()),vmdl::vvd::Vvd::read(vvd_file.as_ref()),vmdl::vtx::Vtx::read(vtx_file.as_ref())){
							(Ok(mdl),Ok(vvd),Ok(vtx))=>{
								let model=vmdl::Model::from_parts(mdl,vtx,vvd);
								let texture_paths=model.texture_directories();
								if texture_paths.len()!=1{
									println!("WARNING: multiple texture paths");
								}
								let skin=model.skin_tables().nth(0).unwrap();

								let mut spam_pos=Vec::with_capacity(model.vertices().len());
								let mut spam_normal=Vec::with_capacity(model.vertices().len());
								let mut spam_tex=Vec::with_capacity(model.vertices().len());
								let mut spam_vertices=Vec::with_capacity(model.vertices().len());
								for (i,vertex) in model.vertices().iter().enumerate(){
									spam_pos.push(valve_transform(<[f32;3]>::from(vertex.position)));
									spam_normal.push(valve_transform(<[f32;3]>::from(vertex.normal)));
									spam_tex.push(glam::Vec2::from_array(vertex.texture_coordinates));
									spam_vertices.push(crate::model::IndexedVertex{
										pos:i as u32,
										tex:i as u32,
										normal:i as u32,
										color:0,
									});
								}

								let model_id=prop_models.len();
								model_map.insert(model_name,model_id);
								prop_models.push(crate::model::IndexedModel{
									unique_pos:spam_pos,
									unique_normal:spam_normal,
									unique_tex:spam_tex,
									unique_color:vec![glam::Vec4::ONE],
									unique_vertices:spam_vertices,
									groups:model.meshes().map(|mesh|{
										let texture=if let (Some(texture_path),Some(texture_name))=(texture_paths.get(0),skin.texture(mesh.material_index())){
											let mut path=std::path::PathBuf::from(texture_path.as_str());
											path.push(texture_name);
											let texture_location=path.as_os_str().to_str().unwrap();
											let texture_id=if let Some(&texture_id)=texture_id_from_name.get(texture_location){
												texture_id
											}else{
												println!("texture! {}",texture_location);
												let texture_id=name_from_texture_id.len() as u32;
												texture_id_from_name.insert(texture_location.to_string(),texture_id);
												name_from_texture_id.push(texture_location.to_string());
												texture_id
											};
											Some(texture_id)
										}else{
											None
										};

										crate::model::IndexedGroup{
											texture,
											polys:{
												//looking at the code, it would seem that the strips are pre-deindexed into triangle lists when calling this function
												mesh.vertex_strip_indices().map(|strip|{
													strip.collect::<Vec<usize>>().chunks(3).map(|tri|{
														crate::model::IndexedPolygon{vertices:vec![tri[0] as u32,tri[1] as u32,tri[2] as u32]}
													}).collect::<Vec<crate::model::IndexedPolygon>>()
												}).flatten().collect()
											},
										}
									}).collect(),
									instances:Vec::new(),
								});
							},
							_=>println!("model_name={} error",model_name),
						}
					},
					_=>println!("no model name={}",model_name),
				}
			}

			//generate model instances
			for prop in bsp.static_props(){
				let placement=prop.as_prop_placement();
				if let Some(&model_index)=model_map.get(placement.model){
					prop_models[model_index].instances.push(crate::model::ModelInstance{
						transform:crate::integer::Planar64Affine3::new(
							crate::integer::Planar64Mat3::try_from(
								glam::Mat3A::from_diagonal(glam::Vec3::splat(placement.scale))
								//TODO: figure this out
								*glam::Mat3A::from_quat(glam::Quat::from_xyzw(
									placement.rotation.v.x,//b
									placement.rotation.v.y,//c
									placement.rotation.v.z,//d
									placement.rotation.s,//a
								))
							).unwrap(),
							valve_transform(<[f32;3]>::from(placement.origin)),
						),
						attributes:crate::model::CollisionAttributes::Decoration,
						..Default::default()
					});
				}else{
					//println!("model not found {}",placement.model);
				}
			}

			//actually add the prop models
			prop_models.append(&mut models);

			Ok(crate::model::IndexedModelInstances{
				textures:name_from_texture_id,
				models:prop_models,
				spawn_point,
				modes:Vec::new(),
			})
		},
		Err(e)=>{
			println!("rotten {:?}",e);
			Err(e)
		},
	}
}

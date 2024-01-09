const VALVE_SCALE:f32=1.0/16.0;
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

			let models=bsp.models().map(|world_model|{
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
					let (s,t)=(glam::Vec3A::from_slice(&face_texture.texture_transforms_u[0..3]),glam::Vec3A::from_slice(&face_texture.texture_transforms_v[0..3]));
					let (s0,t0)=(face_texture.texture_transforms_u[3],face_texture.texture_transforms_v[3]);

					//texture
					let texture_id=if let Some(&texture_id)=texture_id_from_name.get(face_texture_data.name()){
						texture_id
					}else{
						let texture_id=name_from_texture_id.len() as u32;
						texture_id_from_name.insert(face_texture_data.name(),texture_id);
						name_from_texture_id.push(face_texture_data.name().to_string());
						texture_id
					};

					//normal
					let normal=face.normal();
					let normal_idx=spam_normal.len() as u32;
					spam_normal.push(crate::integer::Planar64Vec3::try_from([normal.x,normal.z,normal.y]).unwrap());
					
					crate::model::IndexedGroup{
						texture:Some(texture_id),
						polys:vec![crate::model::IndexedPolygon{vertices:face.vertex_indexes().map(|vertex_index|{
							let pos=glam::Vec3A::from_array(vertices[vertex_index as usize]);
							let pos_idx=spam_pos.len();
							spam_pos.push(crate::integer::Planar64Vec3::try_from(glam::Vec3Swizzles::xzy(pos)*VALVE_SCALE).unwrap());

							//calculate texture coordinates
							let tex=[(pos.dot(s)+s0)/face_texture_data.width as f32,(pos.dot(t)+t0)/face_texture_data.height as f32];
							let tex_idx=spam_tex.len() as u32;
							spam_tex.push(crate::model::TextureCoordinate::from_array(tex));

							let i=spam_vertices.len() as u32;
							spam_vertices.push(crate::model::IndexedVertex{
								pos: pos_idx as u32,
								tex: tex_idx as u32,
								normal: normal_idx,
								color: 0,
							});
							i
						}).collect()}],
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
						..Default::default()
					}],
				}
			}).collect();

			Ok(crate::model::IndexedModelInstances{
				textures:name_from_texture_id,
				models,
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
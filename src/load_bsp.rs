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

			let mut indexed_models=Vec::new();

			let vertices: Vec<_> = bsp
				.vertices
				.iter()
				.map(|vertex|[vertex.position.x*VALVE_SCALE,vertex.position.z*VALVE_SCALE,vertex.position.y*VALVE_SCALE])
				.collect();

			let world_objects=bsp.models().map(|world_model|{
				let world_polygons:Vec<obj::SimplePolygon> = world_model
					.faces()
					.filter(|face| face.is_visible())
					.map(|face| {
						face.vertex_indexes()
							.map(|vertex_index| obj::IndexTuple(vertex_index as usize, Some(0), Some(0)))
							.collect()
					})
					.map(obj::SimplePolygon)
					.collect();

				obj::Object {
					name: "".to_string(),
					groups: vec![obj::Group {
						name: "".to_string(),
						index: 0,
						material: None,
						polys: world_polygons,
					}],
				}
			}).collect();

			let obj_data = obj::ObjData {
				position: vertices,
				texture: vec![[0.0,0.0]],
				normal: vec![[1.0,0.0,0.0]],
				objects: world_objects,
				material_libs: Vec::new(),
			};

			let mut new_indexed_models=crate::model::generate_indexed_model_list_from_obj(obj_data,glam::Vec4::ONE);

			for indexed_model in &mut new_indexed_models{
				indexed_model.instances.push(crate::model::ModelInstance{attributes:crate::model::CollisionAttributes::Decoration,..Default::default()});
			}

			indexed_models.append(&mut new_indexed_models);

			Ok(crate::model::IndexedModelInstances{
				textures:Vec::new(),
				models:indexed_models,
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

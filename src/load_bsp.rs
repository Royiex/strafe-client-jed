
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

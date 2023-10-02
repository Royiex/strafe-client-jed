pub fn generate_indexed_models<R:std::io::Read+std::io::Seek>(input:R) -> crate::model::IndexedModelInstances{
	let mut spawn_point=glam::Vec3::ZERO;

	let mut indexed_models=Vec::new();

	match bsp::Bsp::read(input){
		Ok(guac)=>println!("we got the guac {:?}", guac),
		Err(e)=>println!("rotten {:?}",e),
	}

	crate::model::IndexedModelInstances{
		textures:Vec::new(),
		models:indexed_models,
		spawn_point,
	}
}
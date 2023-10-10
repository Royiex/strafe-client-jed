pub fn generate_indexed_models<R:std::io::Read+std::io::Seek>(input:&mut R) -> crate::model::IndexedModelInstances{
	let mut spawn_point=crate::integer::Planar64Vec3::ZERO;

	let mut indexed_models=Vec::new();

	let mut s=Vec::new();

	match input.read_to_end(&mut s){
		Ok(guac)=>println!("readed to string {:?}", guac),
		Err(e)=>println!("faile {:?}",e),
	}

	match vbsp::Bsp::read(s.as_slice()){
		Ok(guac)=>println!("we got the guac {:?}", guac),
		Err(e)=>println!("rotten {:?}",e),
	}

	crate::model::IndexedModelInstances{
		textures:Vec::new(),
		models:indexed_models,
		spawn_point,
		modes:Vec::new(),
	}
}

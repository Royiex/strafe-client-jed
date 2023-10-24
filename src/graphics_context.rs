struct Context{
	device:wgpu::Device,
	queue:wgpu::Queue,
}

impl Context{
	pub fn new(user_settings:&crate::settings::UserSettings,indexed_model_instances:&crate::model::IndexedModelInstances){
		let mut graphics=crate::graphics::GraphicsState::new();
		graphics.load_user_settings(user_settings);
		graphics.generate_models(indexed_model_instances);
	}
	//into_worker
}
#[derive(Clone)]
pub enum Instruction{
	Render(crate::physics::PhysicsOutputState,crate::integer::Time,glam::IVec2),
	//UpdateModel(crate::graphics::ModelUpdate),
	Resize(winit::dpi::PhysicalSize<u32>,crate::settings::UserSettings),
}

//Ideally the graphics thread worker description is:
/*
WorkerDescription{
	input:Immediate,
	output:Realtime(PoolOrdering::Ordered(3)),
}
*/
//up to three frames in flight, dropping new frame requests when all three are busy, and dropping output frames when one renders out of order

pub fn new<'a>(
		mut graphics:crate::graphics::GraphicsState,
		mut config:wgpu::SurfaceConfiguration,
		surface:wgpu::Surface,
		device:wgpu::Device,
		queue:wgpu::Queue,
	)->crate::compat_worker::INWorker<'a,Instruction>{
	crate::compat_worker::INWorker::new(move |ins:Instruction|{
		match ins{
			Instruction::Resize(size,user_settings)=>{
				config.width=size.width.max(1);
				config.height=size.height.max(1);
				surface.configure(&device,&config);
				graphics.resize(&device,&config,&user_settings);
			}
			Instruction::Render(physics_output,predicted_time,mouse_pos)=>{
				//this has to go deeper somehow
				let frame=match surface.get_current_texture(){
					Ok(frame)=>frame,
					Err(_)=>{
						surface.configure(&device,&config);
						surface
							.get_current_texture()
							.expect("Failed to acquire next surface texture!")
					}
				};
				let view=frame.texture.create_view(&wgpu::TextureViewDescriptor{
					format:Some(config.view_formats[0]),
					..wgpu::TextureViewDescriptor::default()
				});

				graphics.render(&view,&device,&queue,physics_output,predicted_time,mouse_pos);

				frame.present();
			}
		}
	})
}
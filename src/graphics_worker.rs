#[derive(Clone)]
pub enum GraphicsInstruction{
	Render(crate::physics::PhysicsOutputState,crate::integer::Time),
	//UpdateModel(crate::graphics::ModelUpdate),
}

//Ideally the graphics thread worker description is:
/*
WorkerDescription{
	input:Immediate,
	output:Realtime(PoolOrdering::Ordered(3)),
}
*/
//up to three frames in flight, dropping new frame requests when all three are busy, and dropping output frames when one renders out of order

pub fn new(
		graphics:crate::graphics::GraphicsState,
		surface:&wgpu::Surface,
		device:&wgpu::Device,
		queue:&wgpu::Queue,
	)->crate::worker::INWorker<GraphicsInstruction>{
	crate::worker::INWorker::new(a,move |ins:GraphicsInstruction|{
		match ins{
			GraphicsInstruction::Render(physics_output,predicted_time)=>{
				//this has to go deeper somehow
				let frame=match surface.get_current_texture(){
					Ok(frame)=>frame,
					Err(_)=>{
						surface.configure(device,config);
						surface
							.get_current_texture()
							.expect("Failed to acquire next surface texture!")
					}
				};
				let view=frame.texture.create_view(&wgpu::TextureViewDescriptor{
					format:Some(config.view_formats[0]),
					..wgpu::TextureViewDescriptor::default()
				});

				graphics.render(&view,device,queue,physics_output,predicted_time);

				frame.present();
			}
		}
	})
}
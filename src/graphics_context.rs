fn optional_features() -> wgpu::Features {
	wgpu::Features::empty()
}
fn required_features() -> wgpu::Features {
	wgpu::Features::empty()
}
fn required_downlevel_capabilities() -> wgpu::DownlevelCapabilities {
	wgpu::DownlevelCapabilities {
		flags: wgpu::DownlevelFlags::empty(),
		shader_model: wgpu::ShaderModel::Sm5,
		..wgpu::DownlevelCapabilities::default()
	}
}
pub fn required_limits() -> wgpu::Limits {
	wgpu::Limits::downlevel_webgl2_defaults() // These downlevel limits will allow the code to run on all possible hardware
}

struct GraphicsContextPartial1{
	backends:wgpu::Backends,
	instance:wgpu::Instance,
}
fn create_instance()->GraphicsContextPartial1{
	let backends=wgpu::util::backend_bits_from_env().unwrap_or_else(wgpu::Backends::all);
	let dx12_shader_compiler=wgpu::util::dx12_shader_compiler_from_env().unwrap_or_default();
	GraphicsContextPartial1{
		backends,
		instance:wgpu::Instance::new(wgpu::InstanceDescriptor{
			backends,
			dx12_shader_compiler,
		}),
	}
}
impl GraphicsContextPartial1{
	fn create_surface(self,window:&winit::window::Window)->Result<GraphicsContextPartial2,wgpu::CreateSurfaceError>{
		Ok(GraphicsContextPartial2{
			backends:self.backends,
			instance:self.instance,
			surface:unsafe{self.instance.create_surface(window)}?
		})
	}
}
struct GraphicsContextPartial2{
	backends:wgpu::Backends,
	instance:wgpu::Instance,
	surface:wgpu::Surface,
}
impl GraphicsContextPartial2{
	fn pick_adapter(self)->GraphicsContextPartial3{
		let adapter;

		let optional_features=optional_features();
		let required_features=required_features();

		//no helper function smh gotta write it myself
		let adapters=self.instance.enumerate_adapters(self.backends);

		let mut chosen_adapter=None;
		let mut chosen_adapter_score=0;
		for adapter in adapters {
			if !adapter.is_surface_supported(&self.surface) {
				continue;
			}

			let score=match adapter.get_info().device_type{
				wgpu::DeviceType::IntegratedGpu=>3,
				wgpu::DeviceType::DiscreteGpu=>4,
				wgpu::DeviceType::VirtualGpu=>2,
				wgpu::DeviceType::Other|wgpu::DeviceType::Cpu=>1,
			};

			let adapter_features=adapter.features();
			if chosen_adapter_score<score&&adapter_features.contains(required_features) {
				chosen_adapter_score=score;
				chosen_adapter=Some(adapter);
			}
		}

		if let Some(maybe_chosen_adapter)=chosen_adapter{
			adapter=maybe_chosen_adapter;
		}else{
			panic!("No suitable GPU adapters found on the system!");
		}


		let adapter_info=adapter.get_info();
		println!("Using {} ({:?})", adapter_info.name, adapter_info.backend);

		let required_downlevel_capabilities=required_downlevel_capabilities();
		let downlevel_capabilities=adapter.get_downlevel_capabilities();
		assert!(
			downlevel_capabilities.shader_model >= required_downlevel_capabilities.shader_model,
			"Adapter does not support the minimum shader model required to run this example: {:?}",
			required_downlevel_capabilities.shader_model
		);
		assert!(
			downlevel_capabilities
				.flags
				.contains(required_downlevel_capabilities.flags),
			"Adapter does not support the downlevel capabilities required to run this example: {:?}",
			required_downlevel_capabilities.flags - downlevel_capabilities.flags
		);
		GraphicsContextPartial3{
			instance:self.instance,
			surface:self.surface,
			adapter,
		}
	}
}
struct GraphicsContextPartial3{
	instance:wgpu::Instance,
	surface:wgpu::Surface,
	adapter:wgpu::Adapter,
}
impl GraphicsContextPartial3{
	fn request_device(self)->GraphicsContextPartial4{
		let optional_features=optional_features();
		let required_features=required_features();

		// Make sure we use the texture resolution limits from the adapter, so we can support images the size of the surface.
		let needed_limits = required_limits().using_resolution(self.adapter.limits());

		let trace_dir = std::env::var("WGPU_TRACE");
		let (device, queue) = pollster::block_on(self.adapter
			.request_device(
				&wgpu::DeviceDescriptor {
					label: None,
					features: (optional_features & self.adapter.features()) | required_features,
					limits: needed_limits,
				},
				trace_dir.ok().as_ref().map(std::path::Path::new),
			))
			.expect("Unable to find a suitable GPU adapter!");

		GraphicsContextPartial4{
			instance:self.instance,
			surface:self.surface,
			adapter:self.adapter,
			device,
			queue,
		}
	}
}
struct GraphicsContextPartial4{
	instance:wgpu::Instance,
	surface:wgpu::Surface,
	adapter:wgpu::Adapter,
	device:wgpu::Device,
	queue:wgpu::Queue,
}
impl GraphicsContextPartial4{
	fn configure_surface(self,size:&winit::dpi::PhysicalSize<u32>)->GraphicsContext{
		let mut config = self.surface
			.get_default_config(&self.adapter, size.width, size.height)
			.expect("Surface isn't supported by the adapter.");
		let surface_view_format = config.format.add_srgb_suffix();
		config.view_formats.push(surface_view_format);
		self.surface.configure(&self.device, &config);

		GraphicsContext{
			instance:self.instance,
			surface:self.surface,
			adapter:self.adapter,
			device:self.device,
			queue:self.queue,
			config,
		}
	}
}
struct GraphicsContext{
	instance:wgpu::Instance,
	surface:wgpu::Surface,
	adapter:wgpu::Adapter,
	device:wgpu::Device,
	queue:wgpu::Queue,
	config:wgpu::SurfaceConfiguration,
}

pub fn setup(title:&str)->GraphicsContextSetup{
	let event_loop=winit::event_loop::EventLoop::new();

	let window=crate::window::WindowState::create_window(title,&event_loop).unwrap();

	println!("Initializing the surface...");

	let partial_1=create_instance();

	let partial_2=partial_1.create_surface(&window).unwrap();

	let partial_3=partial_2.pick_adapter();

	let partial_4=partial_3.request_device();

	GraphicsContextSetup{
		window,
		event_loop,
		partial_graphics_context:partial_4,
	}
}

struct GraphicsContextSetup{
	window: winit::window::Window,
	event_loop:winit::event_loop::EventLoop<()>,
	partial_graphics_context:crate::graphics_context::GraphicsContextPartial4,
}

impl GraphicsContextSetup{
	pub fn start(self){
		let size = self.window.inner_size();

		let graphics_context=self.partial_graphics_context.configure_surface(&size);

		println!("Initializing global state...");
		let mut example=GlobalState::init();

		println!("Entering render loop...");
		event_loop.run(move |event, _, control_flow| {
			let _ = (&instance, &adapter); // force ownership by the closure
			*control_flow = if cfg!(feature = "metal-auto-capture") {
				ControlFlow::Exit
			} else {
				ControlFlow::Poll
			};
			match event {
				event::Event::RedrawEventsCleared => {
					window.request_redraw();
				}
				event::Event::WindowEvent {
					event:
						WindowEvent::Resized(size)
						| WindowEvent::ScaleFactorChanged {
							new_inner_size: &mut size,
							..
						},
					..
				} => {
					// Once winit is fixed, the detection conditions here can be removed.
					// https://github.com/rust-windowing/winit/issues/2876
					// this has been fixed if I update winit (remove the if statement and only use the else case)
					let max_dimension = adapter.limits().max_texture_dimension_2d;
					if size.width > max_dimension || size.height > max_dimension {
						println!(
							"The resizing size {:?} exceeds the limit of {}.",
							size,
							max_dimension
						);
					} else {
						println!("Resizing to {:?}", size);
						config.width = size.width.max(1);
						config.height = size.height.max(1);
						example.resize(&config, &device, &queue);
						surface.configure(&device, &config);
					}
				}
				event::Event::WindowEvent { event, .. } => match event {
					WindowEvent::KeyboardInput {
						input:
							event::KeyboardInput {
								virtual_keycode: Some(event::VirtualKeyCode::Escape),
								state: event::ElementState::Pressed,
								..
							},
						..
					}
					| WindowEvent::CloseRequested => {
						*control_flow = ControlFlow::Exit;
					}
					WindowEvent::KeyboardInput {
						input:
							event::KeyboardInput {
								virtual_keycode: Some(event::VirtualKeyCode::Scroll),
								state: event::ElementState::Pressed,
								..
							},
						..
					} => {
						println!("{:#?}", instance.generate_report());
					}
					_ => {
						example.update(&window,&device,&queue,event);
					}
				},
				event::Event::DeviceEvent {
					event,
					..
				} => {
					example.device_event(&window,event);
				},
				event::Event::RedrawRequested(_) => {

					let frame = match surface.get_current_texture() {
						Ok(frame) => frame,
						Err(_) => {
							surface.configure(&device, &config);
							surface
								.get_current_texture()
								.expect("Failed to acquire next surface texture!")
						}
					};
					let view = frame.texture.create_view(&wgpu::TextureViewDescriptor {
						format: Some(surface_view_format),
						..wgpu::TextureViewDescriptor::default()
					});

					example.render(&view, &device, &queue);

					frame.present();
				}
				_ => {}
			}
		});
	}
}
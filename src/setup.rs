use crate::window::WindowInstruction;
use strafesnet_common::instruction::TimedInstruction;
use strafesnet_common::integer;

fn optional_features()->wgpu::Features{
	wgpu::Features::TEXTURE_COMPRESSION_ASTC
	|wgpu::Features::TEXTURE_COMPRESSION_ETC2
}
fn required_features()->wgpu::Features{
	wgpu::Features::TEXTURE_COMPRESSION_BC
}
fn required_downlevel_capabilities()->wgpu::DownlevelCapabilities{
	wgpu::DownlevelCapabilities{
		flags:wgpu::DownlevelFlags::empty(),
		shader_model:wgpu::ShaderModel::Sm5,
		..wgpu::DownlevelCapabilities::default()
	}
}
pub fn required_limits()->wgpu::Limits{
	wgpu::Limits::default()
}

struct SetupContextPartial1{
	backends:wgpu::Backends,
	instance:wgpu::Instance,
}
fn create_window(title:&str,event_loop:&winit::event_loop::EventLoop<()>)->Result<winit::window::Window,winit::error::OsError>{
	let mut builder = winit::window::WindowBuilder::new();
	builder = builder.with_title(title);
	#[cfg(windows_OFF)] // TODO
	{
		use winit::platform::windows::WindowBuilderExtWindows;
		builder = builder.with_no_redirection_bitmap(true);
	}
	builder.build(event_loop)
}
fn create_instance()->SetupContextPartial1{
	let backends=wgpu::util::backend_bits_from_env().unwrap_or_else(wgpu::Backends::all);
	let dx12_shader_compiler=wgpu::util::dx12_shader_compiler_from_env().unwrap_or_default();
	SetupContextPartial1{
		backends,
		instance:wgpu::Instance::new(wgpu::InstanceDescriptor{
			backends,
			dx12_shader_compiler,
			..Default::default()
		}),
	}
}
impl SetupContextPartial1{
	fn create_surface<'a>(self,window:&'a winit::window::Window)->Result<SetupContextPartial2<'a>,wgpu::CreateSurfaceError>{
		Ok(SetupContextPartial2{
			backends:self.backends,
			surface:self.instance.create_surface(window)?,
			instance:self.instance,
		})
	}
}
struct SetupContextPartial2<'a>{
	backends:wgpu::Backends,
	instance:wgpu::Instance,
	surface:wgpu::Surface<'a>,
}
impl<'a> SetupContextPartial2<'a>{
	fn pick_adapter(self)->SetupContextPartial3<'a>{
		let adapter;

		//TODO: prefer adapter that implements optional features
		//let optional_features=optional_features();
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
		SetupContextPartial3{
			instance:self.instance,
			surface:self.surface,
			adapter,
		}
	}
}
struct SetupContextPartial3<'a>{
	instance:wgpu::Instance,
	surface:wgpu::Surface<'a>,
	adapter:wgpu::Adapter,
}
impl<'a> SetupContextPartial3<'a>{
	fn request_device(self)->SetupContextPartial4<'a>{
		let optional_features=optional_features();
		let required_features=required_features();

		// Make sure we use the texture resolution limits from the adapter, so we can support images the size of the surface.
		let needed_limits=required_limits().using_resolution(self.adapter.limits());

		let trace_dir=std::env::var("WGPU_TRACE");
		let (device, queue)=pollster::block_on(self.adapter
			.request_device(
				&wgpu::DeviceDescriptor {
					label: None,
					required_features: (optional_features & self.adapter.features()) | required_features,
					required_limits: needed_limits,
				},
				trace_dir.ok().as_ref().map(std::path::Path::new),
			))
			.expect("Unable to find a suitable GPU adapter!");

		SetupContextPartial4{
			instance:self.instance,
			surface:self.surface,
			adapter:self.adapter,
			device,
			queue,
		}
	}
}
struct SetupContextPartial4<'a>{
	instance:wgpu::Instance,
	surface:wgpu::Surface<'a>,
	adapter:wgpu::Adapter,
	device:wgpu::Device,
	queue:wgpu::Queue,
}
impl<'a> SetupContextPartial4<'a>{
	fn configure_surface(self,size:&'a winit::dpi::PhysicalSize<u32>)->SetupContext<'a>{
		let mut config=self.surface
			.get_default_config(&self.adapter, size.width, size.height)
			.expect("Surface isn't supported by the adapter.");
		let surface_view_format=config.format.add_srgb_suffix();
		config.view_formats.push(surface_view_format);
		config.present_mode=wgpu::PresentMode::AutoNoVsync;
		self.surface.configure(&self.device, &config);

		SetupContext{
			instance:self.instance,
			surface:self.surface,
			device:self.device,
			queue:self.queue,
			config,
		}
	}
}
pub struct SetupContext<'a>{
	pub instance:wgpu::Instance,
	pub surface:wgpu::Surface<'a>,
	pub device:wgpu::Device,
	pub queue:wgpu::Queue,
	pub config:wgpu::SurfaceConfiguration,
}

pub fn setup_and_start(title:String){
	let event_loop=winit::event_loop::EventLoop::new().unwrap();

	println!("Initializing the surface...");

	let partial_1=create_instance();

	let window=create_window(title.as_str(),&event_loop).unwrap();

	let partial_2=partial_1.create_surface(&window).unwrap();

	let partial_3=partial_2.pick_adapter();

	let partial_4=partial_3.request_device();

	let size=window.inner_size();

	let setup_context=partial_4.configure_surface(&size);

	//dedicated thread to ping request redraw back and resize the window doesn't seem logical

	let window=crate::window::WindowContextSetup::new(&setup_context,&window);
	//the thread that spawns the physics thread
	let mut window_thread=window.into_worker(setup_context);

	let args:Vec<String>=std::env::args().collect();
	if args.len()==2{
		let path=std::path::PathBuf::from(&args[1]);
		window_thread.send(TimedInstruction{
			time:integer::Time::ZERO,
			instruction:WindowInstruction::WindowEvent(winit::event::WindowEvent::DroppedFile(path)),
		}).unwrap();
	};

	println!("Entering event loop...");
	let root_time=std::time::Instant::now();
	run_event_loop(event_loop,window_thread,root_time).unwrap();
}

fn run_event_loop(
	event_loop:winit::event_loop::EventLoop<()>,
	mut window_thread:crate::compat_worker::QNWorker<TimedInstruction<WindowInstruction>>,
	root_time:std::time::Instant
	)->Result<(),winit::error::EventLoopError>{
		event_loop.run(move |event,elwt|{
			let time=integer::Time::from_nanos(root_time.elapsed().as_nanos() as i64);
			// *control_flow=if cfg!(feature="metal-auto-capture"){
			// 	winit::event_loop::ControlFlow::Exit
			// }else{
			// 	winit::event_loop::ControlFlow::Poll
			// };
			match event{
				winit::event::Event::AboutToWait=>{
					window_thread.send(TimedInstruction{time,instruction:WindowInstruction::RequestRedraw}).unwrap();
				}
				winit::event::Event::WindowEvent {
					event:
						// WindowEvent::Resized(size)
						// | WindowEvent::ScaleFactorChanged {
						// 	new_inner_size: &mut size,
						// 	..
						// },
						winit::event::WindowEvent::Resized(size),//ignoring scale factor changed for now because mutex bruh
					window_id:_,
				} => {
					window_thread.send(TimedInstruction{time,instruction:WindowInstruction::Resize(size)}).unwrap();
				}
				winit::event::Event::WindowEvent{event,..}=>match event{
					winit::event::WindowEvent::KeyboardInput{
						event:
							winit::event::KeyEvent {
							logical_key: winit::keyboard::Key::Named(winit::keyboard::NamedKey::Escape),
								state: winit::event::ElementState::Pressed,
								..
							},
						..
					}
					|winit::event::WindowEvent::CloseRequested=>{
						elwt.exit();
					}
					winit::event::WindowEvent::RedrawRequested=>{
						window_thread.send(TimedInstruction{time,instruction:WindowInstruction::Render}).unwrap();
					}
					_=>{
						window_thread.send(TimedInstruction{time,instruction:WindowInstruction::WindowEvent(event)}).unwrap();
					}
				},
				winit::event::Event::DeviceEvent{
					event,
					..
				} => {
					window_thread.send(TimedInstruction{time,instruction:WindowInstruction::DeviceEvent(event)}).unwrap();
				},
				_=>{}
			}
		})
}
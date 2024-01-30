use crate::physics_worker::InputInstruction;
use strafesnet_common::integer;
use strafesnet_common::instruction::TimedInstruction;

pub enum WindowInstruction{
	Resize(winit::dpi::PhysicalSize<u32>),
	WindowEvent(winit::event::WindowEvent),
	DeviceEvent(winit::event::DeviceEvent),
	RequestRedraw,
	Render,
}

//holds thread handles to dispatch to
struct WindowContext<'a>{
	manual_mouse_lock:bool,
	mouse:crate::physics::MouseState,//std::sync::Arc<std::sync::Mutex<>>
	screen_size:glam::UVec2,
	user_settings:crate::settings::UserSettings,
	window:&'a winit::window::Window,
	physics_thread:crate::compat_worker::QNWorker<'a, TimedInstruction<crate::physics_worker::Instruction>>,
}

impl WindowContext<'_>{
	fn get_middle_of_screen(&self)->winit::dpi::PhysicalPosition<f32>{
		winit::dpi::PhysicalPosition::new(self.screen_size.x as f32/2.0,self.screen_size.y as f32/2.0)
	}
	fn window_event(&mut self,time:integer::Time,event: winit::event::WindowEvent) {
		match event {
			winit::event::WindowEvent::DroppedFile(path)=>{
				//blocking because it's simpler...
				if let Some(indexed_model_instances)=crate::load_file(path){
					self.physics_thread.send(TimedInstruction{time,instruction:crate::physics_worker::Instruction::ClearModels}).unwrap();
					self.physics_thread.send(TimedInstruction{time,instruction:crate::physics_worker::Instruction::GenerateModels(indexed_model_instances)}).unwrap();
				}
			},
			winit::event::WindowEvent::Focused(_state)=>{
				//pause unpause
				//recalculate pressed keys on focus
			},
			winit::event::WindowEvent::KeyboardInput{
				event:winit::event::KeyEvent{state,logical_key,repeat:false,..},
				..
			}=>{
				let s=match state{
					winit::event::ElementState::Pressed=>true,
					winit::event::ElementState::Released=>false,
				};
				match logical_key{
					winit::keyboard::Key::Named(winit::keyboard::NamedKey::Tab)=>{
						if s{
							self.manual_mouse_lock=false;
							match self.window.set_cursor_position(self.get_middle_of_screen()){
								Ok(())=>(),
								Err(e)=>println!("Could not set cursor position: {:?}",e),
							}
							match self.window.set_cursor_grab(winit::window::CursorGrabMode::None){
								Ok(())=>(),
								Err(e)=>println!("Could not release cursor: {:?}",e),
							}
						}else{
							//if cursor is outside window don't lock but apparently there's no get pos function
							//let pos=window.get_cursor_pos();
							match self.window.set_cursor_grab(winit::window::CursorGrabMode::Locked){
								Ok(())=>(),
								Err(_)=>{
									match self.window.set_cursor_grab(winit::window::CursorGrabMode::Confined){
										Ok(())=>(),
										Err(e)=>{
											self.manual_mouse_lock=true;
											println!("Could not confine cursor: {:?}",e)
										},
									}
								}
							}
						}
						self.window.set_cursor_visible(s);
					},
					winit::keyboard::Key::Named(winit::keyboard::NamedKey::F11)=>{
						if s{
							if self.window.fullscreen().is_some(){
								self.window.set_fullscreen(None);
							}else{
								self.window.set_fullscreen(Some(winit::window::Fullscreen::Borderless(None)));
							}
						}
					},
					winit::keyboard::Key::Named(winit::keyboard::NamedKey::Escape)=>{
						if s{
							self.manual_mouse_lock=false;
							match self.window.set_cursor_grab(winit::window::CursorGrabMode::None){
								Ok(())=>(),
								Err(e)=>println!("Could not release cursor: {:?}",e),
							}
							self.window.set_cursor_visible(true);
						}
					},
					keycode=>{
						if let Some(input_instruction)=match keycode{
							winit::keyboard::Key::Named(winit::keyboard::NamedKey::Space)=>Some(InputInstruction::Jump(s)),
							winit::keyboard::Key::Character(key)=>match key.as_str(){
								"w"=>Some(InputInstruction::MoveForward(s)),
								"a"=>Some(InputInstruction::MoveLeft(s)),
								"s"=>Some(InputInstruction::MoveBack(s)),
								"d"=>Some(InputInstruction::MoveRight(s)),
								"e"=>Some(InputInstruction::MoveUp(s)),
								"q"=>Some(InputInstruction::MoveDown(s)),
								"z"=>Some(InputInstruction::Zoom(s)),
								"r"=>if s{Some(InputInstruction::Reset)}else{None},
								_=>None,
							},
							_=>None,
						}{
							self.physics_thread.send(TimedInstruction{
								time,
								instruction:crate::physics_worker::Instruction::Input(input_instruction),
							}).unwrap();
						}
					},
				}
			},
			_=>(),
		}
	}

	fn device_event(&mut self,time:integer::Time,event: winit::event::DeviceEvent) {
		match event {
			winit::event::DeviceEvent::MouseMotion {
			    delta,//these (f64,f64) are integers on my machine
			} => {
				if self.manual_mouse_lock{
					match self.window.set_cursor_position(self.get_middle_of_screen()){
						Ok(())=>(),
						Err(e)=>println!("Could not set cursor position: {:?}",e),
					}
				}
				//do not step the physics because the mouse polling rate is higher than the physics can run.
				//essentially the previous input will be overwritten until a true step runs
				//which is fine because they run all the time.
				let delta=glam::ivec2(delta.0 as i32,delta.1 as i32);
				self.mouse.pos+=delta;
				self.physics_thread.send(TimedInstruction{
					time,
					instruction:crate::physics_worker::Instruction::Input(InputInstruction::MoveMouse(self.mouse.pos)),
				}).unwrap();
			},
			winit::event::DeviceEvent::MouseWheel {
			    delta,
			} => {
				println!("mousewheel {:?}",delta);
				if false{//self.physics.style.use_scroll{
					self.physics_thread.send(TimedInstruction{
						time,
						instruction:crate::physics_worker::Instruction::Input(InputInstruction::Jump(true)),//activates the immediate jump path, but the style modifier prevents controls&CONTROL_JUMP bit from being set to auto jump
					}).unwrap();
				}
			}
			_=>(),
		}
	}
}

pub struct WindowContextSetup<'a>{
	user_settings:crate::settings::UserSettings,
	window:&'a winit::window::Window,
	physics:crate::physics::PhysicsState,
	graphics:crate::graphics::GraphicsState,
}

impl<'a> WindowContextSetup<'a>{
	pub fn new(context:&crate::setup::SetupContext,window:&'a winit::window::Window)->Self{
		//wee
		let user_settings=crate::settings::read_user_settings();

		let args:Vec<String>=std::env::args().collect();
		let indexed_model_instances=if args.len()==2{
			crate::load_file(std::path::PathBuf::from(&args[1]))
		}else{
			None
		}.unwrap_or(crate::default_models());

		let mut physics=crate::physics::PhysicsState::default();
		physics.load_user_settings(&user_settings);
		physics.generate_models(&indexed_model_instances);
		physics.spawn(indexed_model_instances.spawn_point);

		let mut graphics=crate::graphics::GraphicsState::new(&context.device,&context.queue,&context.config);
		graphics.load_user_settings(&user_settings);
		graphics.generate_models(&context.device,&context.queue,indexed_model_instances);

		Self{
			user_settings,
			window,
			graphics,
			physics,
		}
	}

	fn into_context(self,setup_context:crate::setup::SetupContext<'a>)->WindowContext<'a>{
		let screen_size=glam::uvec2(setup_context.config.width,setup_context.config.height);
		let graphics_thread=crate::graphics_worker::new(self.graphics,setup_context.config,setup_context.surface,setup_context.device,setup_context.queue);
		WindowContext{
			manual_mouse_lock:false,
			mouse:crate::physics::MouseState::default(),
			//make sure to update this!!!!!
			screen_size,
			user_settings:self.user_settings,
			window:self.window,
			physics_thread:crate::physics_worker::new(self.physics,graphics_thread),
		}
	}

	pub fn into_worker(self,setup_context:crate::setup::SetupContext<'a>)->crate::compat_worker::QNWorker<'a,TimedInstruction<WindowInstruction>>{
		let mut window_context=self.into_context(setup_context);
		crate::compat_worker::QNWorker::new(move |ins:TimedInstruction<WindowInstruction>|{
			match ins.instruction{
				WindowInstruction::RequestRedraw=>{
					window_context.window.request_redraw();
				}
				WindowInstruction::WindowEvent(window_event)=>{
					window_context.window_event(ins.time,window_event);
				},
				WindowInstruction::DeviceEvent(device_event)=>{
					window_context.device_event(ins.time,device_event);
				},
				WindowInstruction::Resize(size)=>{
					window_context.physics_thread.send(
						TimedInstruction{
							time:ins.time,
							instruction:crate::physics_worker::Instruction::Resize(size,window_context.user_settings.clone())
						}
					).unwrap();
				}
				WindowInstruction::Render=>{
					window_context.physics_thread.send(
						TimedInstruction{
							time:ins.time,
							instruction:crate::physics_worker::Instruction::Render
						}
					).unwrap();
				}
			}
		})
	}
}
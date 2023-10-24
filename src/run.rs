use crate::physics::PhysicsInstruction;
use crate::render_thread::InputInstruction;
use crate::instruction::{TimedInstruction, InstructionConsumer};

pub enum RunInstruction{
	Resize(winit::dpi::PhysicalSize<u32>),
	WindowEvent(winit::event::WindowEvent),
	DeviceEvent(winit::event::DeviceEvent),
	Render,
}

pub struct RunState{
	manual_mouse_lock:bool,
	mouse:std::sync::Arc<std::sync::Mutex<crate::physics::MouseState>>,
	user_settings:crate::settings::UserSettings,
	//Ideally the graphics thread worker description is:
	/*
	WorkerDescription{
		input:Immediate,
		output:Realtime(PoolOrdering::Ordered(3)),
	}
	*/
	//up to three frames in flight, dropping new frame requests when all three are busy, and dropping output frames when one renders out of order
	graphics_thread:crate::worker::INWorker<crate::graphics::GraphicsInstruction>,
	physics_thread:crate::worker::QNWorker<TimedInstruction<InputInstruction>>,
}

impl RunState {
	fn init() -> Self {
		//wee
		let user_settings=crate::settings::read_user_settings();

		let mut graphics=GraphicsState::new();

		graphics.load_user_settings(&user_settings);

		//how to multithread

		//1. build
		physics.generate_models(&indexed_model_instances);

		//2. move
		let physics_thread=physics.into_worker();

		//3. forget

		let mut state=Self{
			manual_mouse_lock:false,
			mouse:physics::MouseState::default(),
			user_settings,
			graphics,
			physics_thread,
		};
		state.generate_model_graphics(&device,&queue,indexed_model_instances);

		let args:Vec<String>=std::env::args().collect();
		if args.len()==2{
			let indexed_model_instances=load_file(std::path::PathBuf::from(&args[1]));
			state.render_thread=RenderThread::new(user_settings,indexed_model_instances);
		}

		return state;
	}
	fn window_event(&mut self, time:crate::integer::Time, event: winit::event::WindowEvent) {
		match event {
			winit::event::WindowEvent::DroppedFile(path)=>{
				std::thread::spawn(move ||{
					let indexed_model_instances=load_file(path);
					self.render_thread.send(Instruction::Die(indexed_model_instances));
				});
			},
			winit::event::WindowEvent::Focused(state)=>{
				//pause unpause
				//recalculate pressed keys on focus
			},
			winit::event::WindowEvent::KeyboardInput {
				input:winit::event::KeyboardInput{state, virtual_keycode,..},
				..
			}=>{
				let s=match state {
					winit::event::ElementState::Pressed => true,
					winit::event::ElementState::Released => false,
				};
				match virtual_keycode{
					Some(winit::event::VirtualKeyCode::Tab)=>{
						if s{
							self.manual_mouse_lock=false;
							match window.set_cursor_position(winit::dpi::PhysicalPosition::new(self.graphics.camera.screen_size.x as f32/2.0, self.graphics.camera.screen_size.y as f32/2.0)){
								Ok(())=>(),
								Err(e)=>println!("Could not set cursor position: {:?}",e),
							}
							match window.set_cursor_grab(winit::window::CursorGrabMode::None){
								Ok(())=>(),
								Err(e)=>println!("Could not release cursor: {:?}",e),
							}
						}else{
							//if cursor is outside window don't lock but apparently there's no get pos function
							//let pos=window.get_cursor_pos();
							match window.set_cursor_grab(winit::window::CursorGrabMode::Locked){
								Ok(())=>(),
								Err(_)=>{
									match window.set_cursor_grab(winit::window::CursorGrabMode::Confined){
										Ok(())=>(),
										Err(e)=>{
											self.manual_mouse_lock=true;
											println!("Could not confine cursor: {:?}",e)
										},
									}
								}
							}
						}
						window.set_cursor_visible(s);
					},
					Some(winit::event::VirtualKeyCode::F11)=>{
						if s{
							if window.fullscreen().is_some(){
								window.set_fullscreen(None);
							}else{
								window.set_fullscreen(Some(winit::window::Fullscreen::Borderless(None)));
							}
						}
					},
					Some(winit::event::VirtualKeyCode::Escape)=>{
						if s{
							self.manual_mouse_lock=false;
							match window.set_cursor_grab(winit::window::CursorGrabMode::None){
								Ok(())=>(),
								Err(e)=>println!("Could not release cursor: {:?}",e),
							}
							window.set_cursor_visible(true);
						}
					},
					Some(keycode)=>{
						if let Some(input_instruction)=match keycode {
							winit::event::VirtualKeyCode::W => Some(InputInstruction::MoveForward(s)),
							winit::event::VirtualKeyCode::A => Some(InputInstruction::MoveLeft(s)),
							winit::event::VirtualKeyCode::S => Some(InputInstruction::MoveBack(s)),
							winit::event::VirtualKeyCode::D => Some(InputInstruction::MoveRight(s)),
							winit::event::VirtualKeyCode::E => Some(InputInstruction::MoveUp(s)),
							winit::event::VirtualKeyCode::Q => Some(InputInstruction::MoveDown(s)),
							winit::event::VirtualKeyCode::Space => Some(InputInstruction::Jump(s)),
							winit::event::VirtualKeyCode::Z => Some(InputInstruction::Zoom(s)),
							winit::event::VirtualKeyCode::R => if s{Some(InputInstruction::Reset)}else{None},
							_ => None,
						}{
							self.physics_thread.send(TimedInstruction{
								time,
								instruction:input_instruction,
							}).unwrap();
						}
					},
					_=>(),
				}
			},
			_=>(),
		}
	}

	fn device_event(&mut self, time:crate::integer::Time, event: winit::event::DeviceEvent) {
		match event {
			winit::event::DeviceEvent::MouseMotion {
			    delta,//these (f64,f64) are integers on my machine
			} => {
				if self.manual_mouse_lock{
					match window.set_cursor_position(winit::dpi::PhysicalPosition::new(self.graphics.camera.screen_size.x as f32/2.0, self.graphics.camera.screen_size.y as f32/2.0)){
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
					instruction:InputInstruction::MoveMouse(self.mouse.pos),
				}).unwrap();
			},
			winit::event::DeviceEvent::MouseWheel {
			    delta,
			} => {
				println!("mousewheel {:?}",delta);
				if false{//self.physics.style.use_scroll{
					self.physics_thread.send(TimedInstruction{
						time,
						instruction:InputInstruction::Jump(true),//activates the immediate jump path, but the style modifier prevents controls&CONTROL_JUMP bit from being set to auto jump
					}).unwrap();
				}
			}
			_=>(),
		}
	}

	pub fn into_worker(self,mut setup_context:crate::setup_context::SetupContext)->crate::worker::QNWorker<TimedInstruction<RunInstruction>>{
		//create child context
		let physics_context=crate::physics_context::Context::new(indexed_models,&setup_context);//this needs all the context for graphics_context too
		let physics_thread=physics_context.into_worker();
		//
		crate::worker::QNWorker::new(move |ins:TimedInstruction<RunInstruction>|{
			match ins.instruction{
				RunInstruction::WindowEvent(window_event)=>{
					self.window_event(ins.time,window_event);
				},
				RunInstruction::DeviceEvent(device_event)=>{
					self.device_event(ins.time,device_event);
				},
				RunInstruction::Resize(size)=>{
					setup_context.config.width=size.width.max(1);
					setup_context.config.height=size.height.max(1);
					setup_context.surface.configure(&setup_context.device,&setup_context.config);
					physics_thread.send(TimedInstruction{time:ins.time,instruction:PhysicsInstruction::Resize(size)});
				}
				RunInstruction::Render=>{
					let frame=match setup_context.surface.get_current_texture(){
						Ok(frame)=>frame,
						Err(_)=>{
							setup_context.surface.configure(&setup_context.device,&setup_context.config);
							setup_context.surface
								.get_current_texture()
								.expect("Failed to acquire next surface texture!")
						}
					};
					let view=frame.texture.create_view(&wgpu::TextureViewDescriptor{
						format:Some(setup_context.config.view_formats[0]),
						..wgpu::TextureViewDescriptor::default()
					});

					physics_thread.send(TimedInstruction{time:ins.time,instruction:PhysicsInstruction::Render(view)});

					frame.present();
				}
			}
		})
	}
}
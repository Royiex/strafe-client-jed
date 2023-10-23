pub enum WindowInstruction{
	Resize(),
}
pub struct WindowState{
	//ok
}
impl WindowState{
	fn resize(&mut self);
	fn render(&self);

	fn window_event(&mut self, window: &winit::window::Window, event: winit::event::WindowEvent) {
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

	fn device_event(&mut self, window: &winit::window::Window, event: winit::event::DeviceEvent) {
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

	pub fn create_window(title:&str,event_loop:&winit::event_loop::EventLoop<()>)->Result<winit::window::Window,winit::error::OsError>{
		let mut builder = winit::window::WindowBuilder::new();
		builder = builder.with_title(title);
		#[cfg(windows_OFF)] // TODO
		{
			use winit::platform::windows::WindowBuilderExtWindows;
			builder = builder.with_no_redirection_bitmap(true);
		}
		builder.build(event_loop)
	}
	pub fn into_thread(window:winit::window::Window){
		//
	}
}
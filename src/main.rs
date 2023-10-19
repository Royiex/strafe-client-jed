use std::{borrow::Cow, time::Instant};
use wgpu::{util::DeviceExt, AstcBlock, AstcChannel};
use model_graphics::{GraphicsVertex,ModelGraphicsInstance};
use physics::{InputInstruction, PhysicsInstruction};
use instruction::{TimedInstruction, InstructionConsumer};

mod bvh;
mod aabb;
mod model;
mod model_graphics;
mod zeroes;
mod worker;
mod integer;
mod physics;
mod graphics;
mod settings;
mod framework;
mod primitives;
mod instruction;
mod load_roblox;


pub struct GlobalState{
	start_time: std::time::Instant,
	manual_mouse_lock:bool,
	mouse:physics::MouseState,
	user_settings:settings::UserSettings,
	graphics:graphics::GraphicsState,
	physics_thread:worker::CompatWorker<TimedInstruction<InputInstruction>,physics::PhysicsOutputState,Box<dyn FnMut(TimedInstruction<InputInstruction>)->physics::PhysicsOutputState>>,
}

impl framework::Example for GlobalState {
	fn optional_features() -> wgpu::Features {
		wgpu::Features::TEXTURE_COMPRESSION_ASTC
			| wgpu::Features::TEXTURE_COMPRESSION_ETC2
	}
	fn required_features() -> wgpu::Features {
		wgpu::Features::TEXTURE_COMPRESSION_BC
	}
	fn required_limits() -> wgpu::Limits {
		wgpu::Limits::default() //framework.rs was using goofy limits that caused me a multi-day headache
	}
	fn init(
		config: &wgpu::SurfaceConfiguration,
		_adapter: &wgpu::Adapter,
		device: &wgpu::Device,
		queue: &wgpu::Queue,
	) -> Self {
		//wee
		let user_settings=settings::read_user_settings();
		let mut indexed_models = Vec::new();
		indexed_models.append(&mut model::generate_indexed_model_list_from_obj(obj::ObjData::load_buf(&include_bytes!("../models/teslacyberv3.0.obj")[..]).unwrap(),glam::Vec4::ONE));
		indexed_models.push(primitives::unit_sphere());
		indexed_models.push(primitives::unit_cylinder());
		indexed_models.push(primitives::unit_cube());
		println!("models.len = {:?}", indexed_models.len());
		indexed_models[0].instances.push(model::ModelInstance{
			transform:integer::Planar64Affine3::try_from(glam::Affine3A::from_translation(glam::vec3(10.,0.,-10.))).unwrap(),
			..Default::default()
		});
		//quad monkeys
		indexed_models[1].instances.push(model::ModelInstance{
			transform:integer::Planar64Affine3::try_from(glam::Affine3A::from_translation(glam::vec3(10.,5.,10.))).unwrap(),
			..Default::default()
		});
		indexed_models[1].instances.push(model::ModelInstance{
			transform:integer::Planar64Affine3::try_from(glam::Affine3A::from_translation(glam::vec3(20.,5.,10.))).unwrap(),
			color:glam::vec4(1.0,0.0,0.0,1.0),
			..Default::default()
		});
		indexed_models[1].instances.push(model::ModelInstance{
			transform:integer::Planar64Affine3::try_from(glam::Affine3A::from_translation(glam::vec3(10.,5.,20.))).unwrap(),
			color:glam::vec4(0.0,1.0,0.0,1.0),
			..Default::default()
		});
		indexed_models[1].instances.push(model::ModelInstance{
			transform:integer::Planar64Affine3::try_from(glam::Affine3A::from_translation(glam::vec3(20.,5.,20.))).unwrap(),
			color:glam::vec4(0.0,0.0,1.0,1.0),
			..Default::default()
		});
		//decorative monkey
		indexed_models[1].instances.push(model::ModelInstance{
			transform:integer::Planar64Affine3::try_from(glam::Affine3A::from_translation(glam::vec3(15.,10.,15.))).unwrap(),
			color:glam::vec4(0.5,0.5,0.5,0.5),
			attributes:model::CollisionAttributes::Decoration,
			..Default::default()
		});
		//teapot
		indexed_models[2].instances.push(model::ModelInstance{
			transform:integer::Planar64Affine3::try_from(glam::Affine3A::from_scale_rotation_translation(glam::vec3(0.5, 1.0, 0.2),glam::quat(-0.22248298016985793,-0.839457167990537,-0.05603504040830783,-0.49261857546227916),glam::vec3(-10.,7.,10.))).unwrap(),
			..Default::default()
		});
		//ground
		indexed_models[3].instances.push(model::ModelInstance{
			transform:integer::Planar64Affine3::try_from(glam::Affine3A::from_translation(glam::vec3(0.,0.,0.))*glam::Affine3A::from_scale(glam::vec3(160.0, 1.0, 160.0))).unwrap(),
			..Default::default()
		});

		let mut graphics=GraphicsState::new();

		graphics.load_user_settings(&user_settings);

		let indexed_model_instances=model::IndexedModelInstances{
			textures:Vec::new(),
			models:indexed_models,
			spawn_point:integer::Planar64Vec3::Y*50,
			modes:Vec::new(),
		};

		//how to multithread

		//1. build
		physics.generate_models(&indexed_model_instances);

		//2. move
		let physics_thread=physics.into_worker();

		//3. forget

		let mut state=GlobalState{
			start_time:Instant::now(),
			manual_mouse_lock:false,
			mouse:physics::MouseState::default(),
			user_settings,
			graphics,
			physics_thread,
		};
		state.generate_model_graphics(&device,&queue,indexed_model_instances);

		let args:Vec<String>=std::env::args().collect();
		if args.len()==2{
			state.load_file(std::path::PathBuf::from(&args[1]), device, queue);
		}

		return state;
	}

	fn load_file(&mut self,path: std::path::PathBuf, device: &wgpu::Device, queue: &wgpu::Queue){
		println!("Loading file: {:?}", &path);
		//oh boy! let's load the map!
		if let Ok(file)=std::fs::File::open(path){
			let mut input = std::io::BufReader::new(file);
			let mut first_8=[0u8;8];
			//.rbxm roblox binary = "<roblox!"
			//.rbxmx roblox xml = "<roblox "
			//.bsp = "VBSP"
			//.vmf = 
			//.snf = "SNMF"
			//.snf = "SNBF"
			if let (Ok(()),Ok(()))=(std::io::Read::read_exact(&mut input, &mut first_8),std::io::Seek::rewind(&mut input)){
				if let Some(indexed_model_instances)={
					match &first_8[0..4]{
						b"<rob"=>{
							match match &first_8[4..8]{
								b"lox!"=>rbx_binary::from_reader(input).map_err(|e|format!("{:?}",e)),
								b"lox "=>rbx_xml::from_reader(input,rbx_xml::DecodeOptions::default()).map_err(|e|format!("{:?}",e)),
								other=>Err(format!("Unknown Roblox file type {:?}",other)),
							}{
								Ok(dom)=>Some(load_roblox::generate_indexed_models(dom)),
								Err(e)=>{
									println!("Error loading roblox file:{:?}",e);
									None
								},
							}
						},
						//b"VBSP"=>Some(load_bsp::generate_indexed_models(input)),
						//b"SNFM"=>Some(sniffer::generate_indexed_models(input)),
						//b"SNFB"=>Some(sniffer::load_bot(input)),
						other=>{
							println!("loser file {:?}",other);
							None
						},
					}
				}{
					let spawn_point=indexed_model_instances.spawn_point;
					//if generate_indexed_models succeeds, clear the previous ones
					self.graphics.clear();

					let mut physics=physics::PhysicsState::default();
					//physics.spawn()
					physics.game.stage_id=0;
					physics.spawn_point=spawn_point;
					physics.process_instruction(instruction::TimedInstruction{
						time:physics.time,
						instruction: PhysicsInstruction::Input(physics::PhysicsInputInstruction::Reset),
					});
					physics.load_user_settings(&self.user_settings);
					physics.generate_models(&indexed_model_instances);
					self.physics_thread=physics.into_worker();

					//graphics.load_user_settings(&self.user_settings);
					self.generate_model_graphics(device,queue,indexed_model_instances);
					//manual reset
				}else{
					println!("No modeldatas were generated");
				}
			}else{
				println!("Failed to read first 8 bytes and seek back to beginning of file.");
			}
		}else{
			println!("Could not open file");
		}
	}

	#[allow(clippy::single_match)]
	fn update(&mut self, window: &winit::window::Window, device: &wgpu::Device, queue: &wgpu::Queue, event: winit::event::WindowEvent) {
		let time=integer::Time::from_nanos(self.start_time.elapsed().as_nanos() as i64);
		match event {
			winit::event::WindowEvent::DroppedFile(path) => self.load_file(path,device,queue),
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
		//there's no way this is the best way get a timestamp.
		let time=integer::Time::from_nanos(self.start_time.elapsed().as_nanos() as i64);
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
}

fn main() {
	framework::run::<GlobalState>(
		format!("Strafe Client v{}",
			env!("CARGO_PKG_VERSION")
		).as_str()
	);
}

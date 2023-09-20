use bytemuck::{Pod, Zeroable};
use strafe_client::{instruction::{TimedInstruction, InstructionConsumer},body::{InputInstruction, PhysicsInstruction}};
use std::{borrow::Cow, time::Instant};
use wgpu::{util::DeviceExt, AstcBlock, AstcChannel};

const IMAGE_SIZE: u32 = 128;

#[derive(Clone, Copy, Pod, Zeroable)]
#[repr(C)]
struct Vertex {
	pos: [f32; 3],
	texture: [f32; 2],
	normal: [f32; 3],
}

struct Entity {
	index_count: u32,
	index_buf: wgpu::Buffer,
}

//temp?
struct ModelData {
	transform: glam::Mat4,
	vertex_buf: wgpu::Buffer,
	entities: Vec<Entity>,
}

struct ModelGraphics {
	transform: glam::Mat4,
	vertex_buf: wgpu::Buffer,
	entities: Vec<Entity>,
	bind_group: wgpu::BindGroup,
	model_buf: wgpu::Buffer,
}

pub struct Skybox {
	start_time: std::time::Instant,
	screen_size: (u32, u32),
	physics: strafe_client::body::PhysicsState,
	sky_pipeline: wgpu::RenderPipeline,
	entity_pipeline: wgpu::RenderPipeline,
	ground_pipeline: wgpu::RenderPipeline,
	main_bind_group: wgpu::BindGroup,
	camera_buf: wgpu::Buffer,
	models: Vec<ModelGraphics>,
	depth_view: wgpu::TextureView,
	staging_belt: wgpu::util::StagingBelt,
}

impl Skybox {
	const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth24Plus;

	fn create_depth_texture(
		config: &wgpu::SurfaceConfiguration,
		device: &wgpu::Device,
	) -> wgpu::TextureView {
		let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
			size: wgpu::Extent3d {
				width: config.width,
				height: config.height,
				depth_or_array_layers: 1,
			},
			mip_level_count: 1,
			sample_count: 1,
			dimension: wgpu::TextureDimension::D2,
			format: Self::DEPTH_FORMAT,
			usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
			label: None,
			view_formats: &[],
		});

		depth_texture.create_view(&wgpu::TextureViewDescriptor::default())
	}
}

fn get_transform_uniform_data(transform:&glam::Mat4) -> [f32; 4*4] {
	let mut raw = [0f32; 4*4];
	raw[0..16].copy_from_slice(&AsRef::<[f32; 4*4]>::as_ref(transform)[..]);
	raw
}

fn add_obj(device:&wgpu::Device,modeldatas:& mut Vec<ModelData>,source:&[u8]){
	let data = obj::ObjData::load_buf(&source[..]).unwrap();
	let mut vertices = Vec::new();
	let mut vertex_index = std::collections::HashMap::<obj::IndexTuple,u16>::new();
	for object in data.objects {
		let mut entities = Vec::<Entity>::new();
		for group in object.groups {
			let mut indices = Vec::new();
			for poly in group.polys {
				for end_index in 2..poly.0.len() {
					for &index in &[0, end_index - 1, end_index] {
						let vert = poly.0[index];
						if let Some(&i)=vertex_index.get(&vert){
							indices.push(i);
						}else{
							let i=vertices.len() as u16;
							vertices.push(Vertex {
								pos: data.position[vert.0],
								texture: data.texture[vert.1.unwrap()],
								normal: data.normal[vert.2.unwrap()],
							});
							vertex_index.insert(vert,i);
							indices.push(i);
						}
					}
				}
			}
			let index_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
				label: Some("Index"),
				contents: bytemuck::cast_slice(&indices),
				usage: wgpu::BufferUsages::INDEX,
			});
			entities.push(Entity {
				index_buf,
				index_count: indices.len() as u32,
			});
		}
		let vertex_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("Vertex"),
			contents: bytemuck::cast_slice(&vertices),
			usage: wgpu::BufferUsages::VERTEX,
		});
		modeldatas.push(ModelData {
			transform: glam::Mat4::default(),
			vertex_buf,
			entities,
		})
	}
}


fn to_uniform_data(camera: &strafe_client::body::Camera, pos: glam::Vec3) -> [f32; 16 * 3 + 4] {
	let proj=camera.proj();
	let proj_inv = proj.inverse();
	let view=camera.view(pos);
	let view_inv = view.inverse();

	let mut raw = [0f32; 16 * 3 + 4];
	raw[..16].copy_from_slice(&AsRef::<[f32; 16]>::as_ref(&proj)[..]);
	raw[16..32].copy_from_slice(&AsRef::<[f32; 16]>::as_ref(&proj_inv)[..]);
	raw[32..48].copy_from_slice(&AsRef::<[f32; 16]>::as_ref(&view_inv)[..]);
	raw[48..52].copy_from_slice(AsRef::<[f32; 4]>::as_ref(&view.col(3)));
	raw
}

impl strafe_client::framework::Example for Skybox {
	fn optional_features() -> wgpu::Features {
		wgpu::Features::TEXTURE_COMPRESSION_ASTC
			| wgpu::Features::TEXTURE_COMPRESSION_ETC2
			| wgpu::Features::TEXTURE_COMPRESSION_BC
	}

	fn init(
		config: &wgpu::SurfaceConfiguration,
		_adapter: &wgpu::Adapter,
		device: &wgpu::Device,
		queue: &wgpu::Queue,
	) -> Self {
		let mut modeldatas = Vec::<ModelData>::new();
		add_obj(device,& mut modeldatas,include_bytes!("../models/teslacyberv3.0.obj"));
		add_obj(device,& mut modeldatas,include_bytes!("../models/suzanne.obj"));
		add_obj(device,& mut modeldatas,include_bytes!("../models/teapot.obj"));
		println!("models.len = {:?}", modeldatas.len());
		modeldatas[1].transform=glam::Mat4::from_translation(glam::vec3(10.,5.,10.));
		modeldatas[2].transform=glam::Mat4::from_translation(glam::vec3(-10.,5.,10.));

		let main_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			label: None,
			entries: &[
				wgpu::BindGroupLayoutEntry {
					binding: 0,
					visibility: wgpu::ShaderStages::VERTEX,
					ty: wgpu::BindingType::Buffer {
						ty: wgpu::BufferBindingType::Uniform,
						has_dynamic_offset: false,
						min_binding_size: None,
					},
					count: None,
				},
				wgpu::BindGroupLayoutEntry {
					binding: 1,
					visibility: wgpu::ShaderStages::FRAGMENT,
					ty: wgpu::BindingType::Texture {
						sample_type: wgpu::TextureSampleType::Float { filterable: true },
						multisampled: false,
						view_dimension: wgpu::TextureViewDimension::Cube,
					},
					count: None,
				},
				wgpu::BindGroupLayoutEntry {
					binding: 2,
					visibility: wgpu::ShaderStages::FRAGMENT,
					ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
					count: None,
				},
			],
		});
		let model_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			label: None,
			entries: &[
				wgpu::BindGroupLayoutEntry {
					binding: 0,
					visibility: wgpu::ShaderStages::VERTEX,
					ty: wgpu::BindingType::Buffer {
						ty: wgpu::BufferBindingType::Uniform,
						has_dynamic_offset: false,
						min_binding_size: None,
					},
					count: None,
				},
			],
		});

		// Create the render pipeline
		let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
			label: None,
			source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(include_str!("shader.wgsl"))),
		});

		let physics = strafe_client::body::PhysicsState {
			body: strafe_client::body::Body::with_pva(glam::vec3(0.0,50.0,0.0),glam::vec3(0.0,0.0,0.0),glam::vec3(0.0,-100.0,0.0)),
			time: 0,
			tick: 0,
			strafe_tick_num: 100,//100t
			strafe_tick_den: 1_000_000_000,
			gravity: glam::vec3(0.0,-100.0,0.0),
			friction: 1.2,
			walk_accel: 90.0,
			mv: 2.7,
			grounded: false,
			jump_trying: false,
			walkspeed: 18.0,
			contacts: std::collections::HashSet::new(),
			models_cringe_clone: modeldatas.iter().map(|m|strafe_client::body::Model::new(m.transform)).collect(),
			walk: strafe_client::body::WalkState::new(),
			hitbox_halfsize: glam::vec3(1.0,2.5,1.0),
			camera: strafe_client::body::Camera::from_offset(glam::vec3(0.0,4.5-2.5,0.0),(config.width as f32)/(config.height as f32)),
			mouse_interpolation: strafe_client::body::MouseInterpolationState::new(),
			controls: 0,
		};

		let camera_uniforms = to_uniform_data(&physics.camera,physics.body.extrapolated_position(0));
		let camera_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("Camera"),
			contents: bytemuck::cast_slice(&camera_uniforms),
			usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
		});

		//drain the modeldata vec so entities can be /moved/ to models.entities
		let mut models = Vec::<ModelGraphics>::with_capacity(modeldatas.len());
		for (i,modeldata) in modeldatas.drain(..).enumerate() {
			let model_uniforms = get_transform_uniform_data(&modeldata.transform);
			let model_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
				label: Some(format!("ModelGraphics{}",i).as_str()),
				contents: bytemuck::cast_slice(&model_uniforms),
				usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
			});
			let model_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
				layout: &model_bind_group_layout,
				entries: &[
					wgpu::BindGroupEntry {
						binding: 0,
						resource: model_buf.as_entire_binding(),
					},
				],
				label: Some(format!("ModelGraphics{}",i).as_str()),
			});
			//all of these are being moved here
			models.push(ModelGraphics{
				transform: modeldata.transform,
				vertex_buf:modeldata.vertex_buf,
				entities: modeldata.entities,
				bind_group: model_bind_group,
				model_buf,
			})
		}

		let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
			label: None,
			bind_group_layouts: &[&main_bind_group_layout, &model_bind_group_layout],
			push_constant_ranges: &[],
		});

		// Create the render pipelines
		let sky_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
			label: Some("Sky"),
			layout: Some(&pipeline_layout),
			vertex: wgpu::VertexState {
				module: &shader,
				entry_point: "vs_sky",
				buffers: &[],
			},
			fragment: Some(wgpu::FragmentState {
				module: &shader,
				entry_point: "fs_sky",
				targets: &[Some(config.view_formats[0].into())],
			}),
			primitive: wgpu::PrimitiveState {
				front_face: wgpu::FrontFace::Cw,
				..Default::default()
			},
			depth_stencil: Some(wgpu::DepthStencilState {
				format: Self::DEPTH_FORMAT,
				depth_write_enabled: false,
				depth_compare: wgpu::CompareFunction::LessEqual,
				stencil: wgpu::StencilState::default(),
				bias: wgpu::DepthBiasState::default(),
			}),
			multisample: wgpu::MultisampleState::default(),
			multiview: None,
		});
		let entity_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
			label: Some("Entity"),
			layout: Some(&pipeline_layout),
			vertex: wgpu::VertexState {
				module: &shader,
				entry_point: "vs_entity",
				buffers: &[wgpu::VertexBufferLayout {
					array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
					step_mode: wgpu::VertexStepMode::Vertex,
					attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x2, 2 => Float32x3],
				}],
			},
			fragment: Some(wgpu::FragmentState {
				module: &shader,
				entry_point: "fs_entity",
				targets: &[Some(config.view_formats[0].into())],
			}),
			primitive: wgpu::PrimitiveState {
				front_face: wgpu::FrontFace::Cw,
				..Default::default()
			},
			depth_stencil: Some(wgpu::DepthStencilState {
				format: Self::DEPTH_FORMAT,
				depth_write_enabled: true,
				depth_compare: wgpu::CompareFunction::LessEqual,
				stencil: wgpu::StencilState::default(),
				bias: wgpu::DepthBiasState::default(),
			}),
			multisample: wgpu::MultisampleState::default(),
			multiview: None,
		});
		let ground_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
			label: Some("Ground"),
			layout: Some(&pipeline_layout),
			vertex: wgpu::VertexState {
				module: &shader,
				entry_point: "vs_ground",
				buffers: &[],
			},
			fragment: Some(wgpu::FragmentState {
				module: &shader,
				entry_point: "fs_ground",
				targets: &[Some(config.view_formats[0].into())],
			}),
			primitive: wgpu::PrimitiveState {
				front_face: wgpu::FrontFace::Cw,
				..Default::default()
			},
			depth_stencil: Some(wgpu::DepthStencilState {
				format: Self::DEPTH_FORMAT,
				depth_write_enabled: true,
				depth_compare: wgpu::CompareFunction::LessEqual,
				stencil: wgpu::StencilState::default(),
				bias: wgpu::DepthBiasState::default(),
			}),
			multisample: wgpu::MultisampleState::default(),
			multiview: None,
		});

		let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
			label: None,
			address_mode_u: wgpu::AddressMode::ClampToEdge,
			address_mode_v: wgpu::AddressMode::ClampToEdge,
			address_mode_w: wgpu::AddressMode::ClampToEdge,
			mag_filter: wgpu::FilterMode::Linear,
			min_filter: wgpu::FilterMode::Linear,
			mipmap_filter: wgpu::FilterMode::Linear,
			..Default::default()
		});

		let device_features = device.features();

		let skybox_format = if device_features.contains(wgpu::Features::TEXTURE_COMPRESSION_ASTC) {
			log::info!("Using ASTC");
			wgpu::TextureFormat::Astc {
				block: AstcBlock::B4x4,
				channel: AstcChannel::UnormSrgb,
			}
		} else if device_features.contains(wgpu::Features::TEXTURE_COMPRESSION_ETC2) {
			log::info!("Using ETC2");
			wgpu::TextureFormat::Etc2Rgb8UnormSrgb
		} else if device_features.contains(wgpu::Features::TEXTURE_COMPRESSION_BC) {
			log::info!("Using BC");
			wgpu::TextureFormat::Bc1RgbaUnormSrgb
		} else {
			log::info!("Using plain");
			wgpu::TextureFormat::Bgra8UnormSrgb
		};

		let size = wgpu::Extent3d {
			width: IMAGE_SIZE,
			height: IMAGE_SIZE,
			depth_or_array_layers: 6,
		};

		let layer_size = wgpu::Extent3d {
			depth_or_array_layers: 1,
			..size
		};
		let max_mips = layer_size.max_mips(wgpu::TextureDimension::D2);

		log::debug!(
			"Copying {:?} skybox images of size {}, {}, 6 with {} mips to gpu",
			skybox_format,
			IMAGE_SIZE,
			IMAGE_SIZE,
			max_mips,
		);

		let bytes = match skybox_format {
			wgpu::TextureFormat::Astc {
				block: AstcBlock::B4x4,
				channel: AstcChannel::UnormSrgb,
			} => &include_bytes!("../images/astc.dds")[..],
			wgpu::TextureFormat::Etc2Rgb8UnormSrgb => &include_bytes!("../images/etc2.dds")[..],
			wgpu::TextureFormat::Bc1RgbaUnormSrgb => &include_bytes!("../images/bc1.dds")[..],
			wgpu::TextureFormat::Bgra8UnormSrgb => &include_bytes!("../images/bgra.dds")[..],
			_ => unreachable!(),
		};

		let image = ddsfile::Dds::read(&mut std::io::Cursor::new(&bytes)).unwrap();

		let texture = device.create_texture_with_data(
			queue,
			&wgpu::TextureDescriptor {
				size,
				mip_level_count: max_mips,
				sample_count: 1,
				dimension: wgpu::TextureDimension::D2,
				format: skybox_format,
				usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
				label: None,
				view_formats: &[],
			},
			&image.data,
		);

		let texture_view = texture.create_view(&wgpu::TextureViewDescriptor {
			label: None,
			dimension: Some(wgpu::TextureViewDimension::Cube),
			..wgpu::TextureViewDescriptor::default()
		});
		let main_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &main_bind_group_layout,
			entries: &[
				wgpu::BindGroupEntry {
					binding: 0,
					resource: camera_buf.as_entire_binding(),
				},
				wgpu::BindGroupEntry {
					binding: 1,
					resource: wgpu::BindingResource::TextureView(&texture_view),
				},
				wgpu::BindGroupEntry {
					binding: 2,
					resource: wgpu::BindingResource::Sampler(&sampler),
				},
			],
			label: Some("Camera"),
		});

		let depth_view = Self::create_depth_texture(config, device);

		Skybox {
			start_time: Instant::now(),
			screen_size: (config.width,config.height),
			physics,
			sky_pipeline,
			entity_pipeline,
			ground_pipeline,
			main_bind_group,
			camera_buf,
			models,
			depth_view,
			staging_belt: wgpu::util::StagingBelt::new(0x100),
		}
	}

	#[allow(clippy::single_match)]
	fn update(&mut self, event: winit::event::WindowEvent) {
		//nothing atm
	}

	fn device_event(&mut self, event: winit::event::DeviceEvent) {
		//there's no way this is the best way get a timestamp.
		let time=self.start_time.elapsed().as_nanos() as i64;
		self.physics.run(time);//call it a day
		match event {
			winit::event::DeviceEvent::Key(winit::event::KeyboardInput {
				state,
				scancode: keycode,
				..
			}) => {
				let s=match state {
					winit::event::ElementState::Pressed => true,
					winit::event::ElementState::Released => false,
				};
				if let Some(input_instruction)=match keycode {
					17 => Some(InputInstruction::MoveForward(s)),//W
					30 => Some(InputInstruction::MoveLeft(s)),//A
					31 => Some(InputInstruction::MoveBack(s)),//S
					32 => Some(InputInstruction::MoveRight(s)),//D
					18 => Some(InputInstruction::MoveUp(s)),//E
					16 => Some(InputInstruction::MoveDown(s)),//Q
					57 => Some(InputInstruction::Jump(s)),//Space
					44 => Some(InputInstruction::Zoom(s)),//Z
					19 => if s{Some(InputInstruction::Reset)}else{None},//R
					_ => None,
				}
				{
					self.physics.process_instruction(TimedInstruction{
						time,
						instruction:PhysicsInstruction::Input(input_instruction),
					})
				}
			},
			winit::event::DeviceEvent::MouseMotion {
			    delta,//these (f64,f64) are integers on my machine
			} => {
				self.physics.process_instruction(TimedInstruction{
					time,
					instruction:PhysicsInstruction::Input(InputInstruction::MoveMouse(glam::ivec2(delta.0 as i32,delta.1 as i32))),
				})
			},
			winit::event::DeviceEvent::MouseWheel {
			    delta,
			} => {
				println!("mousewheel{:?}",delta);
				if true{//self.physics.use_scroll
					self.physics.process_instruction(TimedInstruction{
						time,
						instruction:PhysicsInstruction::Input(InputInstruction::Jump(true)),//activates the immediate jump path, but the style modifier prevents controls&CONTROL_JUMP bit from being set to auto jump
					})
				}
			}
			_=>(),
		}
	}

	fn resize(
		&mut self,
		config: &wgpu::SurfaceConfiguration,
		device: &wgpu::Device,
		_queue: &wgpu::Queue,
	) {
		self.depth_view = Self::create_depth_texture(config, device);
		self.screen_size = (config.width, config.height);
		self.physics.camera.set_fov_aspect(1.0,(config.width as f32)/(config.height as f32));
	}

	fn render(
		&mut self,
		view: &wgpu::TextureView,
		device: &wgpu::Device,
		queue: &wgpu::Queue,
		_spawner: &strafe_client::framework::Spawner,
	) {
		let time=self.start_time.elapsed().as_nanos() as i64;

		self.physics.run(time);

		let mut encoder =
			device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });

		// update rotation
		let camera_uniforms = to_uniform_data(&self.physics.camera,self.physics.body.extrapolated_position(time));
		self.staging_belt
			.write_buffer(
				&mut encoder,
				&self.camera_buf,
				0,
				wgpu::BufferSize::new((camera_uniforms.len() * 4) as wgpu::BufferAddress).unwrap(),
				device,
			)
			.copy_from_slice(bytemuck::cast_slice(&camera_uniforms));
		//This code only needs to run when the uniforms change
		for model in self.models.iter() {
			let model_uniforms = get_transform_uniform_data(&model.transform);
			self.staging_belt
				.write_buffer(
					&mut encoder,
					&model.model_buf,//description of where data will be written when command is executed
					0,//offset in staging belt?
					wgpu::BufferSize::new((model_uniforms.len() * 4) as wgpu::BufferAddress).unwrap(),
					device,
				)
				.copy_from_slice(bytemuck::cast_slice(&model_uniforms));
		}
		self.staging_belt.finish();

		{
			let mut rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
				label: None,
				color_attachments: &[Some(wgpu::RenderPassColorAttachment {
					view,
					resolve_target: None,
					ops: wgpu::Operations {
						load: wgpu::LoadOp::Clear(wgpu::Color {
							r: 0.1,
							g: 0.2,
							b: 0.3,
							a: 1.0,
						}),
						store: true,
					},
				})],
				depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
					view: &self.depth_view,
					depth_ops: Some(wgpu::Operations {
						load: wgpu::LoadOp::Clear(1.0),
						store: false,
					}),
					stencil_ops: None,
				}),
			});

			rpass.set_bind_group(0, &self.main_bind_group, &[]);

			rpass.set_pipeline(&self.entity_pipeline);
			for model in self.models.iter() {
				rpass.set_bind_group(1, &model.bind_group, &[]);
				rpass.set_vertex_buffer(0, model.vertex_buf.slice(..));

				for entity in model.entities.iter() {
					rpass.set_index_buffer(entity.index_buf.slice(..), wgpu::IndexFormat::Uint16);
					rpass.draw_indexed(0..entity.index_count, 0, 0..1);
				}
			}

			rpass.set_pipeline(&self.ground_pipeline);
			//rpass.set_index_buffer(&[0u16,1,2,1,2,3][..] as wgpu::BufferSlice, wgpu::IndexFormat::Uint16);
			//rpass.draw_indexed(0..4, 0, 0..1);
			rpass.draw(0..6, 0..1);

			rpass.set_pipeline(&self.sky_pipeline);
			rpass.draw(0..3, 0..1);
		}

		queue.submit(std::iter::once(encoder.finish()));

		self.staging_belt.recall();
	}
}

fn main() {
	strafe_client::framework::run::<Skybox>(
		format!("Strafe Client v{}",
			env!("CARGO_PKG_VERSION")
		).as_str()
	);
}

use bytemuck::{Pod, Zeroable};
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
	transforms: Vec<glam::Mat4>,
	vertex_buf: wgpu::Buffer,
	entities: Vec<Entity>,
}

struct ModelGraphics {
	transforms: Vec<glam::Mat4>,
	vertex_buf: wgpu::Buffer,
	entities: Vec<Entity>,
	bind_group: wgpu::BindGroup,
	model_buf: wgpu::Buffer,
}

// Note: we use the Y=up coordinate space in this example.
struct Camera {
	screen_size: (u32, u32),
	offset: glam::Vec3,
	fov: f32,
	yaw: f32,
	pitch: f32,
	controls: u32,
}

const CONTROL_MOVEFORWARD:u32 = 0b00000001;
const CONTROL_MOVEBACK:u32 = 0b00000010;
const CONTROL_MOVERIGHT:u32 = 0b00000100;
const CONTROL_MOVELEFT:u32 = 0b00001000;
const CONTROL_MOVEUP:u32 = 0b00010000;
const CONTROL_MOVEDOWN:u32 = 0b00100000;
const CONTROL_JUMP:u32 = 0b01000000;
const CONTROL_ZOOM:u32 = 0b10000000;

const FORWARD_DIR:glam::Vec3 = glam::Vec3::new(0.0,0.0,-1.0);
const RIGHT_DIR:glam::Vec3 = glam::Vec3::new(1.0,0.0,0.0);
const UP_DIR:glam::Vec3 = glam::Vec3::new(0.0,1.0,0.0);

fn get_control_dir(controls: u32) -> glam::Vec3{
	//don't get fancy just do it
	let mut control_dir:glam::Vec3 = glam::Vec3::new(0.0,0.0,0.0);
	if controls & CONTROL_MOVEFORWARD == CONTROL_MOVEFORWARD {
		control_dir+=FORWARD_DIR;
	}
	if controls & CONTROL_MOVEBACK == CONTROL_MOVEBACK {
		control_dir+=-FORWARD_DIR;
	}
	if controls & CONTROL_MOVELEFT == CONTROL_MOVELEFT {
		control_dir+=-RIGHT_DIR;
	}
	if controls & CONTROL_MOVERIGHT == CONTROL_MOVERIGHT {
		control_dir+=RIGHT_DIR;
	}
	if controls & CONTROL_MOVEUP == CONTROL_MOVEUP {
		control_dir+=UP_DIR;
	}
	if controls & CONTROL_MOVEDOWN == CONTROL_MOVEDOWN {
		control_dir+=-UP_DIR;
	}
	return control_dir
}

	#[inline]
	fn perspective_rh(fov_y_slope: f32, aspect_ratio: f32, z_near: f32, z_far: f32) -> glam::Mat4 {
		//glam_assert!(z_near > 0.0 && z_far > 0.0);
		let r = z_far / (z_near - z_far);
		glam::Mat4::from_cols(
			glam::Vec4::new(1.0/(fov_y_slope * aspect_ratio), 0.0, 0.0, 0.0),
			glam::Vec4::new(0.0, 1.0/fov_y_slope, 0.0, 0.0),
			glam::Vec4::new(0.0, 0.0, r, -1.0),
			glam::Vec4::new(0.0, 0.0, r * z_near, 0.0),
		)
	}

impl Camera {
	fn to_uniform_data(&self, pos: glam::Vec3) -> [f32; 16 * 3 + 4] {
		let aspect = self.screen_size.0 as f32 / self.screen_size.1 as f32;
		let fov = if self.controls&CONTROL_ZOOM==0 {
			self.fov
		}else{
			self.fov/5.0
		};
		let proj = perspective_rh(fov, aspect, 0.5, 1000.0);
		let proj_inv = proj.inverse();
		let view = glam::Mat4::from_translation(pos+self.offset) * glam::Mat4::from_euler(glam::EulerRot::YXZ, self.yaw, self.pitch, 0f32);
		let view_inv = view.inverse();

		let mut raw = [0f32; 16 * 3 + 4];
		raw[..16].copy_from_slice(&AsRef::<[f32; 16]>::as_ref(&proj)[..]);
		raw[16..32].copy_from_slice(&AsRef::<[f32; 16]>::as_ref(&proj_inv)[..]);
		raw[32..48].copy_from_slice(&AsRef::<[f32; 16]>::as_ref(&view_inv)[..]);
		raw[48..52].copy_from_slice(AsRef::<[f32; 4]>::as_ref(&view.col(3)));
		raw
	}
}

pub struct Skybox {
	start_time: std::time::Instant,
	camera: Camera,
	physics: strafe_client::body::PhysicsState,
	sky_pipeline: wgpu::RenderPipeline,
	entity_pipeline: wgpu::RenderPipeline,
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

fn get_transform_uniform_data(transforms:&Vec<glam::Mat4>) -> Vec<f32> {
	let mut raw = Vec::with_capacity(4*4*transforms.len());
	for (i,t) in transforms.iter().enumerate(){
    	let mut v = raw.split_off(4*4*i);
    	raw.extend_from_slice(&AsRef::<[f32; 4*4]>::as_ref(t)[..]);
    	raw.append(&mut v);
	}
	raw
}

fn add_obj(device:&wgpu::Device,modeldatas:& mut Vec<ModelData>,data:obj::ObjData){
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
			transforms: vec![glam::Mat4::default()],
			vertex_buf,
			entities,
		})
	}
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
		let ground=obj::ObjData{
			position: vec![[-1.0,0.0,-1.0],[1.0,0.0,-1.0],[1.0,0.0,1.0],[-1.0,0.0,1.0]],
			texture: vec![[-10.0,-10.0],[10.0,-10.0],[10.0,10.0],[-10.0,10.0]],
			normal: vec![[0.0,1.0,0.0]],
			objects: vec![obj::Object{
				name: "Ground Object".to_owned(),
				groups: vec![obj::Group{
					name: "Ground Group".to_owned(),
					index: 0,
					material: None,
					polys: vec![obj::SimplePolygon(vec![
						obj::IndexTuple(0,Some(0),Some(0)),
						obj::IndexTuple(1,Some(1),Some(0)),
						obj::IndexTuple(2,Some(2),Some(0)),
						obj::IndexTuple(3,Some(3),Some(0)),
					])]
				}]
			}],
			material_libs: Vec::new(),
		};
		add_obj(device,& mut modeldatas,obj::ObjData::load_buf(&include_bytes!("../models/teslacyberv3.0.obj")[..]).unwrap());
		add_obj(device,& mut modeldatas,obj::ObjData::load_buf(&include_bytes!("../models/suzanne.obj")[..]).unwrap());
		add_obj(device,& mut modeldatas,obj::ObjData::load_buf(&include_bytes!("../models/teapot.obj")[..]).unwrap());
		add_obj(device,& mut modeldatas,ground);
		println!("models.len = {:?}", modeldatas.len());
		modeldatas[0].transforms[0]=glam::Mat4::from_translation(glam::vec3(10.,0.,-10.));
		modeldatas[1].transforms[0]=glam::Mat4::from_translation(glam::vec3(10.,5.,10.));
		modeldatas[1].transforms.push(glam::Mat4::from_translation(glam::vec3(20.,5.,10.)));
		modeldatas[1].transforms.push(glam::Mat4::from_translation(glam::vec3(10.,5.,20.)));
		modeldatas[1].transforms.push(glam::Mat4::from_translation(glam::vec3(20.,5.,20.)));
		modeldatas[2].transforms[0]=glam::Mat4::from_translation(glam::vec3(-10.,5.,10.));
		modeldatas[3].transforms[0]=glam::Mat4::from_translation(glam::vec3(0.,0.,0.))*glam::Mat4::from_scale(glam::vec3(160.0, 1.0, 160.0));

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
			label: Some("Model"),
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

		let camera = Camera {
			screen_size: (config.width, config.height),
			offset: glam::Vec3::new(0.0,4.5-2.5,0.0),
			fov: 1.0, //fov_slope = tan(fov_y/2)
			pitch: 0.0,
			yaw: 0.0,
			controls:0,
		};
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
			temp_control_dir: glam::Vec3::ZERO,
			walkspeed: 18.0,
			contacts: std::collections::HashSet::new(),
    		models_cringe_clone: modeldatas.iter().map(|m|m.transforms.iter().map(|t|strafe_client::body::Model::new(*t))).flatten().collect(),
    		walk: strafe_client::body::WalkState::new(),
    		hitbox_halfsize: glam::vec3(1.0,2.5,1.0),
		};

		let camera_uniforms = camera.to_uniform_data(physics.body.extrapolated_position(0));
		let camera_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("Camera"),
			contents: bytemuck::cast_slice(&camera_uniforms),
			usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
		});

		//drain the modeldata vec so entities can be /moved/ to models.entities
		let mut models = Vec::<ModelGraphics>::with_capacity(modeldatas.len());
		for (i,modeldata) in modeldatas.drain(..).enumerate() {
			let model_uniforms = get_transform_uniform_data(&modeldata.transforms);
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
				transforms: modeldata.transforms,
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
			camera,
			physics,
			sky_pipeline,
			entity_pipeline,
			main_bind_group,
			camera_buf,
			models,
			depth_view,
			staging_belt: wgpu::util::StagingBelt::new(0x100),
		}
	}

	#[allow(clippy::single_match)]
	fn update(&mut self, event: winit::event::WindowEvent) {
		match event {
			winit::event::WindowEvent::KeyboardInput {
				input:
					winit::event::KeyboardInput {
						state,
						virtual_keycode: Some(keycode),
						..
					},
				..
			} => {
				match (state,keycode) {
					(k,winit::event::VirtualKeyCode::W) => match k {
						winit::event::ElementState::Pressed => self.camera.controls|=CONTROL_MOVEFORWARD,
						winit::event::ElementState::Released => self.camera.controls&=!CONTROL_MOVEFORWARD,
					}
					(k,winit::event::VirtualKeyCode::A) => match k {
						winit::event::ElementState::Pressed => self.camera.controls|=CONTROL_MOVELEFT,
						winit::event::ElementState::Released => self.camera.controls&=!CONTROL_MOVELEFT,
					}
					(k,winit::event::VirtualKeyCode::S) => match k {
						winit::event::ElementState::Pressed => self.camera.controls|=CONTROL_MOVEBACK,
						winit::event::ElementState::Released => self.camera.controls&=!CONTROL_MOVEBACK,
					}
					(k,winit::event::VirtualKeyCode::D) => match k {
						winit::event::ElementState::Pressed => self.camera.controls|=CONTROL_MOVERIGHT,
						winit::event::ElementState::Released => self.camera.controls&=!CONTROL_MOVERIGHT,
					}
					(k,winit::event::VirtualKeyCode::E) => match k {
						winit::event::ElementState::Pressed => self.camera.controls|=CONTROL_MOVEUP,
						winit::event::ElementState::Released => self.camera.controls&=!CONTROL_MOVEUP,
					}
					(k,winit::event::VirtualKeyCode::Q) => match k {
						winit::event::ElementState::Pressed => self.camera.controls|=CONTROL_MOVEDOWN,
						winit::event::ElementState::Released => self.camera.controls&=!CONTROL_MOVEDOWN,
					}
					(k,winit::event::VirtualKeyCode::Space) => match k {
						winit::event::ElementState::Pressed => self.camera.controls|=CONTROL_JUMP,
						winit::event::ElementState::Released => self.camera.controls&=!CONTROL_JUMP,
					}
					(k,winit::event::VirtualKeyCode::Z) => match k {
						winit::event::ElementState::Pressed => self.camera.controls|=CONTROL_ZOOM,
						winit::event::ElementState::Released => self.camera.controls&=!CONTROL_ZOOM,
					}
					_ => (),
				}
			}
			_ => {}
		}
	}

	fn move_mouse(&mut self, delta: (f64,f64)) {
		self.camera.pitch=(self.camera.pitch as f64+delta.1/-2048.) as f32;
		self.camera.yaw=(self.camera.yaw as f64+delta.0/-2048.) as f32;
	}

	fn resize(
		&mut self,
		config: &wgpu::SurfaceConfiguration,
		device: &wgpu::Device,
		_queue: &wgpu::Queue,
	) {
		self.depth_view = Self::create_depth_texture(config, device);
		self.camera.screen_size = (config.width, config.height);
	}

	fn render(
		&mut self,
		view: &wgpu::TextureView,
		device: &wgpu::Device,
		queue: &wgpu::Queue,
		_spawner: &strafe_client::framework::Spawner,
	) {
		let camera_mat=glam::Mat3::from_rotation_y(self.camera.yaw);
		let control_dir=camera_mat*get_control_dir(self.camera.controls&(CONTROL_MOVELEFT|CONTROL_MOVERIGHT|CONTROL_MOVEFORWARD|CONTROL_MOVEBACK)).normalize_or_zero();

		let time=self.start_time.elapsed().as_nanos() as i64;

		self.physics.run(time);

		//ALL OF THIS IS TOTALLY WRONG!!!
		let walk_target_velocity=self.physics.walkspeed*control_dir;
		//autohop (already pressing spacebar; the signal to begin trying to jump is different)
		if self.physics.grounded&&walk_target_velocity!=self.physics.walk.target_velocity {
			strafe_client::instruction::InstructionConsumer::process_instruction(&mut self.physics, strafe_client::instruction::TimedInstruction{
				time,
				instruction:strafe_client::body::PhysicsInstruction::SetWalkTargetVelocity(walk_target_velocity)
			});
		}

		if control_dir!=self.physics.temp_control_dir {
			strafe_client::instruction::InstructionConsumer::process_instruction(&mut self.physics, strafe_client::instruction::TimedInstruction{
				time,
				instruction:strafe_client::body::PhysicsInstruction::SetControlDir(control_dir)
			});
		}

		self.physics.jump_trying=self.camera.controls&CONTROL_JUMP!=0;
		//autohop (already pressing spacebar; the signal to begin trying to jump is different)
		if self.physics.grounded&&self.physics.jump_trying {
			//scroll will be implemented with InputInstruction::Jump(true) but it blocks setting self.jump_trying=true
			strafe_client::instruction::InstructionConsumer::process_instruction(&mut self.physics, strafe_client::instruction::TimedInstruction{
				time,
				instruction:strafe_client::body::PhysicsInstruction::Jump
			});
		}

		let mut encoder =
			device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });

		// update rotation
		let camera_uniforms = self.camera.to_uniform_data(self.physics.body.extrapolated_position(time));
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
			let model_uniforms = get_transform_uniform_data(&model.transforms);
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
					rpass.draw_indexed(0..entity.index_count, 0, 0..model.transforms.len() as u32);
				}
			}

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

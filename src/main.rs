use std::{borrow::Cow, time::Instant};
use wgpu::{util::DeviceExt, AstcBlock, AstcChannel};
use model::{Vertex,ModelInstance,ModelGraphicsInstance};
use physics::{InputInstruction, PhysicsInstruction};
use instruction::{TimedInstruction, InstructionConsumer};

mod bvh;
mod aabb;
mod model;
mod zeroes;
mod worker;
mod physics;
mod settings;
mod framework;
mod primitives;
mod instruction;
mod load_roblox;

struct Entity {
	index_count: u32,
	index_buf: wgpu::Buffer,
}

struct ModelGraphics {
	instances: Vec<ModelGraphicsInstance>,
	vertex_buf: wgpu::Buffer,
	entities: Vec<Entity>,
	bind_group: wgpu::BindGroup,
	model_buf: wgpu::Buffer,
}

pub struct GraphicsSamplers{
	repeat: wgpu::Sampler,
}

pub struct GraphicsBindGroupLayouts{
	model: wgpu::BindGroupLayout,
}

pub struct GraphicsBindGroups {
	camera: wgpu::BindGroup,
	skybox_texture: wgpu::BindGroup,
}

pub struct GraphicsPipelines{
	skybox: wgpu::RenderPipeline,
	model: wgpu::RenderPipeline,
}

pub struct GraphicsCamera{
	screen_size: glam::UVec2,
	fov: glam::Vec2,//slope
	//camera angles and such are extrapolated and passed in every time
}

#[inline]
fn perspective_rh(fov_x_slope: f32, fov_y_slope: f32, z_near: f32, z_far: f32) -> glam::Mat4 {
	//glam_assert!(z_near > 0.0 && z_far > 0.0);
	let r = z_far / (z_near - z_far);
	glam::Mat4::from_cols(
		glam::Vec4::new(1.0/fov_x_slope, 0.0, 0.0, 0.0),
		glam::Vec4::new(0.0, 1.0/fov_y_slope, 0.0, 0.0),
		glam::Vec4::new(0.0, 0.0, r, -1.0),
		glam::Vec4::new(0.0, 0.0, r * z_near, 0.0),
	)
}
impl GraphicsCamera{
	pub fn new(screen_size:glam::UVec2,fov:glam::Vec2)->Self{
		Self{
			screen_size,
			fov,
		}
	}
	pub fn proj(&self)->glam::Mat4{
		perspective_rh(self.fov.x, self.fov.y, 0.5, 2000.0)
	}
	pub fn world(&self,pos:glam::Vec3,angles:glam::Vec2)->glam::Mat4{
		//f32 good enough for view matrix
		glam::Mat4::from_translation(pos) * glam::Mat4::from_euler(glam::EulerRot::YXZ, angles.x, angles.y, 0f32)
	}

	pub fn to_uniform_data(&self,(pos,angles): (glam::Vec3,glam::Vec2)) -> [f32; 16 * 4] {
		let proj=self.proj();
		let proj_inv = proj.inverse();
		let view_inv=self.world(pos,angles);
		let view=view_inv.inverse();

		let mut raw = [0f32; 16 * 4];
		raw[..16].copy_from_slice(&AsRef::<[f32; 16]>::as_ref(&proj)[..]);
		raw[16..32].copy_from_slice(&AsRef::<[f32; 16]>::as_ref(&proj_inv)[..]);
		raw[32..48].copy_from_slice(&AsRef::<[f32; 16]>::as_ref(&view)[..]);
		raw[48..64].copy_from_slice(&AsRef::<[f32; 16]>::as_ref(&view_inv)[..]);
		raw
	}
}

pub struct GraphicsState{
	pipelines: GraphicsPipelines,
	bind_groups: GraphicsBindGroups,
	bind_group_layouts: GraphicsBindGroupLayouts,
	samplers: GraphicsSamplers,
	camera:GraphicsCamera,
	camera_buf: wgpu::Buffer,
	temp_squid_texture_view: wgpu::TextureView,
	models: Vec<ModelGraphics>,
	depth_view: wgpu::TextureView,
	staging_belt: wgpu::util::StagingBelt,
}

impl GraphicsState{
	pub fn clear(&mut self){
		self.models.clear();
	}
	pub fn load_user_settings(&mut self,user_settings:&settings::UserSettings){
		self.camera.fov=user_settings.calculate_fov(1.0,&self.camera.screen_size).as_vec2();
	}
}

pub struct GlobalState{
	start_time: std::time::Instant,
	manual_mouse_lock:bool,
	mouse:physics::MouseState,
	user_settings:settings::UserSettings,
	graphics:GraphicsState,
	physics_thread:worker::CompatWorker<TimedInstruction<InputInstruction>,physics::PhysicsOutputState,Box<dyn FnMut(TimedInstruction<InputInstruction>)->physics::PhysicsOutputState>>,
}

impl GlobalState{
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

	fn generate_model_graphics(&mut self,device:&wgpu::Device,queue:&wgpu::Queue,indexed_models:model::IndexedModelInstances){
		//generate texture view per texture

		//idk how to do this gooder lol
		let mut double_map=std::collections::HashMap::<u32,u32>::new();
		let mut texture_loading_threads=Vec::new();
		let num_textures=indexed_models.textures.len();
		for (i,texture_id) in indexed_models.textures.into_iter().enumerate(){
			if let Ok(mut file) = std::fs::File::open(std::path::Path::new(&format!("textures/{}.dds",texture_id))){
				double_map.insert(i as u32, texture_loading_threads.len() as u32);
				texture_loading_threads.push((texture_id,std::thread::spawn(move ||{
					ddsfile::Dds::read(&mut file).unwrap()
				})));
			}
		}

		let texture_views:Vec<wgpu::TextureView>=texture_loading_threads.into_iter().map(|(texture_id,thread)|{
			let image=thread.join().unwrap();

			let (mut width,mut height)=(image.get_width(),image.get_height());

			let format=match image.header10.unwrap().dxgi_format{
				ddsfile::DxgiFormat::R8G8B8A8_UNorm_sRGB => wgpu::TextureFormat::Rgba8UnormSrgb,
				ddsfile::DxgiFormat::BC7_UNorm_sRGB => {
					//floor(w,4), should be ceil(w,4)
					width=width/4*4;
					height=height/4*4;
					wgpu::TextureFormat::Bc7RgbaUnormSrgb
				},
				other=>panic!("unsupported format {:?}",other),
			};

			let size = wgpu::Extent3d {
				width,
				height,
				depth_or_array_layers: 1,
			};

			let layer_size = wgpu::Extent3d {
				depth_or_array_layers: 1,
				..size
			};
			let max_mips = layer_size.max_mips(wgpu::TextureDimension::D2);

			let texture = device.create_texture_with_data(
				queue,
				&wgpu::TextureDescriptor {
					size,
					mip_level_count: max_mips,
					sample_count: 1,
					dimension: wgpu::TextureDimension::D2,
					format,
					usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
					label: Some(format!("Texture{}",texture_id).as_str()),
					view_formats: &[],
				},
				&image.data,
			);
			texture.create_view(&wgpu::TextureViewDescriptor {
				label: Some(format!("Texture{} View",texture_id).as_str()),
				dimension: Some(wgpu::TextureViewDimension::D2),
				..wgpu::TextureViewDescriptor::default()
			})
		}).collect();

		//split groups with different textures into separate models
		//the models received here are supposed to be tightly packed, i.e. no code needs to check if two models are using the same groups.
		let indexed_models_len=indexed_models.models.len();
		let mut unique_texture_models=Vec::with_capacity(indexed_models_len);
		for model in indexed_models.models.into_iter(){
			//convert ModelInstance into ModelGraphicsInstance
			let instances:Vec<ModelGraphicsInstance>=model.instances.into_iter().filter_map(|instance|{
				if instance.color.w==0.0{
					None
				}else{
					Some(ModelGraphicsInstance{
						transform: glam::Mat4::from(instance.transform),
						normal_transform: glam::Mat3::from(instance.transform.matrix3.inverse().transpose()),
						color: instance.color,
					})
				}
			}).collect();
			//check each group, if it's using a new texture then make a new clone of the model
			let id=unique_texture_models.len();
			let mut unique_textures=Vec::new();
			for group in model.groups.into_iter(){
				//ignore zero copy optimization for now
				let texture_index=if let Some(texture_index)=unique_textures.iter().position(|&texture|texture==group.texture){
					texture_index
				}else{
					//create new texture_index
					let texture_index=unique_textures.len();
					unique_textures.push(group.texture);
					unique_texture_models.push(model::IndexedModelSingleTexture{
						unique_pos:model.unique_pos.clone(),
						unique_tex:model.unique_tex.clone(),
						unique_normal:model.unique_normal.clone(),
						unique_color:model.unique_color.clone(),
						unique_vertices:model.unique_vertices.clone(),
						texture:group.texture,
						groups:Vec::new(),
						instances:instances.clone(),
					});
					texture_index
				};
				unique_texture_models[id+texture_index].groups.push(model::IndexedGroupFixedTexture{
					polys:group.polys,
				});
			}
		}
		//de-index models
		let mut models=Vec::with_capacity(unique_texture_models.len());
		for model in unique_texture_models.into_iter(){
			let mut vertices = Vec::new();
			let mut index_from_vertex = std::collections::HashMap::new();//::<IndexedVertex,usize>
			let mut entities = Vec::new();
			//this mut be combined in a more complex way if the models use different render patterns per group
				let mut indices = Vec::new();
			for group in model.groups {
				for poly in group.polys {
					for end_index in 2..poly.vertices.len() {
						for &index in &[0, end_index - 1, end_index] {
							let vertex_index = poly.vertices[index];
							if let Some(&i)=index_from_vertex.get(&vertex_index){
								indices.push(i);
							}else{
								let i=vertices.len() as u16;
								let vertex=&model.unique_vertices[vertex_index as usize];
								vertices.push(Vertex {
									pos: model.unique_pos[vertex.pos as usize],
									tex: model.unique_tex[vertex.tex as usize],
									normal: model.unique_normal[vertex.normal as usize],
									color:model.unique_color[vertex.color as usize],
								});
								index_from_vertex.insert(vertex_index,i);
								indices.push(i);
							}
						}
					}
				}
			}
				entities.push(indices);
			models.push(model::ModelSingleTexture{
				instances:model.instances,
				vertices,
				entities,
				texture:model.texture,
			});
		}
		//.into_iter() the modeldata vec so entities can be /moved/ to models.entities
		let mut model_count=0;
		let mut instance_count=0;
		let uniform_buffer_binding_size=<GlobalState as framework::Example>::required_limits().max_uniform_buffer_binding_size as usize;
		let chunk_size=uniform_buffer_binding_size/MODEL_BUFFER_SIZE_BYTES;
		self.graphics.models.reserve(models.len());
		for model in models.into_iter() {
			instance_count+=model.instances.len();
			for instances_chunk in model.instances.rchunks(chunk_size){
				model_count+=1;
				let model_uniforms = get_instances_buffer_data(instances_chunk);
				let model_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
					label: Some(format!("Model{} Buf",model_count).as_str()),
					contents: bytemuck::cast_slice(&model_uniforms),
					usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
				});
				let texture_view=match model.texture{
					Some(texture_id)=>{
						match double_map.get(&texture_id){
							Some(&mapped_texture_id)=>&texture_views[mapped_texture_id as usize],
							None=>&self.graphics.temp_squid_texture_view,
						}
					},
					None=>&self.graphics.temp_squid_texture_view,
				};
				let model_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
					layout: &self.graphics.bind_group_layouts.model,
					entries: &[
						wgpu::BindGroupEntry {
							binding: 0,
							resource: model_buf.as_entire_binding(),
						},
						wgpu::BindGroupEntry {
							binding: 1,
							resource: wgpu::BindingResource::TextureView(texture_view),
						},
						wgpu::BindGroupEntry {
							binding: 2,
							resource: wgpu::BindingResource::Sampler(&self.graphics.samplers.repeat),
						},
					],
					label: Some(format!("Model{} Bind Group",model_count).as_str()),
				});
				let vertex_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
					label: Some("Vertex"),
					contents: bytemuck::cast_slice(&model.vertices),
					usage: wgpu::BufferUsages::VERTEX,
				});
				//all of these are being moved here
				self.graphics.models.push(ModelGraphics{
					instances:instances_chunk.to_vec(),
					vertex_buf,
					entities: model.entities.iter().map(|indices|{
						let index_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
							label: Some("Index"),
							contents: bytemuck::cast_slice(&indices),
							usage: wgpu::BufferUsages::INDEX,
						});
						Entity {
							index_buf,
							index_count: indices.len() as u32,
						}
					}).collect(),
					bind_group: model_bind_group,
					model_buf,
				});
			}
		}
		println!("Texture References={}",num_textures);
		println!("Textures Loaded={}",texture_views.len());
		println!("Indexed Models={}",indexed_models_len);
		println!("Graphics Objects: {}",self.graphics.models.len());
		println!("Graphics Instances: {}",instance_count);
	}
}

const MODEL_BUFFER_SIZE:usize=4*4 + 12 + 4;//let size=std::mem::size_of::<ModelInstance>();
const MODEL_BUFFER_SIZE_BYTES:usize=MODEL_BUFFER_SIZE*4;
fn get_instances_buffer_data(instances:&[ModelGraphicsInstance]) -> Vec<f32> {
	let mut raw = Vec::with_capacity(MODEL_BUFFER_SIZE*instances.len());
	for (i,mi) in instances.iter().enumerate(){
    	let mut v = raw.split_off(MODEL_BUFFER_SIZE*i);
    	//model transform
    	raw.extend_from_slice(&AsRef::<[f32; 4*4]>::as_ref(&mi.transform)[..]);
    	//normal transform
    	raw.extend_from_slice(AsRef::<[f32; 3]>::as_ref(&mi.normal_transform.x_axis));
    	raw.extend_from_slice(&[0.0]);
    	raw.extend_from_slice(AsRef::<[f32; 3]>::as_ref(&mi.normal_transform.y_axis));
    	raw.extend_from_slice(&[0.0]);
    	raw.extend_from_slice(AsRef::<[f32; 3]>::as_ref(&mi.normal_transform.z_axis));
    	raw.extend_from_slice(&[0.0]);
    	//color
    	raw.extend_from_slice(AsRef::<[f32; 4]>::as_ref(&mi.color));
    	raw.append(&mut v);
	}
	raw
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
		indexed_models.append(&mut model::generate_indexed_model_list_from_obj(obj::ObjData::load_buf(&include_bytes!("../models/teslacyberv3.0.obj")[..]).unwrap(),*glam::Vec4::ONE.as_ref()));
		indexed_models.push(primitives::unit_sphere());
		indexed_models.push(primitives::unit_cylinder());
		indexed_models.push(primitives::unit_cube());
		println!("models.len = {:?}", indexed_models.len());
		indexed_models[0].instances.push(ModelInstance{
			transform:glam::Affine3A::from_translation(glam::vec3(10.,0.,-10.)),
			..Default::default()
		});
		//quad monkeys
		indexed_models[1].instances.push(ModelInstance{
			transform:glam::Affine3A::from_translation(glam::vec3(10.,5.,10.)),
			..Default::default()
		});
		indexed_models[1].instances.push(ModelInstance{
			transform:glam::Affine3A::from_translation(glam::vec3(20.,5.,10.)),
			color:glam::vec4(1.0,0.0,0.0,1.0),
			..Default::default()
		});
		indexed_models[1].instances.push(ModelInstance{
			transform:glam::Affine3A::from_translation(glam::vec3(10.,5.,20.)),
			color:glam::vec4(0.0,1.0,0.0,1.0),
			..Default::default()
		});
		indexed_models[1].instances.push(ModelInstance{
			transform:glam::Affine3A::from_translation(glam::vec3(20.,5.,20.)),
			color:glam::vec4(0.0,0.0,1.0,1.0),
			..Default::default()
		});
		//decorative monkey
		indexed_models[1].instances.push(ModelInstance{
			transform:glam::Affine3A::from_translation(glam::vec3(15.,10.,15.)),
			color:glam::vec4(0.5,0.5,0.5,0.5),
			attributes:model::CollisionAttributes::Decoration,
			..Default::default()
		});
		//teapot
		indexed_models[2].instances.push(ModelInstance{
			transform:glam::Affine3A::from_scale_rotation_translation(glam::vec3(0.5, 1.0, 0.2),glam::quat(-0.22248298016985793,-0.839457167990537,-0.05603504040830783,-0.49261857546227916),glam::vec3(-10.,7.,10.)),
			..Default::default()
		});
		//ground
		indexed_models[3].instances.push(ModelInstance{
			transform:glam::Affine3A::from_translation(glam::vec3(0.,0.,0.))*glam::Affine3A::from_scale(glam::vec3(160.0, 1.0, 160.0)),
			..Default::default()
		});

		let camera_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
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
		let skybox_texture_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			label: Some("Skybox Texture Bind Group Layout"),
			entries: &[
				wgpu::BindGroupLayoutEntry {
					binding: 0,
					visibility: wgpu::ShaderStages::FRAGMENT,
					ty: wgpu::BindingType::Texture {
						sample_type: wgpu::TextureSampleType::Float { filterable: true },
						multisampled: false,
						view_dimension: wgpu::TextureViewDimension::Cube,
					},
					count: None,
				},
				wgpu::BindGroupLayoutEntry {
					binding: 1,
					visibility: wgpu::ShaderStages::FRAGMENT,
					ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
					count: None,
				},
			],
		});
		let model_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			label: Some("Model Bind Group Layout"),
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
						view_dimension: wgpu::TextureViewDimension::D2,
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

		let clamp_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
			label: Some("Clamp Sampler"),
			address_mode_u: wgpu::AddressMode::ClampToEdge,
			address_mode_v: wgpu::AddressMode::ClampToEdge,
			address_mode_w: wgpu::AddressMode::ClampToEdge,
			mag_filter: wgpu::FilterMode::Linear,
			min_filter: wgpu::FilterMode::Linear,
			mipmap_filter: wgpu::FilterMode::Linear,
			..Default::default()
		});
		let repeat_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
			label: Some("Repeat Sampler"),
			address_mode_u: wgpu::AddressMode::Repeat,
			address_mode_v: wgpu::AddressMode::Repeat,
			address_mode_w: wgpu::AddressMode::Repeat,
			mag_filter: wgpu::FilterMode::Linear,
			min_filter: wgpu::FilterMode::Linear,
			mipmap_filter: wgpu::FilterMode::Linear,
			..Default::default()
		});

		// Create the render pipeline
		let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
			label: None,
			source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(include_str!("shader.wgsl"))),
		});

		//load textures
		let device_features = device.features();

		let skybox_texture_view={
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

			let skybox_image = ddsfile::Dds::read(&mut std::io::Cursor::new(bytes)).unwrap();

			let size = wgpu::Extent3d {
				width: skybox_image.get_width(),
				height: skybox_image.get_height(),
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
				size.width,
				size.height,
				max_mips,
			);

			let skybox_texture = device.create_texture_with_data(
				queue,
				&wgpu::TextureDescriptor {
					size,
					mip_level_count: max_mips,
					sample_count: 1,
					dimension: wgpu::TextureDimension::D2,
					format: skybox_format,
					usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
					label: Some("Skybox Texture"),
					view_formats: &[],
				},
				&skybox_image.data,
			);

			skybox_texture.create_view(&wgpu::TextureViewDescriptor {
				label: Some("Skybox Texture View"),
				dimension: Some(wgpu::TextureViewDimension::Cube),
				..wgpu::TextureViewDescriptor::default()
			})
		};

		//squid
		let squid_texture_view={
			let bytes = include_bytes!("../images/squid.dds");

			let image = ddsfile::Dds::read(&mut std::io::Cursor::new(bytes)).unwrap();

			let size = wgpu::Extent3d {
				width: image.get_width(),
				height: image.get_height(),
				depth_or_array_layers: 1,
			};

			let layer_size = wgpu::Extent3d {
				depth_or_array_layers: 1,
				..size
			};
			let max_mips = layer_size.max_mips(wgpu::TextureDimension::D2);

			let texture = device.create_texture_with_data(
				queue,
				&wgpu::TextureDescriptor {
					size,
					mip_level_count: max_mips,
					sample_count: 1,
					dimension: wgpu::TextureDimension::D2,
					format: wgpu::TextureFormat::Bc7RgbaUnorm,
					usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
					label: Some("Squid Texture"),
					view_formats: &[],
				},
				&image.data,
			);

			texture.create_view(&wgpu::TextureViewDescriptor {
				label: Some("Squid Texture View"),
				dimension: Some(wgpu::TextureViewDimension::D2),
				..wgpu::TextureViewDescriptor::default()
			})
		};

		let model_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
			label: None,
			bind_group_layouts: &[
				&camera_bind_group_layout,
				&skybox_texture_bind_group_layout,
				&model_bind_group_layout,
			],
			push_constant_ranges: &[],
		});
		let sky_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
			label: None,
			bind_group_layouts: &[
				&camera_bind_group_layout,
				&skybox_texture_bind_group_layout,
			],
			push_constant_ranges: &[],
		});

		// Create the render pipelines
		let sky_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
			label: Some("Sky Pipeline"),
			layout: Some(&sky_pipeline_layout),
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
		let model_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
			label: Some("Model Pipeline"),
			layout: Some(&model_pipeline_layout),
			vertex: wgpu::VertexState {
				module: &shader,
				entry_point: "vs_entity_texture",
				buffers: &[wgpu::VertexBufferLayout {
					array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
					step_mode: wgpu::VertexStepMode::Vertex,
					attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x2, 2 => Float32x3, 3 => Float32x4],
				}],
			},
			fragment: Some(wgpu::FragmentState {
				module: &shader,
				entry_point: "fs_entity_texture",
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

		let mut physics = physics::PhysicsState::default();

		physics.load_user_settings(&user_settings);

		let screen_size=glam::uvec2(config.width,config.height);

		let camera=GraphicsCamera::new(screen_size,user_settings.calculate_fov(1.0,&screen_size).as_vec2());
		let camera_uniforms = camera.to_uniform_data(physics.output().adjust_mouse(&physics.next_mouse));
		let camera_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("Camera"),
			contents: bytemuck::cast_slice(&camera_uniforms),
			usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
		});
		let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &camera_bind_group_layout,
			entries: &[
				wgpu::BindGroupEntry {
					binding: 0,
					resource: camera_buf.as_entire_binding(),
				},
			],
			label: Some("Camera"),
		});

		let skybox_texture_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &skybox_texture_bind_group_layout,
			entries: &[
				wgpu::BindGroupEntry {
					binding: 0,
					resource: wgpu::BindingResource::TextureView(&skybox_texture_view),
				},
				wgpu::BindGroupEntry {
					binding: 1,
					resource: wgpu::BindingResource::Sampler(&clamp_sampler),
				},
			],
			label: Some("Sky Texture"),
		});

		let depth_view = Self::create_depth_texture(config, device);

		let mut graphics=GraphicsState {
			pipelines:GraphicsPipelines{
				skybox:sky_pipeline,
				model:model_pipeline
			},
			bind_groups:GraphicsBindGroups{
				camera:camera_bind_group,
				skybox_texture:skybox_texture_bind_group,
			},
			camera,
			camera_buf,
			models: Vec::new(),
			depth_view,
			staging_belt: wgpu::util::StagingBelt::new(0x100),
			bind_group_layouts: GraphicsBindGroupLayouts { model: model_bind_group_layout },
			samplers: GraphicsSamplers { repeat: repeat_sampler },
			temp_squid_texture_view: squid_texture_view,
		};

		graphics.load_user_settings(&user_settings);

		let indexed_model_instances=model::IndexedModelInstances{
			textures:Vec::new(),
			models:indexed_models,
			spawn_point:glam::Vec3::Y*50.0,
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
		match event {
			winit::event::WindowEvent::DroppedFile(path) => self.load_file(path,device,queue),
			winit::event::WindowEvent::Focused(state)=>{
				//pause unpause
				//recalculate pressed keys on focus
			}
			_=>(),
		}
	}

	fn device_event(&mut self, window: &winit::window::Window, event: winit::event::DeviceEvent) {
		//there's no way this is the best way get a timestamp.
		let time=self.start_time.elapsed().as_nanos() as i64;
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
					17=>Some(InputInstruction::MoveForward(s)),//W
					30=>Some(InputInstruction::MoveLeft(s)),//A
					31=>Some(InputInstruction::MoveBack(s)),//S
					32=>Some(InputInstruction::MoveRight(s)),//D
					18=>Some(InputInstruction::MoveUp(s)),//E
					16=>Some(InputInstruction::MoveDown(s)),//Q
					57=>Some(InputInstruction::Jump(s)),//Space
					44=>Some(InputInstruction::Zoom(s)),//Z
					19=>if s{Some(InputInstruction::Reset)}else{None},//R
					87=>{//F11
						if s{
							if window.fullscreen().is_some(){
								window.set_fullscreen(None);
							}else{
								window.set_fullscreen(Some(winit::window::Fullscreen::Borderless(None)));
							}
						}
						None
					},
					01=>{//Esc
						if s{
							self.manual_mouse_lock=false;
							match window.set_cursor_grab(winit::window::CursorGrabMode::None){
								Ok(())=>(),
								Err(e)=>println!("Could not release cursor: {:?}",e),
							}
							window.set_cursor_visible(true);
						}
						None
					},
					15=>{//Tab
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
						None
					},
					_ => {println!("scancode {}",keycode);None},
				}{
					self.physics_thread.send(TimedInstruction{
						time,
						instruction:input_instruction,
					}).unwrap();
				}
			},
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

	fn resize(
		&mut self,
		config: &wgpu::SurfaceConfiguration,
		device: &wgpu::Device,
		_queue: &wgpu::Queue,
	) {
		self.graphics.depth_view = Self::create_depth_texture(config, device);
		self.graphics.camera.screen_size=glam::uvec2(config.width, config.height);
		self.graphics.load_user_settings(&self.user_settings);
	}

	fn render(
		&mut self,
		view: &wgpu::TextureView,
		device: &wgpu::Device,
		queue: &wgpu::Queue,
		_spawner: &framework::Spawner,
	) {
		//ideally this would be scheduled to execute and finish right before the render.
		let time=self.start_time.elapsed().as_nanos() as i64;
		self.physics_thread.send(TimedInstruction{
			time,
			instruction:InputInstruction::Idle,
		}).unwrap();
		//update time lol
		self.mouse.time=time;

		let mut encoder =
			device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });

		// update rotation
		let camera_uniforms = self.graphics.camera.to_uniform_data(self.physics_thread.grab_clone().adjust_mouse(&self.mouse));
		self.graphics.staging_belt
			.write_buffer(
				&mut encoder,
				&self.graphics.camera_buf,
				0,
				wgpu::BufferSize::new((camera_uniforms.len() * 4) as wgpu::BufferAddress).unwrap(),
				device,
			)
			.copy_from_slice(bytemuck::cast_slice(&camera_uniforms));
		//This code only needs to run when the uniforms change
		/*
		for model in self.graphics.models.iter() {
			let model_uniforms = get_instances_buffer_data(&model.instances);
			self.graphics.staging_belt
				.write_buffer(
					&mut encoder,
					&model.model_buf,//description of where data will be written when command is executed
					0,//offset in staging belt?
					wgpu::BufferSize::new((model_uniforms.len() * 4) as wgpu::BufferAddress).unwrap(),
					device,
				)
				.copy_from_slice(bytemuck::cast_slice(&model_uniforms));
		}
		*/
		self.graphics.staging_belt.finish();

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
					view: &self.graphics.depth_view,
					depth_ops: Some(wgpu::Operations {
						load: wgpu::LoadOp::Clear(1.0),
						store: false,
					}),
					stencil_ops: None,
				}),
			});

			rpass.set_bind_group(0, &self.graphics.bind_groups.camera, &[]);
			rpass.set_bind_group(1, &self.graphics.bind_groups.skybox_texture, &[]);

			rpass.set_pipeline(&self.graphics.pipelines.model);
			for model in self.graphics.models.iter() {
				rpass.set_bind_group(2, &model.bind_group, &[]);
				rpass.set_vertex_buffer(0, model.vertex_buf.slice(..));

				for entity in model.entities.iter() {
					rpass.set_index_buffer(entity.index_buf.slice(..), wgpu::IndexFormat::Uint16);
					rpass.draw_indexed(0..entity.index_count, 0, 0..model.instances.len() as u32);
				}
			}

			rpass.set_pipeline(&self.graphics.pipelines.skybox);
			rpass.draw(0..3, 0..1);
		}

		queue.submit(std::iter::once(encoder.finish()));

		self.graphics.staging_belt.recall();
	}
}

fn main() {
	framework::run::<GlobalState>(
		format!("Strafe Client v{}",
			env!("CARGO_PKG_VERSION")
		).as_str()
	);
}

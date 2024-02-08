use std::borrow::Cow;
use strafesnet_common::integer;
use wgpu::{util::DeviceExt,AstcBlock,AstcChannel};
use crate::model_graphics::{GraphicsVertex,GraphicsModelColor4,GraphicsModelInstance,GraphicsModelSingleTexture,IndexedGraphicsModelSingleTexture,IndexedGroupFixedTexture};

#[derive(Clone)]
pub struct GraphicsModelUpdate{
	transform:Option<glam::Mat4>,
	color:Option<glam::Vec4>,
}

struct Entity{
	index_count:u32,
	index_buf:wgpu::Buffer,
}
fn create_entities<T:bytemuck::Pod>(device:&wgpu::Device,entities:&Vec<Vec<T>>)->Vec<Entity>{
	entities.iter().map(|indices|{
		let index_buf=device.create_buffer_init(&wgpu::util::BufferInitDescriptor{
			label:Some("Index"),
			contents:bytemuck::cast_slice(indices),
			usage:wgpu::BufferUsages::INDEX,
		});
		Entity{
			index_buf,
			index_count:indices.len() as u32,
		}
	}).collect()
}

struct GraphicsModel{
	entities:Vec<Entity>,
	model_buf:wgpu::Buffer,
	vertex_buf:wgpu::Buffer,
	bind_group:wgpu::BindGroup,
	index_format:wgpu::IndexFormat,
	instances:Vec<GraphicsModelInstance>,
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
impl std::default::Default for GraphicsCamera{
	fn default()->Self{
		Self{
			screen_size:glam::UVec2::ONE,
			fov:glam::Vec2::ONE,
		}
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
	models: Vec<GraphicsModel>,
	depth_view: wgpu::TextureView,
	staging_belt: wgpu::util::StagingBelt,
}

impl GraphicsState{
	const DEPTH_FORMAT: wgpu::TextureFormat=wgpu::TextureFormat::Depth24Plus;
	fn create_depth_texture(
		config: &wgpu::SurfaceConfiguration,
		device: &wgpu::Device,
	) -> wgpu::TextureView {
		let depth_texture=device.create_texture(&wgpu::TextureDescriptor {
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
	pub fn clear(&mut self){
		self.models.clear();
	}
	pub fn load_user_settings(&mut self,user_settings:&crate::settings::UserSettings){
		self.camera.fov=user_settings.calculate_fov(1.0,&self.camera.screen_size).as_vec2();
	}
	pub fn generate_models(&mut self,device:&wgpu::Device,queue:&wgpu::Queue,indexed_models:crate::model::IndexedModelInstances){
		//generate texture view per texture

		//idk how to do this gooder lol
		let mut double_map=std::collections::HashMap::<u32,u32>::new();
		let mut texture_loading_threads=Vec::new();
		let num_textures=indexed_models.textures.len();
		for (i,texture_id) in indexed_models.textures.into_iter().enumerate(){
			let path=std::path::PathBuf::from(format!("textures/{}.dds",texture_id));
			if let Ok(mut file) = std::fs::File::open(path.clone()){
				double_map.insert(i as u32, texture_loading_threads.len() as u32);
				texture_loading_threads.push((texture_id,std::thread::spawn(move ||{
					ddsfile::Dds::read(&mut file).unwrap()
				})));
			}else{
				//println!("missing texture path={:?}",path);
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
				wgpu::util::TextureDataOrder::LayerMajor,
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
			//convert ModelInstance into GraphicsModelInstance
			let instances:Vec<GraphicsModelInstance>=model.instances.into_iter().filter_map(|instance|{
				if instance.color.w==0.0&&!model.groups.iter().any(|g|g.texture.is_some()){
					None
				}else{
					Some(GraphicsModelInstance{
						transform: instance.transform.into(),
						normal_transform: Into::<glam::Mat3>::into(instance.transform.matrix3).inverse().transpose(),
						color:GraphicsModelColor4::from(instance.color),
					})
				}
			}).collect();
			//skip pushing a model if all instances are invisible
			if instances.len()==0{
				continue;
			}
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
					unique_texture_models.push(IndexedGraphicsModelSingleTexture{
						unique_pos:model.unique_pos.iter().map(|&v|*Into::<glam::Vec3>::into(v).as_ref()).collect(),
						unique_tex:model.unique_tex.iter().map(|v|*v.as_ref()).collect(),
						unique_normal:model.unique_normal.iter().map(|&v|*Into::<glam::Vec3>::into(v).as_ref()).collect(),
						unique_color:model.unique_color.iter().map(|v|*v.as_ref()).collect(),
						unique_vertices:model.unique_vertices.clone(),
						texture:group.texture,
						groups:Vec::new(),
						instances:instances.clone(),
					});
					texture_index
				};
				unique_texture_models[id+texture_index].groups.push(IndexedGroupFixedTexture{
					polys:group.polys,
				});
			}
		}
		//check every model to see if it's using the same (texture,color) but has few instances, if it is combine it into one model
		//1. collect unique instances of texture and color, note model id
		//2. for each model id, check if removing it from the pool decreases both the model count and instance count by more than one
		//3. transpose all models that stay in the set

		//best plan: benchmark set_bind_group, set_vertex_buffer, set_index_buffer and draw_indexed
		//check if the estimated render performance is better by transposing multiple model instances into one model instance

		//for now: just deduplicate single models...
		let mut deduplicated_models=Vec::with_capacity(indexed_models_len);//use indexed_models_len because the list will likely get smaller instead of bigger
		let mut unique_texture_color=std::collections::HashMap::new();//texture->color->vec![(model_id,instance_id)]
		for (model_id,model) in unique_texture_models.iter().enumerate(){
			//for now: filter out models with more than one instance
			if 1<model.instances.len(){
				continue;
			}
			//populate hashmap
			let unique_color=if let Some(unique_color)=unique_texture_color.get_mut(&model.texture){
				unique_color
			}else{
				//make new hashmap
				let unique_color=std::collections::HashMap::new();
				unique_texture_color.insert(model.texture,unique_color);
				unique_texture_color.get_mut(&model.texture).unwrap()
			};
			//separate instances by color
			for (instance_id,instance) in model.instances.iter().enumerate(){
				let model_instance_list=if let Some(model_instance_list)=unique_color.get_mut(&instance.color){
					model_instance_list
				}else{
					//make new hashmap
					let model_instance_list=Vec::new();
					unique_color.insert(instance.color.clone(),model_instance_list);
					unique_color.get_mut(&instance.color).unwrap()
				};
				//add model instance to list
				model_instance_list.push((model_id,instance_id));
			}
		}
		//populate a hashset of models selected for transposition
		//construct transposed models
		let mut selected_model_instances=std::collections::HashSet::new();
		for (texture,unique_color) in unique_texture_color.into_iter(){
			for (color,model_instance_list) in unique_color.into_iter(){
				//world transforming one model does not meet the definition of deduplicaiton
				if 1<model_instance_list.len(){
					//create model
					let mut unique_pos=Vec::new();
					let mut pos_id_from=std::collections::HashMap::new();
					let mut unique_tex=Vec::new();
					let mut tex_id_from=std::collections::HashMap::new();
					let mut unique_normal=Vec::new();
					let mut normal_id_from=std::collections::HashMap::new();
					let mut unique_color=Vec::new();
					let mut color_id_from=std::collections::HashMap::new();
					let mut unique_vertices=Vec::new();
					let mut vertex_id_from=std::collections::HashMap::new();

					let mut polys=Vec::new();
					//transform instance vertices
					for (model_id,instance_id) in model_instance_list.into_iter(){
						//populate hashset to prevent these models from being copied
						selected_model_instances.insert(model_id);
						//there is only one instance per model
						let model=&unique_texture_models[model_id];
						let instance=&model.instances[instance_id];
						//just hash word slices LOL
						let map_pos_id:Vec<u32>=model.unique_pos.iter().map(|untransformed_pos|{
							let pos=instance.transform.transform_point3(glam::Vec3::from_array(untransformed_pos.clone())).to_array();
							let h=pos.map(|v|bytemuck::cast::<f32,u32>(v));
							(if let Some(&pos_id)=pos_id_from.get(&h){
								pos_id
							}else{
								let pos_id=unique_pos.len();
								unique_pos.push(pos.clone());
								pos_id_from.insert(h,pos_id);
								pos_id
							}) as u32
						}).collect();
						let map_tex_id:Vec<u32>=model.unique_tex.iter().map(|tex|{
							let h=tex.map(|v|bytemuck::cast::<f32,u32>(v));
							(if let Some(&tex_id)=tex_id_from.get(&h){
								tex_id
							}else{
								let tex_id=unique_tex.len();
								unique_tex.push(tex.clone());
								tex_id_from.insert(h,tex_id);
								tex_id
							}) as u32
						}).collect();
						let map_normal_id:Vec<u32>=model.unique_normal.iter().map(|untransformed_normal|{
							let normal=(instance.normal_transform*glam::Vec3::from_array(untransformed_normal.clone())).to_array();
							let h=normal.map(|v|bytemuck::cast::<f32,u32>(v));
							(if let Some(&normal_id)=normal_id_from.get(&h){
								normal_id
							}else{
								let normal_id=unique_normal.len();
								unique_normal.push(normal.clone());
								normal_id_from.insert(h,normal_id);
								normal_id
							}) as u32
						}).collect();
						let map_color_id:Vec<u32>=model.unique_color.iter().map(|color|{
							let h=color.map(|v|bytemuck::cast::<f32,u32>(v));
							(if let Some(&color_id)=color_id_from.get(&h){
								color_id
							}else{
								let color_id=unique_color.len();
								unique_color.push(color.clone());
								color_id_from.insert(h,color_id);
								color_id
							}) as u32
						}).collect();
						//map the indexed vertices onto new indices
						//creating the vertex map is slightly different because the vertices are directly hashable
						let map_vertex_id:Vec<u32>=model.unique_vertices.iter().map(|unmapped_vertex|{
							let vertex=crate::model::IndexedVertex{
								pos:map_pos_id[unmapped_vertex.pos as usize],
								tex:map_tex_id[unmapped_vertex.tex as usize],
								normal:map_normal_id[unmapped_vertex.normal as usize],
								color:map_color_id[unmapped_vertex.color as usize],
							};
							(if let Some(&vertex_id)=vertex_id_from.get(&vertex){
								vertex_id
							}else{
								let vertex_id=unique_vertices.len();
								unique_vertices.push(vertex.clone());
								vertex_id_from.insert(vertex,vertex_id);
								vertex_id
							}) as u32
						}).collect();
						for group in &model.groups{
							for poly in &group.polys{
								polys.push(crate::model::IndexedPolygon{vertices:poly.vertices.iter().map(|&vertex_id|map_vertex_id[vertex_id as usize]).collect()});
							}
						}
					}
					//push model into dedup
					deduplicated_models.push(IndexedGraphicsModelSingleTexture{
						unique_pos,
						unique_tex,
						unique_normal,
						unique_color,
						unique_vertices,
						texture,
						groups:vec![IndexedGroupFixedTexture{
							polys
						}],
						instances:vec![GraphicsModelInstance{
							transform:glam::Mat4::IDENTITY,
							normal_transform:glam::Mat3::IDENTITY,
							color
						}],
					});
				}
			}
		}
		//fill untouched models
		for (model_id,model) in unique_texture_models.into_iter().enumerate(){
			if !selected_model_instances.contains(&model_id){
				deduplicated_models.push(model);
			}
		}

		//de-index models
		let deduplicated_models_len=deduplicated_models.len();
		let models:Vec<GraphicsModelSingleTexture>=deduplicated_models.into_iter().map(|model|{
			let mut vertices = Vec::new();
			let mut index_from_vertex = std::collections::HashMap::new();//::<IndexedVertex,usize>
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
								let i=vertices.len();
								let vertex=&model.unique_vertices[vertex_index as usize];
								vertices.push(GraphicsVertex{
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
			GraphicsModelSingleTexture{
				instances:model.instances,
				entities:if (u32::MAX as usize)<vertices.len(){
					panic!("Model has too many vertices!")
				}else if (u16::MAX as usize)<vertices.len(){
					crate::model_graphics::Entities::U32(vec![indices.into_iter().map(|vertex_id|vertex_id as u32).collect()])
				}else{
					crate::model_graphics::Entities::U16(vec![indices.into_iter().map(|vertex_id|vertex_id as u16).collect()])
				},
				vertices,
				texture:model.texture,
			}
		}).collect();
		//.into_iter() the modeldata vec so entities can be /moved/ to models.entities
		let mut model_count=0;
		let mut instance_count=0;
		let uniform_buffer_binding_size=crate::setup::required_limits().max_uniform_buffer_binding_size as usize;
		let chunk_size=uniform_buffer_binding_size/MODEL_BUFFER_SIZE_BYTES;
		self.models.reserve(models.len());
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
							None=>&self.temp_squid_texture_view,
						}
					},
					None=>&self.temp_squid_texture_view,
				};
				let model_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
					layout: &self.bind_group_layouts.model,
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
							resource: wgpu::BindingResource::Sampler(&self.samplers.repeat),
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
				self.models.push(GraphicsModel{
					instances:instances_chunk.to_vec(),
					vertex_buf,
					index_format:match &model.entities{
						crate::model_graphics::Entities::U32(_)=>wgpu::IndexFormat::Uint32,
						crate::model_graphics::Entities::U16(_)=>wgpu::IndexFormat::Uint16,
					},
					entities:match &model.entities{
						crate::model_graphics::Entities::U32(entities)=>create_entities(device,entities),
						crate::model_graphics::Entities::U16(entities)=>create_entities(device,entities),
					},
					bind_group: model_bind_group,
					model_buf,
				});
			}
		}
		println!("Texture References={}",num_textures);
		println!("Textures Loaded={}",texture_views.len());
		println!("Indexed Models={}",indexed_models_len);
		println!("Deduplicated Models={}",deduplicated_models_len);
		println!("Graphics Objects: {}",self.models.len());
		println!("Graphics Instances: {}",instance_count);
	}

	pub fn new(
		device:&wgpu::Device,
		queue:&wgpu::Queue,
		config:&wgpu::SurfaceConfiguration,
	)->Self{
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
			anisotropy_clamp:16,
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
				println!("Using ASTC");
				wgpu::TextureFormat::Astc {
					block: AstcBlock::B4x4,
					channel: AstcChannel::UnormSrgb,
				}
			} else if device_features.contains(wgpu::Features::TEXTURE_COMPRESSION_ETC2) {
				println!("Using ETC2");
				wgpu::TextureFormat::Etc2Rgb8UnormSrgb
			} else if device_features.contains(wgpu::Features::TEXTURE_COMPRESSION_BC) {
				println!("Using BC");
				wgpu::TextureFormat::Bc1RgbaUnormSrgb
			} else {
				println!("Using plain");
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
				wgpu::util::TextureDataOrder::LayerMajor,
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
				wgpu::util::TextureDataOrder::LayerMajor,
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
					array_stride: std::mem::size_of::<GraphicsVertex>() as wgpu::BufferAddress,
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
				cull_mode:Some(wgpu::Face::Front),
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

		let camera=GraphicsCamera::default();
		let camera_uniforms = camera.to_uniform_data(crate::physics::PhysicsOutputState::default().extrapolate(glam::IVec2::ZERO,integer::Time::ZERO));
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

		Self{
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
		}
	}
	pub fn resize(
		&mut self,
		device:&wgpu::Device,
		config:&wgpu::SurfaceConfiguration,
		user_settings:&crate::settings::UserSettings,
	) {
		self.depth_view=Self::create_depth_texture(config,device);
		self.camera.screen_size=glam::uvec2(config.width,config.height);
		self.load_user_settings(user_settings);
	}
	pub fn render(
		&mut self,
		view:&wgpu::TextureView,
		device:&wgpu::Device,
		queue:&wgpu::Queue,
		physics_output:crate::physics::PhysicsOutputState,
		predicted_time:integer::Time,
		mouse_pos:glam::IVec2,
	) {
		//TODO: use scheduled frame times to create beautiful smoothing simulation physics extrapolation assuming no input

		let mut encoder =
			device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });

		// update rotation
		let camera_uniforms = self.camera.to_uniform_data(physics_output.extrapolate(mouse_pos,predicted_time));
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
		/*
		for model in self.models.iter() {
			let model_uniforms = get_instances_buffer_data(&model.instances);
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
		*/
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
						store:wgpu::StoreOp::Store,
					},
				})],
				depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
					view: &self.depth_view,
					depth_ops: Some(wgpu::Operations {
						load: wgpu::LoadOp::Clear(1.0),
						store:wgpu::StoreOp::Discard,
					}),
					stencil_ops: None,
				}),
				timestamp_writes:Default::default(),
				occlusion_query_set:Default::default(),
			});

			rpass.set_bind_group(0, &self.bind_groups.camera, &[]);
			rpass.set_bind_group(1, &self.bind_groups.skybox_texture, &[]);

			rpass.set_pipeline(&self.pipelines.model);
			for model in self.models.iter() {
				rpass.set_bind_group(2, &model.bind_group, &[]);
				rpass.set_vertex_buffer(0, model.vertex_buf.slice(..));

				for entity in model.entities.iter(){
					rpass.set_index_buffer(entity.index_buf.slice(..),model.index_format);
					rpass.draw_indexed(0..entity.index_count,0,0..model.instances.len() as u32);
				}
			}

			rpass.set_pipeline(&self.pipelines.skybox);
			rpass.draw(0..3, 0..1);
		}

		queue.submit(std::iter::once(encoder.finish()));

		self.staging_belt.recall();
	}
}
const MODEL_BUFFER_SIZE:usize=4*4 + 12 + 4;//let size=std::mem::size_of::<ModelInstance>();
const MODEL_BUFFER_SIZE_BYTES:usize=MODEL_BUFFER_SIZE*4;
fn get_instances_buffer_data(instances:&[GraphicsModelInstance]) -> Vec<f32> {
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
		raw.extend_from_slice(AsRef::<[f32; 4]>::as_ref(&mi.color.get()));
		raw.append(&mut v);
	}
	raw
}

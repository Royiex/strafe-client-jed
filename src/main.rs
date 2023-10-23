use std::time::Instant;
use physics::PhysicsInstruction;
use render_thread::InputInstruction;
use instruction::{TimedInstruction, InstructionConsumer};

mod bvh;
mod aabb;
mod model;
mod window;
mod worker;
mod zeroes;
mod integer;
mod physics;
mod graphics;
mod settings;
mod primitives;
mod instruction;
mod load_roblox;
mod render_thread;
mod model_graphics;
mod graphics_context;


pub struct GlobalState{
	manual_mouse_lock:bool,
	mouse:std::sync::Arc<std::sync::Mutex<physics::MouseState>>,
	user_settings:settings::UserSettings,
	//Ideally the graphics thread worker description is:
	/*
	WorkerDescription{
		input:Immediate,
		output:Realtime(PoolOrdering::Ordered(3)),
	}
	*/
	//up to three frames in flight, dropping new frame requests when all three are busy, and dropping output frames when one renders out of order
	graphics_thread:worker::INWorker<graphics::GraphicsInstruction>,
	physics_thread:worker::QNWorker<TimedInstruction<InputInstruction>>,
}

fn load_file(path: std::path::PathBuf)->Option<model::IndexedModelInstances>{
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
		}else{
			println!("Failed to read first 8 bytes and seek back to beginning of file.");
			None
		}
	}else{
		println!("Could not open file");
		None
	}
}

fn default_models()->model::IndexedModelInstances{
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
	model::IndexedModelInstances{
		textures:Vec::new(),
		models:indexed_models,
		spawn_point:integer::Planar64Vec3::Y*50,
		modes:Vec::new(),
	}
}

impl GlobalState {
	fn init() -> Self {
		//wee
		let user_settings=settings::read_user_settings();

		let mut graphics=GraphicsState::new();

		graphics.load_user_settings(&user_settings);

		//how to multithread

		//1. build
		physics.generate_models(&indexed_model_instances);

		//2. move
		let physics_thread=physics.into_worker();

		//3. forget

		let mut state=GlobalState{
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
}

fn main(){
	let title=format!("Strafe Client v{}",env!("CARGO_PKG_VERSION")).as_str();
	let context=graphics_context::setup(title);
	let global_state=GlobalState::init();//new
	global_state.replace_models(&context,default_models());
	context.start(global_state);
}

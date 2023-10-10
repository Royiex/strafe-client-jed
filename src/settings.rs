enum DerivedFov{
	FromScreenAspect,
	FromAspect{num:u32,den:u32},
}
enum Fov{
	Exactly{x:f32,y:f32},
	DeriveX{x:DerivedFov,y:f32},
	DeriveY{x:f32,y:DerivedFov},
}

struct DerivedSensitivity{
	ratio:f32,
}
enum Sensitivity{
	Exactly{x:f32,y:f32},
	DeriveX{x:DerivedSensitivity,y:f32},
	DeriveY{x:f32,y:DerivedSensitivity},
}

pub struct UserSettings{
	fov:Fov,
	sensitivity:Sensitivity,
}

pub fn read_user_settings(){
	let mut sensitivity=1.0/1024.0;
	if let Ok(file)=std::fs::File::open("settings.conf"){
		let cfg=configparser::ini::Ini::new();
		if let Ok(Some(sens))=cfg.getfloat("user","sensitivity_x"){
			sensitivity=sens;
		}
	}
}
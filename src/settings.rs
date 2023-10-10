struct Ratio{
	ratio:f64,
}
enum DerivedFov{
	FromScreenAspect,
	FromAspect(Ratio),
}
enum Fov{
	Exactly{x:f64,y:f64},
	DeriveX{x:DerivedFov,y:f64},
	DeriveY{x:f64,y:DerivedFov},
}
impl Default for Fov{
	fn default() -> Self {
		Fov::DeriveX{x:DerivedFov::FromScreenAspect,y:1.0}
	}
}

enum Sensitivity{
	Exactly{x:f64,y:f64},
	DeriveX{x:Ratio,y:f64},
	DeriveY{x:f64,y:Ratio},
}
impl Default for Sensitivity{
	fn default() -> Self {
		Sensitivity::DeriveY{x:0.001,y:Ratio{ratio:1.0}}
	}
}


#[derive(Default)]
pub struct UserSettings{
	fov:Fov,
	sensitivity:Sensitivity,
}
impl UserSettings{
	pub fn calculate_fov(&self,zoom:f64,screen_size:&glam::UVec2)->glam::DVec2{
		zoom*match &self.fov{
			&Fov::Exactly{x,y}=>glam::dvec2(x,y),
			Fov::DeriveX{x,y}=>match x{
				DerivedFov::FromScreenAspect=>glam::dvec2(y*(screen_size.x as f64/screen_size.y as f64),*y),
				DerivedFov::FromAspect(ratio)=>glam::dvec2(y*ratio.ratio,*y),
			},
			Fov::DeriveY{x,y}=>match y{
				DerivedFov::FromScreenAspect=>glam::dvec2(*x,x*(screen_size.y as f64/screen_size.x as f64)),
				DerivedFov::FromAspect(ratio)=>glam::dvec2(*x,x*ratio.ratio),
			},
		}
	}
	pub fn calculate_sensitivity(&self)->glam::DVec2{
		match &self.sensitivity{
			&Sensitivity::Exactly{x,y}=>glam::dvec2(x,y),
			Sensitivity::DeriveX{x,y}=>glam::dvec2(y*x.ratio,*y),
			Sensitivity::DeriveY{x,y}=>glam::dvec2(*x,x*y.ratio),
		}
	}
}

/*
//sensitivity is raw input dots (i.e. dpi = dots per inch) to radians conversion factor
sensitivity_x=0.001
sensitivity_mul_x_to_y=1
Sensitivity::DeriveY{x:0.0.001,y:DerivedSensitivity{ratio:1.0}}
*/

pub fn read_user_settings()->UserSettings{
	if let Ok(file)=std::fs::File::open("settings.conf"){
		let cfg=configparser::ini::Ini::new();
		let (cfg_fov_x,cfg_fov_y)=(cfg.getfloat("camera","fov_x"),cfg.getfloat("camera","fov_x"));
		let fov=match(cfg_fov_x,cfg_fov_y){
			(Ok(Some(fov_x)),Ok(Some(fov_y)))=>Fov::Exactly {
				x:fov_x,
				y:fov_y
			},
			(Ok(Some(fov_x)),Ok(None))=>Fov::DeriveY{
				x:fov_x,
				y:if let Ok(Some(fov_mul_x_to_y))=cfg.getfloat("camera","fov_mul_x_to_y"){
					DerivedFov::FromAspect(Ratio{ratio:fov_mul_x_to_y})
				}else{
					DerivedFov::FromScreenAspect
				}
			},
			(Ok(None),Ok(Some(fov_y)))=>Fov::DeriveX{
				x:if let Ok(Some(fov_mul_y_to_x))=cfg.getfloat("camera","fov_mul_y_to_x"){
					DerivedFov::FromAspect(Ratio{ratio:fov_mul_y_to_x})
				}else{
					DerivedFov::FromScreenAspect
				},
				y:fov_y,
			},
			_=>{
				Fov::default()
			},
		};
		let (cfg_sensitivity_x,cfg_sensitivity_y)=(cfg.getfloat("camera","sensitivity_x"),cfg.getfloat("camera","sensitivity_x"));
		let sensitivity=match(cfg_sensitivity_x,cfg_sensitivity_y){
			(Ok(Some(sensitivity_x)),Ok(Some(sensitivity_y)))=>Sensitivity::Exactly {
				x:sensitivity_x,
				y:sensitivity_y
			},
			(Ok(Some(sensitivity_x)),Ok(None))=>Sensitivity::DeriveY{
				x:sensitivity_x,
				y:Ratio{
					ratio:if let Ok(Some(sensitivity_mul_x_to_y))=cfg.getfloat("camera","sensitivity_mul_x_to_y"){sensitivity_mul_x_to_y}else{1.0}
				}
			},
			(Ok(None),Ok(Some(sensitivity_y)))=>Sensitivity::DeriveX{
				x:Ratio{
					ratio:if let Ok(Some(sensitivity_mul_y_to_x))=cfg.getfloat("camera","sensitivity_mul_y_to_x"){sensitivity_mul_y_to_x}else{1.0}
				},
				y:sensitivity_y,
			},
			_=>{
				Sensitivity::default()
			},
		};
		UserSettings{
			fov,
			sensitivity,
		}
	}else{
		UserSettings::default()
	}
}
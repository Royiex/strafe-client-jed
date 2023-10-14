use crate::integer::{Ratio64,Ratio64Vec2};
struct Ratio{
	ratio:f64,
}
enum DerivedFov{
	FromScreenAspect,
	FromAspect(Ratio),
}
enum Fov{
	Exactly{x:f64,y:f64},
	SpecifyXDeriveY{x:f64,y:DerivedFov},
	SpecifyYDeriveX{x:DerivedFov,y:f64},
}
impl Default for Fov{
	fn default()->Self{
		Fov::SpecifyYDeriveX{x:DerivedFov::FromScreenAspect,y:1.0}
	}
}
enum DerivedSensitivity{
	FromRatio(Ratio64),
}
enum Sensitivity{
	Exactly{x:Ratio64,y:Ratio64},
	SpecifyXDeriveY{x:Ratio64,y:DerivedSensitivity},
	SpecifyYDeriveX{x:DerivedSensitivity,y:Ratio64},
}
impl Default for Sensitivity{
	fn default()->Self{
		Sensitivity::SpecifyXDeriveY{x:Ratio64::ONE*524288,y:DerivedSensitivity::FromRatio(Ratio64::ONE)}
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
			Fov::SpecifyXDeriveY{x,y}=>match y{
				DerivedFov::FromScreenAspect=>glam::dvec2(*x,x*(screen_size.y as f64/screen_size.x as f64)),
				DerivedFov::FromAspect(ratio)=>glam::dvec2(*x,x*ratio.ratio),
			},
			Fov::SpecifyYDeriveX{x,y}=>match x{
				DerivedFov::FromScreenAspect=>glam::dvec2(y*(screen_size.x as f64/screen_size.y as f64),*y),
				DerivedFov::FromAspect(ratio)=>glam::dvec2(y*ratio.ratio,*y),
			},
		}
	}
	pub fn calculate_sensitivity(&self)->Ratio64Vec2{
		match &self.sensitivity{
			&Sensitivity::Exactly{x,y}=>Ratio64Vec2::new(x,y),
			Sensitivity::SpecifyXDeriveY{x,y}=>match y{
				&DerivedSensitivity::FromRatio(ratio)=>Ratio64Vec2::new(*x,*x*ratio),
			}
			Sensitivity::SpecifyYDeriveX{x,y}=>match x{
				&DerivedSensitivity::FromRatio(ratio)=>Ratio64Vec2::new(*y*ratio,*y),
			}
		}
	}
}

/*
//sensitivity is raw input dots (i.e. dpi = dots per inch) to radians conversion factor
sensitivity_x=0.001
sensitivity_y_from_x_ratio=1
Sensitivity::DeriveY{x:0.0.001,y:DerivedSensitivity{ratio:1.0}}
*/

pub fn read_user_settings()->UserSettings{
	let mut cfg=configparser::ini::Ini::new();
	if let Ok(_)=cfg.load("settings.conf"){
		let (cfg_fov_x,cfg_fov_y)=(cfg.getfloat("camera","fov_x"),cfg.getfloat("camera","fov_y"));
		let fov=match(cfg_fov_x,cfg_fov_y){
			(Ok(Some(fov_x)),Ok(Some(fov_y)))=>Fov::Exactly {
				x:fov_x,
				y:fov_y
			},
			(Ok(Some(fov_x)),Ok(None))=>Fov::SpecifyXDeriveY{
				x:fov_x,
				y:if let Ok(Some(fov_y_from_x_ratio))=cfg.getfloat("camera","fov_y_from_x_ratio"){
					DerivedFov::FromAspect(Ratio{ratio:fov_y_from_x_ratio})
				}else{
					DerivedFov::FromScreenAspect
				}
			},
			(Ok(None),Ok(Some(fov_y)))=>Fov::SpecifyYDeriveX{
				x:if let Ok(Some(fov_x_from_y_ratio))=cfg.getfloat("camera","fov_x_from_y_ratio"){
					DerivedFov::FromAspect(Ratio{ratio:fov_x_from_y_ratio})
				}else{
					DerivedFov::FromScreenAspect
				},
				y:fov_y,
			},
			_=>{
				Fov::default()
			},
		};
		let (cfg_sensitivity_x,cfg_sensitivity_y)=(cfg.getfloat("camera","sensitivity_x"),cfg.getfloat("camera","sensitivity_y"));
		let sensitivity=match(cfg_sensitivity_x,cfg_sensitivity_y){
			(Ok(Some(sensitivity_x)),Ok(Some(sensitivity_y)))=>Sensitivity::Exactly {
				x:Ratio64::try_from(sensitivity_x).unwrap(),
				y:Ratio64::try_from(sensitivity_y).unwrap(),
			},
			(Ok(Some(sensitivity_x)),Ok(None))=>Sensitivity::SpecifyXDeriveY{
				x:Ratio64::try_from(sensitivity_x).unwrap(),
				y:if let Ok(Some(sensitivity_y_from_x_ratio))=cfg.getfloat("camera","sensitivity_y_from_x_ratio"){
					DerivedSensitivity::FromRatio(Ratio64::try_from(sensitivity_y_from_x_ratio).unwrap())
				}else{
					DerivedSensitivity::FromRatio(Ratio64::ONE)
				},
			},
			(Ok(None),Ok(Some(sensitivity_y)))=>Sensitivity::SpecifyYDeriveX{
				x:if let Ok(Some(sensitivity_x_from_y_ratio))=cfg.getfloat("camera","sensitivity_x_from_y_ratio"){
					DerivedSensitivity::FromRatio(Ratio64::try_from(sensitivity_x_from_y_ratio).unwrap())
				}else{
					DerivedSensitivity::FromRatio(Ratio64::ONE)
				},
				y:Ratio64::try_from(sensitivity_y).unwrap(),
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
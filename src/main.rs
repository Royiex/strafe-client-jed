mod setup;
mod window;
mod worker;
mod physics;
mod graphics;
mod settings;
mod face_crawler;
mod compat_worker;
mod model_physics;
mod model_graphics;
mod physics_worker;
mod graphics_worker;

fn main(){
	setup::setup_and_start(format!("Strafe Client v{}",env!("CARGO_PKG_VERSION")));
}

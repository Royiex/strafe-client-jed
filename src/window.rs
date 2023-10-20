pub struct WindowState{
	//ok
}
impl WindowState{
	fn resize(&mut self);
	fn update(&mut self, window: &winit::window::Window, event: WindowEvent);
	fn device_event(&mut self, window: &winit::window::Window, event: DeviceEvent);
	fn render(&self);
	pub fn create_window(title:&str,event_loop:&winit::event_loop::EventLoop<()>)->Result<winit::window::Window,winit::error::OsError>{
		let mut builder = winit::window::WindowBuilder::new();
		builder = builder.with_title(title);
		#[cfg(windows_OFF)] // TODO
		{
			use winit::platform::windows::WindowBuilderExtWindows;
			builder = builder.with_no_redirection_bitmap(true);
		}
		builder.build(event_loop)
	}
}
struct WindowState{
	//ok
}
impl WindowState{
	fn resize(&mut self);
	fn update(&mut self, window: &winit::window::Window, event: WindowEvent);
	fn device_event(&mut self, window: &winit::window::Window, event: DeviceEvent);
	fn render(&self);
}
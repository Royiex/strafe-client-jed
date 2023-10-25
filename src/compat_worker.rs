pub type QNWorker<Task>=CompatNWorker<Task>;
pub type INWorker<Task>=CompatNWorker<Task>;

pub struct CompatNWorker<Task>{
	data:std::marker::PhantomData<Task>,
	f:Box<dyn FnMut(Task)>,
}

impl<Task> CompatNWorker<Task>{
	pub fn new(f:impl FnMut(Task))->Self{
		Self{
			data:std::marker::PhantomData,
			f:Box::new(f),
		}
	}

	pub fn send(&self,task:Task)->Result<(),()>{
		(self.f)(task);
		Ok(())
	}
}

pub type QNWorker<'a,Task>=CompatNWorker<'a,Task>;
pub type INWorker<'a,Task>=CompatNWorker<'a,Task>;

pub struct CompatNWorker<'a,Task>{
	data:std::marker::PhantomData<Task>,
	f:Box<dyn FnMut(Task)+'a>,
}

impl<'a,Task> CompatNWorker<'a,Task>{
	pub fn new(f:impl FnMut(Task)+'a)->CompatNWorker<'a,Task>{
		Self{
			data:std::marker::PhantomData,
			f:Box::new(f),
		}
	}

	pub fn send(&mut self,task:Task)->Result<(),()>{
		(self.f)(task);
		Ok(())
	}
}

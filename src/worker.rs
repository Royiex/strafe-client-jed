use std::thread;
use std::sync::{mpsc,Arc};
use parking_lot::Mutex;

//The goal here is to have a worker thread that parks itself when it runs out of work.
//The worker thread publishes the result of its work back to the worker object for every item in the work queue.
//The physics (target use case) knows when it has not changed the body, so not updating the value is also an option.

pub struct Worker<Task:Send,Value:Clone> {
	sender: mpsc::Sender<Task>,
	value:Arc<Mutex<Value>>,
}

impl<Task:Send+'static,Value:Clone+Send+'static> Worker<Task,Value> {
	pub fn new<F:FnMut(Task)->Value+Send+'static>(value:Value,mut f:F) -> Self {
		let (sender, receiver) = mpsc::channel::<Task>();
		let ret=Self {
			sender,
			value:Arc::new(Mutex::new(value)),
		};
		let value=ret.value.clone();
		thread::spawn(move || {
			loop {
				match receiver.recv() {
					Ok(task) => {
						let v=f(task);//make sure function is evaluated before lock is acquired
						*value.lock()=v;
					}
					Err(_) => {
						println!("Worker stopping.",);
						break;
					}
				}
			}
		});
		ret
	}

	pub fn send(&self,task:Task)->Result<(), mpsc::SendError<Task>>{
		self.sender.send(task)
	}

	pub fn grab_clone(&self)->Value{
		self.value.lock().clone()
	}
}

pub struct CompatWorker<Task,Value:Clone,F>{
	data:std::marker::PhantomData<Task>,
	f:F,
	value:Value,
}

impl<Task,Value:Clone,F:FnMut(Task)->Value> CompatWorker<Task,Value,F> {
	pub fn new(value:Value,f:F) -> Self {
		Self {
			f,
			value,
			data:std::marker::PhantomData,
		}
	}

	pub fn send(&mut self,task:Task)->Result<(),()>{
		self.value=(self.f)(task);
		Ok(())
	}

	pub fn grab_clone(&self)->Value{
		self.value.clone()
	}
}

#[test]//How to run this test with printing: cargo test --release -- --nocapture
fn test_worker() {
	println!("hiiiii");
	// Create the worker thread
	let worker = Worker::new(crate::physics::Body::with_pva(crate::integer::Planar64Vec3::ZERO,crate::integer::Planar64Vec3::ZERO,crate::integer::Planar64Vec3::ZERO),
		|_|crate::physics::Body::with_pva(crate::integer::Planar64Vec3::ONE,crate::integer::Planar64Vec3::ONE,crate::integer::Planar64Vec3::ONE)
	);

	// Send tasks to the worker
	for _ in 0..5 {
		let task = crate::instruction::TimedInstruction{
			time:crate::integer::Time::ZERO,
			instruction:crate::physics::PhysicsInstruction::StrafeTick,
		};
		worker.send(task).unwrap();
	}

	// Optional: Signal the worker to stop (in a real-world scenario)
	// sender.send("STOP".to_string()).unwrap();

	// Sleep to allow the worker thread to finish processing
	thread::sleep(std::time::Duration::from_secs(2));

	// Send a new task
	let task = crate::instruction::TimedInstruction{
		time:crate::integer::Time::ZERO,
		instruction:crate::physics::PhysicsInstruction::StrafeTick,
	};
	worker.send(task).unwrap();

	println!("value={:?}",worker.grab_clone());

	// wait long enough to see print from final task
	thread::sleep(std::time::Duration::from_secs(1));
}

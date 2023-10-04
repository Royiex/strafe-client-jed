use std::thread;
use std::sync::{mpsc,Arc};
use parking_lot::{Mutex,Condvar};

struct Worker<Task:Send> {
    sender: mpsc::Sender<Task>,
    receiver: Arc<(Mutex<mpsc::Receiver<Task>>,Condvar)>,
}

impl<Task:Send+'static> Worker<Task> {
    fn new() -> Self {
        let (sender, receiver) = mpsc::channel::<Task>();
        Self {
            sender,
            receiver:Arc::new((Mutex::new(receiver),Condvar::new())),
        }
    }

    fn send(&self,task:Task)->Result<(), mpsc::SendError<Task>>{
        let ret=self.sender.send(task);
        self.receiver.1.notify_one();
        ret
    }

    fn start(&self) {
        let receiver=self.receiver.clone();
        thread::spawn(move || {
            loop{
                loop {
                    match receiver.0.lock().recv() {
                        Ok(_task) => {
                            println!("Worker got a task");
                            // Process the task
                        }
                        Err(_) => {
                            println!("Worker stopping.",);
                            break;
                        }
                    }
                }
                receiver.1.wait(&mut receiver.0.lock());
            }
        });
    }
}

#[test]
fn test_worker() {

    // Create the worker thread
    let worker = Worker::new();

    // Start the worker thread
    worker.start();

    // Send tasks to the worker
    for i in 0..5 {
        let task = crate::instruction::TimedInstruction{
            time:0,
            instruction:crate::body::PhysicsInstruction::StrafeTick,
        };
        worker.send(task).unwrap();
    }

    // Optional: Signal the worker to stop (in a real-world scenario)
    // sender.send("STOP".to_string()).unwrap();

    // Sleep to allow the worker thread to finish processing
    thread::sleep(std::time::Duration::from_secs(2));

    // Send a new task
    let task = crate::instruction::TimedInstruction{
        time:0,
        instruction:crate::body::PhysicsInstruction::StrafeTick,
    };
    worker.send(task).unwrap();
}

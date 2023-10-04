use std::thread;
use std::sync::{mpsc, Arc, Mutex};

struct Worker {
    id: usize,
    receiver: Arc<Mutex<mpsc::Receiver<Task>>>,
}

impl Worker {
    fn new(id: usize, receiver: Arc<Mutex<mpsc::Receiver<Task>>>) -> Worker {
        Worker { id, receiver }
    }

    fn start(self) {
        thread::spawn(move || {
            loop {
                let task = self.receiver.lock().unwrap().recv();
                match task {
                    Ok(task) => {
                        println!("Worker {} got a task: {}", self.id, task);
                        // Process the task
                    }
                    Err(_) => {
                        println!("Worker {} stopping.", self.id);
                        break;
                    }
                }
            }
        });
    }
}

type Task = String;

fn main() {
    let (sender, receiver) = mpsc::channel::<Task>();
    let receiver = Arc::new(Mutex::new(receiver));

    // Create a worker thread
    let worker = Worker::new(1, Arc::clone(&receiver));

    // Start the worker thread
    worker.start();

    // Send tasks to the worker
    for i in 0..5 {
        let task = format!("Task {}", i);
        sender.send(task).unwrap();
    }

    // Optional: Signal the worker to stop (in a real-world scenario)
    // sender.send("STOP".to_string()).unwrap();

    // Sleep to allow worker thread to finish processing
    thread::sleep(std::time::Duration::from_secs(2));

    // Send another task, which will spawn a new worker
    let new_worker = Worker::new(2, Arc::clone(&receiver));
    new_worker.start();
    sender.send("New Task".to_string()).unwrap();

    // Sleep to allow the new worker thread to process the task
    thread::sleep(std::time::Duration::from_secs(2));
}

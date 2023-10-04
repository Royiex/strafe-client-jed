use std::thread;
use std::sync::{mpsc, Arc, Mutex};

struct Worker {
    id: usize,
    receiver: Arc<Mutex<mpsc::Receiver<Task>>>,
    is_active: Arc<Mutex<bool>>,
}

impl Worker {
    fn new(id: usize, receiver: Arc<Mutex<mpsc::Receiver<Task>>>, is_active: Arc<Mutex<bool>>) -> Worker {
        Worker { id, receiver, is_active }
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

            // Set is_active to false when the worker is done
            *self.is_active.lock().unwrap() = false;
        });
    }
}

type Task = String;

fn main() {
    let (sender, receiver) = mpsc::channel::<Task>();
    let receiver = Arc::new(Mutex::new(receiver));
    let is_active = Arc::new(Mutex::new(true));

    // Create the first worker thread
    let worker = Worker::new(1, Arc::clone(&receiver), Arc::clone(&is_active));

    // Start the first worker thread
    worker.start();

    // Send tasks to the first worker
    for i in 0..5 {
        let task = format!("Task {}", i);
        sender.send(task).unwrap();
    }

    // Optional: Signal the first worker to stop (in a real-world scenario)
    // sender.send("STOP".to_string()).unwrap();

    // Sleep to allow the first worker thread to finish processing
    thread::sleep(std::time::Duration::from_secs(2));

    // Check if the first worker is still active
    let is_first_worker_active = *is_active.lock().unwrap();

    if !is_first_worker_active {
        // If the first worker is done, spawn a new worker
        let new_worker = Worker::new(2, Arc::clone(&receiver), Arc::clone(&is_active));
        new_worker.start();
        sender.send("New Task".to_string()).unwrap();

        // Sleep to allow the new worker thread to process the task
        thread::sleep(std::time::Duration::from_secs(2));
    } else {
        println!("First worker is still active. Skipping new worker.");
    }
}

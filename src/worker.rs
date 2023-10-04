use std::thread;
use std::sync::{mpsc,Arc};
use parking_lot::{Mutex,Condvar};
use std::sync::atomic::{AtomicBool, Ordering};

struct Worker {
    id: usize,
    receiver: Arc<(Mutex<mpsc::Receiver<Task>>, Condvar)>,
    is_active: Arc<AtomicBool>,
}

impl Worker {
    fn new(id: usize, receiver: Arc<(Mutex<mpsc::Receiver<Task>>, Condvar)>, is_active: Arc<AtomicBool>) -> Worker {
        Worker { id, receiver, is_active }
    }

    fn start(self) {
        thread::spawn(move || {
            loop {
                let (ref lock, ref cvar) = &*self.receiver;
                let task = lock.lock().unwrap().recv();
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
            self.is_active.store(false, Ordering::SeqCst);
            self.receiver.1.notify_one();
        });
    }
}

type Task = String;

fn main() {
    let (sender, receiver) = mpsc::channel::<Task>();
    let receiver = Arc::new((Mutex::new(receiver), Condvar::new()));
    let is_active = Arc::new(AtomicBool::new(true));

    // Create the worker thread
    let worker = Worker::new(1, Arc::clone(&receiver), Arc::clone(&is_active));

    // Start the worker thread
    worker.start();

    // Send tasks to the worker
    for i in 0..5 {
        let task = format!("Task {}", i);
        sender.send(task).unwrap();
    }

    // Optional: Signal the worker to stop (in a real-world scenario)
    // sender.send("STOP".to_string()).unwrap();

    // Sleep to allow the worker thread to finish processing
    thread::sleep(std::time::Duration::from_secs(2));

    // Check if the worker is still active
    let is_worker_active = is_active.load(Ordering::SeqCst);

    if !is_worker_active {
        // If the worker is done, signal it to process a new task
        is_active.store(true, Ordering::SeqCst);
        let (ref lock, ref cvar) = &*receiver;
        cvar.notify_one();

        // Send a new task
        sender.send("New Task".to_string()).unwrap();

        // Wait for the worker to finish processing
        cvar.wait_while(lock.lock(), |&active| active);
    } else {
        println!("Worker is still active. Skipping new task.");
    }
}

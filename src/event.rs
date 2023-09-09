pub struct EventStruct<E> {
	pub time: crate::body::TIME,
	pub event: E,
}

pub trait EventEmitter<E> {
	fn next_event(&self) -> Option<EventStruct<E>>;
}
pub trait EventConsumer<E> {
	fn process_event(&mut self, event:EventStruct<E>);
}

//PROPER PRIVATE FIELDS!!!
pub struct EventCollector<E> {
	event: Option<EventStruct<E>>,
}
impl<E> EventCollector<E> {
	pub fn new() -> Self {
		Self{event:None}
	}

	pub fn collect(&mut self,test_event:Option<EventStruct<E>>){
		match &test_event {
			Some(unwrap_test_event) => match &self.event {
				Some(unwrap_best_event) => if unwrap_test_event.time<unwrap_best_event.time {
					self.event=test_event;
				},
				None => self.event=test_event,
			},
			None => (),
		}
	}
	pub fn event(self) -> Option<EventStruct<E>> {
		//STEAL EVENT AND DESTROY EVENTCOLLECTOR
		return self.event
	}
}
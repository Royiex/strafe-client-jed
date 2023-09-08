pub struct EventStruct {
	pub time: crate::body::TIME,
	pub event: EventEnum,
}

pub enum EventEnum {
	CollisionStart,//(Collideable),//Body::CollisionStart
	CollisionEnd,//(Collideable),//Body::CollisionEnd
	StrafeTick,
	Jump,
}

pub trait EventTrait {
	fn next_event(&self) -> Option<EventStruct>;
}

//PROPER PRIVATE FIELDS!!!
pub struct EventCollector {
	event: Option<EventStruct>,
}
impl EventCollector {
	pub fn new() -> Self {
		Self{event:None}
	}

	pub fn collect(&mut self,test_event:Option<EventStruct>){
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
	pub fn event(self) -> Option<EventStruct> {
		//STEAL EVENT AND DESTROY EVENTCOLLECTOR
		return self.event
	}
}
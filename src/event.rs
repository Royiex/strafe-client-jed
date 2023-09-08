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
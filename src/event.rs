enum EventEnum {
	//Body::CollisionStart
	//Body::CollisionEnd
}

pub trait EventTrait {
	fn next_event() -> EventEnum;
}
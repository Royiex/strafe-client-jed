pub enum EventEnum {
	CollisionStart(crate::body::TIMESTAMP),//,Collideable),//Body::CollisionStart
	CollisionEnd(crate::body::TIMESTAMP),//,Collideable),//Body::CollisionEnd
	StrafeTick(crate::body::TIMESTAMP),
	Jump(crate::body::TIMESTAMP),
}

pub trait EventTrait {
	fn next_event(&self) -> Option<EventEnum>;
}
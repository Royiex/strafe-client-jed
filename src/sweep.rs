
//something that implements body + hitbox + transform can predict collision
impl crate::sweep::PredictCollision for Model {
	fn predict_collision(&self,other:&Model) -> Option<crate::event::EventStruct> {
		//math!
		None
	}
}
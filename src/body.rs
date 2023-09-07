pub struct Body {
	position: glam::I64Vec3,//2^32 = 1 u
	velocity: glam::I64Vec3,//2^32 = 1 u/s
	time: i64,//nanoseconds x xxxxD!
}
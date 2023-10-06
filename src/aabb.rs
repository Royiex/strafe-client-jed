#[derive(Debug,Clone,Copy,Hash,Eq,PartialEq)]
pub enum AabbFace{
	Right,//+X
	Top,
	Back,
	Left,
	Bottom,
	Front,
}
#[derive(Clone)]
pub struct Aabb {
	pub min: glam::Vec3,
	pub max: glam::Vec3,
}

impl Aabb {
	const VERTEX_DATA: [glam::Vec3; 8] = [
		glam::vec3(1., -1., -1.),
		glam::vec3(1., 1., -1.),
		glam::vec3(1., 1., 1.),
		glam::vec3(1., -1., 1.),
		glam::vec3(-1., -1., 1.),
		glam::vec3(-1., 1., 1.),
		glam::vec3(-1., 1., -1.),
		glam::vec3(-1., -1., -1.),
	];

	pub fn new() -> Self {
		Self {min: glam::Vec3::INFINITY,max: glam::Vec3::NEG_INFINITY}
	}

	pub fn grow(&mut self, point:glam::Vec3){
		self.min=self.min.min(point);
		self.max=self.max.max(point);
	}

	pub fn normal(face:AabbFace) -> glam::Vec3 {
		match face {
			AabbFace::Right => glam::vec3(1.,0.,0.),
			AabbFace::Top => glam::vec3(0.,1.,0.),
			AabbFace::Back => glam::vec3(0.,0.,1.),
			AabbFace::Left => glam::vec3(-1.,0.,0.),
			AabbFace::Bottom => glam::vec3(0.,-1.,0.),
			AabbFace::Front => glam::vec3(0.,0.,-1.),
		}
	}
	pub fn unit_vertices() -> [glam::Vec3;8] {
		return Self::VERTEX_DATA;
	}
	pub fn face(&self,face:AabbFace) -> Aabb {
		let mut aabb=self.clone();
		//in this implementation face = worldspace aabb face
		match face {
			AabbFace::Right => aabb.min.x=aabb.max.x,
			AabbFace::Top => aabb.min.y=aabb.max.y,
			AabbFace::Back => aabb.min.z=aabb.max.z,
			AabbFace::Left => aabb.max.x=aabb.min.x,
			AabbFace::Bottom => aabb.max.y=aabb.min.y,
			AabbFace::Front => aabb.max.z=aabb.min.z,
		}
		return aabb;
	}
}
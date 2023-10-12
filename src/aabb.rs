use crate::integer::Planar64Vec3;

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
pub struct Aabb{
	pub min:Planar64Vec3,
	pub max:Planar64Vec3,
}

impl Default for Aabb {
	fn default()->Self {
		Self{min:Planar64Vec3::MAX,max:Planar64Vec3::MIN}
	}
}

impl Aabb{
	const VERTEX_DATA:[Planar64Vec3;8]=[
		Planar64Vec3::int( 1,-1,-1),
		Planar64Vec3::int( 1, 1,-1),
		Planar64Vec3::int( 1, 1, 1),
		Planar64Vec3::int( 1,-1, 1),
		Planar64Vec3::int(-1,-1, 1),
		Planar64Vec3::int(-1, 1, 1),
		Planar64Vec3::int(-1, 1,-1),
		Planar64Vec3::int(-1,-1,-1),
	];

	pub fn grow(&mut self,point:Planar64Vec3){
		self.min=self.min.min(point);
		self.max=self.max.max(point);
	}
	pub fn join(&mut self,aabb:&Aabb){
		self.min=self.min.min(aabb.min);
		self.max=self.max.max(aabb.max);
	}
	pub fn inflate(&mut self,hs:Planar64Vec3){
		self.min-=hs;
		self.max+=hs;
	}
	pub fn intersects(&self,aabb:&Aabb)->bool{
		(self.min.cmplt(aabb.max)&aabb.min.cmplt(self.max)).all()
	}
	pub fn normal(face:AabbFace)->Planar64Vec3{
		match face {
			AabbFace::Right=>Planar64Vec3::int(1,0,0),
			AabbFace::Top=>Planar64Vec3::int(0,1,0),
			AabbFace::Back=>Planar64Vec3::int(0,0,1),
			AabbFace::Left=>Planar64Vec3::int(-1,0,0),
			AabbFace::Bottom=>Planar64Vec3::int(0,-1,0),
			AabbFace::Front=>Planar64Vec3::int(0,0,-1),
		}
	}
	pub fn unit_vertices()->[Planar64Vec3;8] {
		return Self::VERTEX_DATA;
	}
	// pub fn face(&self,face:AabbFace)->Aabb {
	// 	let mut aabb=self.clone();
	// 	//in this implementation face = worldspace aabb face
	// 	match face {
	// 		AabbFace::Right => aabb.min.x=aabb.max.x,
	// 		AabbFace::Top => aabb.min.y=aabb.max.y,
	// 		AabbFace::Back => aabb.min.z=aabb.max.z,
	// 		AabbFace::Left => aabb.max.x=aabb.min.x,
	// 		AabbFace::Bottom => aabb.max.y=aabb.min.y,
	// 		AabbFace::Front => aabb.max.z=aabb.min.z,
	// 	}
	// 	return aabb;
	// }
	pub fn center(&self)->Planar64Vec3{
		return self.min.midpoint(self.max)
	}
	//probably use floats for area & volume because we don't care about precision
	// pub fn area_weight(&self)->f32{
	// 	let d=self.max-self.min;
	// 	d.x*d.y+d.y*d.z+d.z*d.x
	// }
	// pub fn volume(&self)->f32{
	// 	let d=self.max-self.min;
	// 	d.x*d.y*d.z
	// }
}
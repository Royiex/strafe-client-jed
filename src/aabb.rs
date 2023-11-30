use crate::integer::Planar64Vec3;

#[derive(Clone)]
pub struct Aabb{
	min:Planar64Vec3,
	max:Planar64Vec3,
}

impl Default for Aabb {
	fn default()->Self {
		Self{min:Planar64Vec3::MAX,max:Planar64Vec3::MIN}
	}
}

impl Aabb{
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
	pub fn size(&self)->Planar64Vec3{
		self.max-self.min
	}
	pub fn center(&self)->Planar64Vec3{
		self.min.midpoint(self.max)
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
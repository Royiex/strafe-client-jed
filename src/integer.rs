//integer units
pub struct Time(i64);
impl Time{
	pub const ONE_SECOND:Self=Self(1_000_000_000);
	pub const ONE_MILLISECOND:Self=Self(1_000_000);
	pub const ONE_MICROSECOND:Self=Self(1_000);
	pub const ONE_NANOSECOND:Self=Self(1);
	pub fn from_secs(num:i64)->Self{
		Self(Self::ONE_SECOND.0*num)
	}
	pub fn from_millis(num:i64)->Self{
		Self(Self::ONE_MILLISECOND.0*num)
	}
	pub fn from_micros(num:i64)->Self{
		Self(Self::ONE_MICROSECOND.0*num)
	}
	pub fn from_nanos(num:i64)->Self{
		Self(Self::ONE_NANOSECOND.0*num)
	}
	//should I have checked subtraction? force all time variables to be positive?
}

///[-1.0,1.0] = [-2^30,2^30]
pub struct Unit32(i32);
impl Unit32{
	#[inline]
	pub fn as_planar64(&self) -> Planar64{
		Planar64(4*(self.0 as i64))
	}
}

///[-pi,pi) = [-2^31,2^31-1]
pub struct Angle32(i32);
impl Angle32{
	#[inline]
	pub fn cos(&self)->Unit32{
		Unit32(unsafe{((self.0 as f64*(std::f64::consts::PI/((2<<31) as f64))).cos()*((2<<30) as f64)).to_int_unchecked()})
	}
	#[inline]
	pub fn sin(&self)->Unit32{
		Unit32(unsafe{((self.0 as f64*(std::f64::consts::PI/((2<<31) as f64))).sin()*((2<<30) as f64)).to_int_unchecked()})
	}
}
///[-pi,pi) = [-2^31,2^31-1]
pub struct Angle32Vec2(glam::IVec2);
impl Angle32Vec2{
	//?
}

///[-1.0,1.0] = [-2^30,2^30]
pub struct Unit32Vec3(glam::IVec3);
///[-1.0,1.0] = [-2^30,2^30]
pub struct Unit32Mat3{
	x_axis:Unit32Vec3,
	y_axis:Unit32Vec3,
	z_axis:Unit32Vec3,
}

///[-1.0,1.0] = [-2^62,2^62]
pub struct Unit64(i64);
///[-pi,pi) = [-2^63,2^63-1]
pub struct Angle64(i64);

///[-1.0,1.0] = [-2^62,2^62]
pub struct Unit64Vec3(glam::IVec3);
///[-1.0,1.0] = [-2^62,2^62]
pub struct Unit64Mat3{
	x_axis:Unit64Vec3,
	y_axis:Unit64Vec3,
	z_axis:Unit64Vec3,
}


///[-1.0,1.0] = [-2^32,2^32]
#[derive(Clone,Copy)]
pub struct Planar64(i64);
impl Planar64{
	pub const ONE:Self=Self(2<<32);
	pub fn new(num:i32)->Self{
		Self(Self::ONE.0*num as i64)
	}
	pub fn from_ratio(num:i64,den:std::num::NonZeroU64)->Self{
		Self(Self::ONE.0*num/den.get() as i64)
	}
}
impl std::ops::Add<Planar64> for Planar64{
	type Output=Planar64;
	#[inline]
	fn add(self, rhs: Self) -> Self::Output {
		Planar64(self.0+rhs.0)
	}
}
impl std::ops::Mul<i64> for Planar64{
	type Output=Planar64;
	#[inline]
	fn mul(self, rhs: i64) -> Self::Output {
		Planar64(self.0*rhs)
	}
}
impl std::ops::Mul<Planar64> for Planar64{
	type Output=Planar64;
	#[inline]
	fn mul(self, rhs: Self) -> Self::Output {
		Planar64((((self.0 as i128)*(rhs.0 as i128))>>64) as i64)
	}
}
impl std::ops::Div<i64> for Planar64{
	type Output=Planar64;
	#[inline]
	fn div(self, rhs: i64) -> Self::Output {
		Planar64(self.0/rhs)
	}
}


///[-1.0,1.0] = [-2^32,2^32]
#[derive(Clone,Copy)]
pub struct Planar64Vec3(glam::I64Vec3);
impl Planar64Vec3{
	pub fn new(x:i32,y:i32,z:i32)->Self{
		Self(glam::i64vec3((x as i64)<<32,(y as i64)<<32,(z as i64)<<32))
	}
	#[inline]
	pub fn x(&self)->Planar64{
		Planar64(self.0.x)
	}
	#[inline]
	pub fn y(&self)->Planar64{
		Planar64(self.0.y)
	}
	#[inline]
	pub fn z(&self)->Planar64{
		Planar64(self.0.z)
	}
}
impl std::ops::Mul<Planar64> for Planar64Vec3{
	type Output=Planar64Vec3;
	#[inline]
	fn mul(self, rhs: Planar64) -> Self::Output {
		Planar64Vec3(glam::i64vec3(
			(((self.0.x as i128)*(rhs.0 as i128))>>64) as i64,
			(((self.0.y as i128)*(rhs.0 as i128))>>64) as i64,
			(((self.0.z as i128)*(rhs.0 as i128))>>64) as i64
		))
	}
}

///[-1.0,1.0] = [-2^32,2^32]
#[derive(Clone,Copy)]
pub struct Planar64Mat3{
	x_axis:Planar64Vec3,
	y_axis:Planar64Vec3,
	z_axis:Planar64Vec3,
}

///[-1.0,1.0] = [-2^32,2^32]
#[derive(Clone,Copy)]
pub struct Planar64Affine3{
	matrix3:Planar64Mat3,//includes scale above 1
	transform:Planar64Vec3,
}

impl Planar64Affine3{
	#[inline]
	pub fn transform_point3(&self,point:Planar64Vec3) -> Planar64Vec3{
		Planar64Vec3(
			self.transform.0
			+(self.matrix3.x_axis*point.x()).0
			+(self.matrix3.y_axis*point.y()).0
			+(self.matrix3.z_axis*point.z()).0
		)
	}
}
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

#[derive(Clone,Copy,Hash)]
pub struct Ratio64{
	num:i64,
	den:std::num::NonZeroU64,
}
impl Ratio64{
	pub const ONE:Self=Ratio64{num:1,den:unsafe{std::num::NonZeroU64::new_unchecked(1)}};
	pub fn mul_ratio(self,rhs:i64)->Self{
		Self{
			num:self.num*rhs,
			den:self.den
		}
	}
	pub fn mul_int(self,rhs:i64)->i64{
		rhs*self.num/self.den.get() as i64
	}
	pub fn rhs_div_int(self,rhs:i64)->i64{
		rhs*self.den.get() as i64/self.num
	}
}
#[derive(Clone,Hash)]
pub struct Ratio64Vec2{
	pub x:Ratio64,
	pub y:Ratio64,
}
impl Ratio64Vec2{
	pub const ONE:Self=Self{x:Ratio64::ONE,y:Ratio64::ONE};
	pub fn mul_ratio(self,rhs:i64)->Self{
		Self{
			x:self.x.mul_ratio(rhs),
			y:self.y.mul_ratio(rhs),
		}
	}
	pub fn mul_int(self,rhs:glam::I64Vec2)->glam::I64Vec2{
		glam::i64vec2(
			self.x.mul_int(rhs.x),
			self.y.mul_int(rhs.y),
		)
	}
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
#[derive(Clone,Copy,Hash)]
pub struct Angle32(i32);
impl Angle32{
	pub const FRAC_PI_2:Self=Self(2<<31);
	pub const PI:Self=Self(-2<<32);
	#[inline]
	pub fn wrap_from_i64(theta:i64)->Self{
		//TODO: make this good
		Self((theta.wrapping_add(2<<31).rem_euclid(2<<32)-(2<<31)) as i32)
	}
	#[inline]
	pub fn clamp_from_i64(theta:i64)->Self{
		//TODO: make this good
		Self(theta.clamp(i32::MIN as i64,i32::MAX as i64) as i32)
	}
	#[inline]
	pub fn get(&self)->i32{
		self.0
	}
	/// Note that theta_min can be larger than theta_max and it will clamp the other way
	#[inline]
	pub fn clamp(&self,theta_min:Self,theta_max:Self)->Self{
		//TODO: make this good
		theta_min+Self::wrap_from_i64((self.0.wrapping_sub(theta_min.0) as i64).rem_euclid(2<<32).clamp(0,(theta_max.0.wrapping_sub(theta_min.0) as i64).rem_euclid(2<<32)))
	}
	#[inline]
	pub fn cos(&self)->Unit32{
		//TODO: fix this rounding towards 0
		Unit32(unsafe{((self.0 as f64*(std::f64::consts::PI/((2<<31) as f64))).cos()*((2<<30) as f64)).to_int_unchecked()})
	}
	#[inline]
	pub fn sin(&self)->Unit32{
		//TODO: fix this rounding towards 0
		Unit32(unsafe{((self.0 as f64*(std::f64::consts::PI/((2<<31) as f64))).sin()*((2<<30) as f64)).to_int_unchecked()})
	}
}
impl Into<f32> for Angle32{
	fn into(self)->f32{
		//TODO: make this good
		(self.0 as f64/-(i32::MIN as f64)*std::f64::consts::PI) as f32
	}
}
impl std::ops::Neg for Angle32{
	type Output=Angle32;
	#[inline]
	fn neg(self)->Self::Output{
		Angle32(self.0.wrapping_neg())
	}
}
impl std::ops::Add<Angle32> for Angle32{
	type Output=Angle32;
	#[inline]
	fn add(self,rhs:Self)->Self::Output {
		Angle32(self.0.wrapping_add(rhs.0))
	}
}
impl std::ops::Sub<Angle32> for Angle32{
	type Output=Angle32;
	#[inline]
	fn sub(self,rhs:Self)->Self::Output {
		Angle32(self.0.wrapping_sub(rhs.0))
	}
}
impl std::ops::Mul<i32> for Angle32{
	type Output=Angle32;
	#[inline]
	fn mul(self,rhs:i32)->Self::Output {
		Angle32(self.0.wrapping_mul(rhs))
	}
}
impl std::ops::Mul<Angle32> for Angle32{
	type Output=Angle32;
	#[inline]
	fn mul(self,rhs:Self)->Self::Output {
		Angle32(self.0.wrapping_mul(rhs.0))
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
#[derive(Clone,Copy,Hash)]
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
#[derive(Clone,Copy,Hash)]
pub struct Planar64Vec3(glam::I64Vec3);
impl Planar64Vec3{
	pub const ZERO:Self=Planar64Vec3(glam::I64Vec3::ZERO);
	pub const ONE:Self=Planar64Vec3(glam::I64Vec3::ONE);
	pub const X:Self=Planar64Vec3(glam::I64Vec3::X);
	pub const Y:Self=Planar64Vec3(glam::I64Vec3::Y);
	pub const Z:Self=Planar64Vec3(glam::I64Vec3::Z);
	pub const NEG_X:Self=Planar64Vec3(glam::I64Vec3::NEG_X);
	pub const NEG_Y:Self=Planar64Vec3(glam::I64Vec3::NEG_Y);
	pub const NEG_Z:Self=Planar64Vec3(glam::I64Vec3::NEG_Z);
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
impl std::ops::Add<Planar64Vec3> for Planar64Vec3{
	type Output=Planar64Vec3;
	#[inline]
	fn add(self,rhs:Planar64Vec3) -> Self::Output {
		Planar64Vec3(self.0+rhs.0)
	}
}
impl std::ops::AddAssign<Planar64Vec3> for Planar64Vec3{
	#[inline]
	fn add_assign(&mut self,rhs:Planar64Vec3){
		*self=*self+rhs
	}
}
impl std::ops::Sub<Planar64Vec3> for Planar64Vec3{
	type Output=Planar64Vec3;
	#[inline]
	fn sub(self,rhs:Planar64Vec3) -> Self::Output {
		Planar64Vec3(self.0-rhs.0)
	}
}
impl std::ops::SubAssign<Planar64Vec3> for Planar64Vec3{
	#[inline]
	fn sub_assign(&mut self,rhs:Planar64Vec3){
		*self=*self-rhs
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
impl std::ops::Div<i64> for Planar64Vec3{
	type Output=Planar64Vec3;
	#[inline]
	fn div(self,rhs:i64)->Self::Output{
		Planar64Vec3(glam::i64vec3(
			self.0.x/rhs,
			self.0.y/rhs,
			self.0.z/rhs,
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

impl Planar64Mat3{
	#[inline]
	pub fn from_cols(x_axis:Planar64Vec3,y_axis:Planar64Vec3,z_axis:Planar64Vec3)->Self{
		Self{
			x_axis,
			y_axis,
			z_axis,
		}
	}
	#[inline]
	pub fn from_rotation_y(angle:Angle32)->Self{
		let theta=angle.0 as f64*(std::f64::consts::PI/((2<<31) as f64));
		let (s,c)=theta.sin_cos();
		let (c,s)=(c*((2<<32) as f64),s*((2<<32) as f64));
		//TODO: fix this rounding towards 0
		let (c,s):(i64,i64)=(unsafe{c.to_int_unchecked()},unsafe{s.to_int_unchecked()});
		Self::from_cols(
			Planar64Vec3(glam::i64vec3(c,0,-s)),
			Planar64Vec3::Y,
			Planar64Vec3(glam::i64vec3(s,0,c)),
		)
	}
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
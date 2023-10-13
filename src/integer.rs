//integer units
#[derive(Clone,Copy,Hash,PartialEq,PartialOrd,Debug)]
pub struct Time(i64);
impl Time{
	pub const ZERO:Self=Self(0);
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
	pub fn nanos(&self)->i64{
		self.0
	}
}
impl From<Planar64> for Time{
	fn from(value:Planar64)->Self{
		Time((((value.0 as i128)*1_000_000_000)>>32) as i64)
	}
}
impl std::fmt::Display for Time{
	fn fmt(&self,f:&mut std::fmt::Formatter<'_>)->std::fmt::Result{
		write!(f,"{}s+{}ns",self.0/Self::ONE_SECOND.0,self.0%Self::ONE_SECOND.0)
	}
}
impl std::ops::Neg for Time{
	type Output=Time;
	#[inline]
	fn neg(self)->Self::Output {
		Time(-self.0)
	}
}
impl std::ops::Add<Time> for Time{
	type Output=Time;
	#[inline]
	fn add(self,rhs:Self)->Self::Output {
		Time(self.0+rhs.0)
	}
}
impl std::ops::Sub<Time> for Time{
	type Output=Time;
	#[inline]
	fn sub(self,rhs:Self)->Self::Output {
		Time(self.0-rhs.0)
	}
}
impl std::ops::Mul<Time> for Time{
	type Output=Time;
	#[inline]
	fn mul(self,rhs:Time)->Self::Output{
		Self((((self.0 as i128)*(rhs.0 as i128))/1_000_000_000) as i64)
	}
}
impl std::ops::Div<i64> for Time{
	type Output=Time;
	#[inline]
	fn div(self,rhs:i64)->Self::Output {
		Time(self.0/rhs)
	}
}

fn gcd(mut a:u64,mut b:u64)->u64{
	while b!=0{
		(a,b)=(b,a.rem_euclid(b));
	};
	a
}
#[derive(Clone,Copy,Hash)]
pub struct Ratio64{
	num:i64,
	den:std::num::NonZeroU64,
}
impl Ratio64{
	pub const ZERO:Self=Ratio64{num:0,den:unsafe{std::num::NonZeroU64::new_unchecked(1)}};
	pub const ONE:Self=Ratio64{num:1,den:unsafe{std::num::NonZeroU64::new_unchecked(1)}};
	pub fn new(num:i64,den:u64)->Option<Ratio64>{
		match std::num::NonZeroU64::new(den){
			Some(_)=>{
				let d=gcd(num.unsigned_abs(),den);
				Some(Self{num:num/d as i64,den:unsafe{std::num::NonZeroU64::new_unchecked(den/d)}})
			},
			None=>None,
		}
	}
	pub fn mul_int(self,rhs:i64)->i64{
		rhs*self.num/self.den.get() as i64
	}
	pub fn rhs_div_int(self,rhs:i64)->i64{
		rhs*self.den.get() as i64/self.num
	}
}
//from num_traits crate
#[inline]
fn integer_decode_f32(f: f32) -> (u64, i16, i8) {
    let bits: u32 = f.to_bits();
    let sign: i8 = if bits >> 31 == 0 { 1 } else { -1 };
    let mut exponent: i16 = ((bits >> 23) & 0xff) as i16;
    let mantissa = if exponent == 0 {
        (bits & 0x7fffff) << 1
    } else {
        (bits & 0x7fffff) | 0x800000
    };
    // Exponent bias + mantissa shift
    exponent -= 127 + 23;
    (mantissa as u64, exponent, sign)
}
#[inline]
fn integer_decode_f64(f: f64) -> (u64, i16, i8) {
    let bits: u64 = f.to_bits();
    let sign: i8 = if bits >> 63 == 0 { 1 } else { -1 };
    let mut exponent: i16 = ((bits >> 52) & 0x7ff) as i16;
    let mantissa = if exponent == 0 {
        (bits & 0xfffffffffffff) << 1
    } else {
        (bits & 0xfffffffffffff) | 0x10000000000000
    };
    // Exponent bias + mantissa shift
    exponent -= 1023 + 52;
    (mantissa, exponent, sign)
}
#[derive(Debug)]
enum Ratio64TryFromFloatError{
	Nan,
	Infinite,
	Subnormal,
	HighlyNegativeExponent(i16),
	HighlyPositiveExponent(i16),
}
fn ratio64_from_mes((m,e,s):(u64,i16,i8))->Result<Ratio64,Ratio64TryFromFloatError>{
	if e< -127{
		//bye bye
		Err(Ratio64TryFromFloatError::HighlyNegativeExponent(e))
	}else if e< -63{
		//TODO
		Err(Ratio64TryFromFloatError::HighlyNegativeExponent(e))
	}else if e<0{
		Ok(Ratio64::new((m as i64)*(s as i64),1<<-e).unwrap())
	}else if e<62-52{
		Ok(Ratio64::new((m as i64)*(s as i64)*(1<<e),1).unwrap())
	}else{
		Err(Ratio64TryFromFloatError::HighlyPositiveExponent(e))
	}
}
impl TryFrom<f32> for Ratio64{
	type Error=Ratio64TryFromFloatError;
	fn try_from(value:f32)->Result<Self,Self::Error>{
		match value.classify(){
			std::num::FpCategory::Nan=>Err(Self::Error::Nan),
			std::num::FpCategory::Infinite=>Err(Self::Error::Infinite),
			std::num::FpCategory::Zero=>Ok(Self::ZERO),
			std::num::FpCategory::Subnormal=>Err(Self::Error::Subnormal),
			std::num::FpCategory::Normal=>ratio64_from_mes(integer_decode_f32(value)),
		}
	}
}
impl TryFrom<f64> for Ratio64{
	type Error=Ratio64TryFromFloatError;
	fn try_from(value:f64)->Result<Self,Self::Error>{
		match value.classify(){
			std::num::FpCategory::Nan=>Err(Self::Error::Nan),
			std::num::FpCategory::Infinite=>Err(Self::Error::Infinite),
			std::num::FpCategory::Zero=>Ok(Self::ZERO),
			std::num::FpCategory::Subnormal=>Err(Self::Error::Subnormal),
			std::num::FpCategory::Normal=>ratio64_from_mes(integer_decode_f64(value)),
		}
	}
}
impl std::ops::Mul<Ratio64> for Ratio64{
	type Output=Ratio64;
	#[inline]
	fn mul(self,rhs:Ratio64)->Self::Output{
		let (num,den)=(self.num*rhs.num,self.den.get()*rhs.den.get());
		let d=gcd(num.unsigned_abs(),den);
		Self{
			num:num/d as i64,
			den:unsafe{std::num::NonZeroU64::new_unchecked(den/d)},
		}
	}
}
impl std::ops::Mul<i64> for Ratio64{
	type Output=Ratio64;
	#[inline]
	fn mul(self,rhs:i64)->Self::Output {
		Self{
			num:self.num*rhs,
			den:self.den,
		}
	}
}
impl std::ops::Div<u64> for Ratio64{
	type Output=Ratio64;
	#[inline]
	fn div(self,rhs:u64)->Self::Output {
		Self{
			num:self.num,
			den:std::num::NonZeroU64::new(self.den.get()*rhs).unwrap(),
		}
	}
}
#[derive(Clone,Hash)]
pub struct Ratio64Vec2{
	pub x:Ratio64,
	pub y:Ratio64,
}
impl Ratio64Vec2{
	pub const ONE:Self=Self{x:Ratio64::ONE,y:Ratio64::ONE};
	pub fn new(x:Ratio64,y:Ratio64)->Self{
		Self{x,y}
	}
	pub fn mul_int(self,rhs:glam::I64Vec2)->glam::I64Vec2{
		glam::i64vec2(
			self.x.mul_int(rhs.x),
			self.y.mul_int(rhs.y),
		)
	}
}
impl std::ops::Mul<i64> for Ratio64Vec2{
	type Output=Ratio64Vec2;
	#[inline]
	fn mul(self,rhs:i64)->Self::Output {
		Self{
			x:self.x*rhs,
			y:self.y*rhs,
		}
	}
}

///[-pi,pi) = [-2^31,2^31-1]
#[derive(Clone,Copy,Hash)]
pub struct Angle32(i32);
impl Angle32{
	pub const FRAC_PI_2:Self=Self(1<<31);
	pub const PI:Self=Self(-1<<32);
	#[inline]
	pub fn wrap_from_i64(theta:i64)->Self{
		//take lower bits
		//note: this was checked on compiler explorer and compiles to 1 instruction!
		Self(i32::from_ne_bytes(((theta&((1<<32)-1)) as u32).to_ne_bytes()))
	}
	#[inline]
	pub fn clamp_from_i64(theta:i64)->Self{
		//the assembly is a bit confusing for this, I thought it was checking the same thing twice
		//but it's just checking and then overwriting the value for both upper and lower bounds.
		Self(theta.clamp(i32::MIN as i64,i32::MAX as i64) as i32)
	}
	#[inline]
	pub fn get(&self)->i32{
		self.0
	}
	/// Clamps the value towards the midpoint of the range.
	/// Note that theta_min can be larger than theta_max and it will wrap clamp the other way around
	#[inline]
	pub fn clamp(&self,theta_min:Self,theta_max:Self)->Self{
		//((max-min as u32)/2 as i32)+min
		let midpoint=((
			u32::from_ne_bytes(theta_max.0.to_ne_bytes())
			.wrapping_sub(u32::from_ne_bytes(theta_min.0.to_ne_bytes()))
			/2
		) as i32)//(u32::MAX/2) as i32 ALWAYS works
		.wrapping_add(theta_min.0);
		//(theta-mid).clamp(max-mid,min-mid)+mid
		Self(
			self.0.wrapping_sub(midpoint)
			.max(theta_min.0.wrapping_sub(midpoint))
			.min(theta_max.0.wrapping_sub(midpoint))
			.wrapping_add(midpoint)
		)
	}
	/*
	#[inline]
	pub fn cos(&self)->Unit32{
		//TODO: fix this rounding towards 0
		Unit32(unsafe{((self.0 as f64*(std::f64::consts::PI/((1<<31) as f64))).cos()*((1<<30) as f64)).to_int_unchecked()})
	}
	#[inline]
	pub fn sin(&self)->Unit32{
		//TODO: fix this rounding towards 0
		Unit32(unsafe{((self.0 as f64*(std::f64::consts::PI/((1<<31) as f64))).sin()*((1<<30) as f64)).to_int_unchecked()})
	}
	*/
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

/* Unit type unused for now, may revive it for map files
///[-1.0,1.0] = [-2^30,2^30]
pub struct Unit32(i32);
impl Unit32{
	#[inline]
	pub fn as_planar64(&self) -> Planar64{
		Planar64(4*(self.0 as i64))
	}
}

///[-1.0,1.0] = [-2^30,2^30]
pub struct Unit32Vec3(glam::IVec3);
impl TryFrom<[f32;3]> for Unit32Vec3{
	type Error=Unit32TryFromFloatError;
	fn try_from(value:[f32;3])->Result<Self,Self::Error>{
		Ok(Self(glam::ivec3(
			Unit32::try_from(Planar64::try_from(value[0])?)?.0,
			Unit32::try_from(Planar64::try_from(value[1])?)?.0,
			Unit32::try_from(Planar64::try_from(value[2])?)?.0,
		)))
	}
}
*/

///[-1.0,1.0] = [-2^32,2^32]
#[derive(Clone,Copy,Hash,Eq,Ord,PartialEq,PartialOrd)]
pub struct Planar64(i64);
impl Planar64{
	pub const ZERO:Self=Self(0);
	pub const ONE:Self=Self(1<<32);
	#[inline]
	pub fn int(num:i32)->Self{
		Self(Self::ONE.0*num as i64)
	}
	#[inline]
	pub fn raw(num:i64)->Self{
		Self(num)
	}
	#[inline]
	pub fn get(&self)->i64{
		self.0
	}
}
impl Into<f32> for Planar64{
	#[inline]
	fn into(self)->f32{
		self.0 as f32/(1<<32) as f32
	}
}
impl From<Ratio64> for Planar64{
	#[inline]
	fn from(ratio:Ratio64)->Self{
		Self(Self::ONE.0*ratio.num/ratio.den.get() as i64)
	}
}
#[derive(Debug)]
enum Planar64TryFromFloatError{
	Nan,
	Infinite,
	Subnormal,
	HighlyNegativeExponent(i16),
	HighlyPositiveExponent(i16),
}
fn planar64_from_mes((m,e,s):(u64,i16,i8))->Result<Planar64,Planar64TryFromFloatError>{
	if e< -32{
		Err(Planar64TryFromFloatError::HighlyNegativeExponent(e))
	}else if e<32-52{
		Ok(Planar64((m as i64)*(s as i64)<<e))
	}else{
		Err(Planar64TryFromFloatError::HighlyPositiveExponent(e))
	}
}
impl TryFrom<f32> for Planar64{
	type Error=Planar64TryFromFloatError;
	fn try_from(value:f32)->Result<Self,Self::Error>{
		match value.classify(){
			std::num::FpCategory::Nan=>Err(Self::Error::Nan),
			std::num::FpCategory::Infinite=>Err(Self::Error::Infinite),
			std::num::FpCategory::Zero=>Ok(Self::ZERO),
			std::num::FpCategory::Subnormal=>Err(Self::Error::Subnormal),
			std::num::FpCategory::Normal=>planar64_from_mes(integer_decode_f32(value)),
		}
	}
}
impl TryFrom<f64> for Planar64{
	type Error=Planar64TryFromFloatError;
	fn try_from(value:f64)->Result<Self,Self::Error>{
		match value.classify(){
			std::num::FpCategory::Nan=>Err(Self::Error::Nan),
			std::num::FpCategory::Infinite=>Err(Self::Error::Infinite),
			std::num::FpCategory::Zero=>Ok(Self::ZERO),
			std::num::FpCategory::Subnormal=>Err(Self::Error::Subnormal),
			std::num::FpCategory::Normal=>planar64_from_mes(integer_decode_f64(value)),
		}
	}
}
impl std::ops::Neg for Planar64{
	type Output=Planar64;
	#[inline]
	fn neg(self)->Self::Output{
		Planar64(-self.0)
	}
}
impl std::ops::Add<Planar64> for Planar64{
	type Output=Planar64;
	#[inline]
	fn add(self, rhs: Self) -> Self::Output {
		Planar64(self.0+rhs.0)
	}
}
impl std::ops::Sub<Planar64> for Planar64{
	type Output=Planar64;
	#[inline]
	fn sub(self, rhs: Self) -> Self::Output {
		Planar64(self.0-rhs.0)
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
impl std::ops::Div<Planar64> for Planar64{
	type Output=Planar64;
	#[inline]
	fn div(self, rhs: Planar64) -> Self::Output {
		Planar64((((self.0 as i128)<<64)/rhs.0 as i128) as i64)
	}
}
// impl PartialOrd<i64> for Planar64{
// 	fn partial_cmp(&self, other: &i64) -> Option<std::cmp::Ordering> {
// 		self.0.partial_cmp(other)
// 	}
// }


///[-1.0,1.0] = [-2^32,2^32]
#[derive(Clone,Copy,Default,Hash,Eq,PartialEq)]
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
	pub const MIN:Self=Planar64Vec3(glam::I64Vec3::MIN);
	pub const MAX:Self=Planar64Vec3(glam::I64Vec3::MAX);
	pub fn int(x:i32,y:i32,z:i32)->Self{
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
	#[inline]
	pub fn min(&self,rhs:Self)->Self{
		Self(glam::i64vec3(
			self.0.x.min(rhs.0.x),
			self.0.y.min(rhs.0.y),
			self.0.z.min(rhs.0.z),
		))
	}
	#[inline]
	pub fn max(&self,rhs:Self)->Self{
		Self(glam::i64vec3(
			self.0.x.max(rhs.0.x),
			self.0.y.max(rhs.0.y),
			self.0.z.max(rhs.0.z),
		))
	}
	#[inline]
	pub fn midpoint(&self,rhs:Self)->Self{
		Self((self.0+rhs.0)/2)
	}
	#[inline]
	pub fn cmplt(&self,rhs:Self)->glam::BVec3{
		self.0.cmplt(rhs.0)
	}
	#[inline]
	pub fn dot(&self,rhs:Self)->Planar64{
		Planar64(((
			(self.0.x as i128)*(rhs.0.x as i128)+
			(self.0.y as i128)*(rhs.0.y as i128)+
			(self.0.z as i128)*(rhs.0.z as i128)
		)>>64) as i64)
	}
	#[inline]
	pub fn length(&self)->Planar64{
		let radicand=(self.0.x as i128)*(self.0.x as i128)+(self.0.y as i128)*(self.0.y as i128)+(self.0.z as i128)*(self.0.z as i128);
		Planar64(unsafe{(radicand as f64).sqrt().to_int_unchecked()})
	}
	#[inline]
	pub fn with_length(&self,length:Planar64)->Self{
		let radicand=(self.0.x as i128)*(self.0.x as i128)+(self.0.y as i128)*(self.0.y as i128)+(self.0.z as i128)*(self.0.z as i128);
		let self_length:i128=unsafe{(radicand as f64).sqrt().to_int_unchecked()};
		//self.0*length/self_length
		Planar64Vec3(
			glam::i64vec3(
				((self.0.x as i128)*(length.0 as i128)/self_length) as i64,
				((self.0.y as i128)*(length.0 as i128)/self_length) as i64,
				((self.0.z as i128)*(length.0 as i128)/self_length) as i64,
			)
		)
	}
}
impl Into<glam::Vec3> for Planar64Vec3{
	fn into(self)->glam::Vec3{
		glam::vec3(
			self.0.x as f32/(1<<32) as f32,
			self.0.y as f32/(1<<32) as f32,
			self.0.z as f32/(1<<32) as f32,
		)
	}
}
impl TryFrom<[f32;3]> for Planar64Vec3{
	type Error=Planar64TryFromFloatError;
	fn try_from(value:[f32;3])->Result<Self,Self::Error>{
		Ok(Self(glam::i64vec3(
			Planar64::try_from(value[0])?.0,
			Planar64::try_from(value[1])?.0,
			Planar64::try_from(value[2])?.0,
		)))
	}
}
impl std::ops::Neg for Planar64Vec3{
	type Output=Planar64Vec3;
	#[inline]
	fn neg(self)->Self::Output{
		Planar64Vec3(-self.0)
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
impl std::ops::Mul<i64> for Planar64Vec3{
	type Output=Planar64Vec3;
	#[inline]
	fn mul(self,rhs:i64)->Self::Output {
		Planar64Vec3(glam::i64vec3(
			self.0.x*rhs,
			self.0.y*rhs,
			self.0.z*rhs
		))
	}
}
impl std::ops::Mul<Time> for Planar64Vec3{
	type Output=Planar64Vec3;
	#[inline]
	fn mul(self,rhs:Time)->Self::Output{
		Planar64Vec3(glam::i64vec3(
			(((self.0.x as i128)*(rhs.0 as i128))/1_000_000_000) as i64,
			(((self.0.y as i128)*(rhs.0 as i128))/1_000_000_000) as i64,
			(((self.0.z as i128)*(rhs.0 as i128))/1_000_000_000) as i64
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
impl Default for Planar64Mat3{
	fn default() -> Self {
		Self{
			x_axis:Planar64Vec3::X,
			y_axis:Planar64Vec3::Y,
			z_axis:Planar64Vec3::Z,
		}
	}
}
impl std::ops::Mul<Planar64Vec3> for Planar64Mat3{
	type Output=Planar64Vec3;
	#[inline]
	fn mul(self,rhs:Planar64Vec3) -> Self::Output {
		self.x_axis*rhs.x()
		+self.y_axis*rhs.y()
		+self.z_axis*rhs.z()
	}
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
		let theta=angle.0 as f64*(std::f64::consts::PI/((1<<31) as f64));
		let (s,c)=theta.sin_cos();
		let (c,s)=(c*((1<<32) as f64),s*((1<<32) as f64));
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
#[derive(Clone,Copy,Default)]
pub struct Planar64Affine3{
	matrix3:Planar64Mat3,//includes scale above 1
	transform:Planar64Vec3,
}

impl Planar64Affine3{
	pub fn new(matrix3:Planar64Mat3,transform:Planar64Vec3)->Self{
		Self{matrix3,transform}
	}
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

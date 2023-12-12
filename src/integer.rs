//integer units
#[derive(Clone,Copy,Hash,Eq,PartialEq,PartialOrd,Debug)]
pub struct Time(i64);
impl Time{
	pub const MIN:Self=Self(i64::MIN);
	pub const MAX:Self=Self(i64::MAX);
	pub const ZERO:Self=Self(0);
	pub const ONE_SECOND:Self=Self(1_000_000_000);
	pub const ONE_MILLISECOND:Self=Self(1_000_000);
	pub const ONE_MICROSECOND:Self=Self(1_000);
	pub const ONE_NANOSECOND:Self=Self(1);
	#[inline]
	pub fn from_secs(num:i64)->Self{
		Self(Self::ONE_SECOND.0*num)
	}
	#[inline]
	pub fn from_millis(num:i64)->Self{
		Self(Self::ONE_MILLISECOND.0*num)
	}
	#[inline]
	pub fn from_micros(num:i64)->Self{
		Self(Self::ONE_MICROSECOND.0*num)
	}
	#[inline]
	pub fn from_nanos(num:i64)->Self{
		Self(Self::ONE_NANOSECOND.0*num)
	}
	//should I have checked subtraction? force all time variables to be positive?
	#[inline]
	pub fn nanos(&self)->i64{
		self.0
	}
}
impl From<Planar64> for Time{
	#[inline]
	fn from(value:Planar64)->Self{
		Time((((value.0 as i128)*1_000_000_000)>>32) as i64)
	}
}
impl std::fmt::Display for Time{
	#[inline]
	fn fmt(&self,f:&mut std::fmt::Formatter<'_>)->std::fmt::Result{
		write!(f,"{}s+{:09}ns",self.0/Self::ONE_SECOND.0,self.0%Self::ONE_SECOND.0)
	}
}
impl std::default::Default for Time{
	fn default()->Self{
		Self(0)
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

#[inline]
const fn gcd(mut a:u64,mut b:u64)->u64{
	while b!=0{
		(a,b)=(b,a.rem_euclid(b));
	};
	a
}
#[derive(Clone,Hash)]
pub struct Ratio64{
	num:i64,
	den:u64,
}
impl Ratio64{
	pub const ZERO:Self=Ratio64{num:0,den:1};
	pub const ONE:Self=Ratio64{num:1,den:1};
	#[inline]
	pub const fn new(num:i64,den:u64)->Option<Ratio64>{
		if den==0{
			None
		}else{
			let d=gcd(num.unsigned_abs(),den);
			Some(Self{num:num/(d as i64),den:den/d})
		}
	}
	#[inline]
	pub fn mul_int(&self,rhs:i64)->i64{
		rhs*self.num/(self.den as i64)
	}
	#[inline]
	pub fn rhs_div_int(&self,rhs:i64)->i64{
		rhs*(self.den as i64)/self.num
	}
	#[inline]
	pub fn mul_ref(&self,rhs:&Ratio64)->Ratio64{
		let (num,den)=(self.num*rhs.num,self.den*rhs.den);
		let d=gcd(num.unsigned_abs(),den);
		Self{
			num:num/(d as i64),
			den:den/d,
		}
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
pub enum Ratio64TryFromFloatError{
	Nan,
	Infinite,
	Subnormal,
	HighlyNegativeExponent(i16),
	HighlyPositiveExponent(i16),
}
const MAX_DENOMINATOR:u128=u64::MAX as u128;
#[inline]
fn ratio64_from_mes((m,e,s):(u64,i16,i8))->Result<Ratio64,Ratio64TryFromFloatError>{
	if e< -127{
		//this can also just be zero
		Err(Ratio64TryFromFloatError::HighlyNegativeExponent(e))
	}else if e< -63{
		//approximate input ratio within denominator limit
		let mut target_num=m as u128;
		let mut target_den=1u128<<-e;

		let mut num=1;
		let mut den=0;
		let mut prev_num=0;
		let mut prev_den=1;

		while target_den!=0{
			let whole=target_num/target_den;
			(target_num,target_den)=(target_den,target_num-whole*target_den);
			let new_num=whole*num+prev_num;
			let new_den=whole*den+prev_den;
			if MAX_DENOMINATOR<new_den{
				break;
			}else{
				(prev_num,prev_den)=(num,den);
				(num,den)=(new_num,new_den);
			}
		}

		Ok(Ratio64::new(num as i64,den as u64).unwrap())
	}else if e<0{
		Ok(Ratio64::new((m as i64)*(s as i64),1<<-e).unwrap())
	}else if (64-m.leading_zeros() as i16)+e<64{
		Ok(Ratio64::new((m as i64)*(s as i64)*(1<<e),1).unwrap())
	}else{
		Err(Ratio64TryFromFloatError::HighlyPositiveExponent(e))
	}
}
impl TryFrom<f32> for Ratio64{
	type Error=Ratio64TryFromFloatError;
	#[inline]
	fn try_from(value:f32)->Result<Self,Self::Error>{
		match value.classify(){
			std::num::FpCategory::Nan=>Err(Self::Error::Nan),
			std::num::FpCategory::Infinite=>Err(Self::Error::Infinite),
			std::num::FpCategory::Zero=>Ok(Self::ZERO),
			std::num::FpCategory::Subnormal
			|std::num::FpCategory::Normal=>ratio64_from_mes(integer_decode_f32(value)),
		}
	}
}
impl TryFrom<f64> for Ratio64{
	type Error=Ratio64TryFromFloatError;
	#[inline]
	fn try_from(value:f64)->Result<Self,Self::Error>{
		match value.classify(){
			std::num::FpCategory::Nan=>Err(Self::Error::Nan),
			std::num::FpCategory::Infinite=>Err(Self::Error::Infinite),
			std::num::FpCategory::Zero=>Ok(Self::ZERO),
			std::num::FpCategory::Subnormal
			|std::num::FpCategory::Normal=>ratio64_from_mes(integer_decode_f64(value)),
		}
	}
}
impl std::ops::Mul<Ratio64> for Ratio64{
	type Output=Ratio64;
	#[inline]
	fn mul(self,rhs:Ratio64)->Self::Output{
		let (num,den)=(self.num*rhs.num,self.den*rhs.den);
		let d=gcd(num.unsigned_abs(),den);
		Self{
			num:num/(d as i64),
			den:den/d,
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
			den:self.den*rhs,
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
	#[inline]
	pub fn new(x:Ratio64,y:Ratio64)->Self{
		Self{x,y}
	}
	#[inline]
	pub fn mul_int(&self,rhs:glam::I64Vec2)->glam::I64Vec2{
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
	pub const FRAC_PI_2:Self=Self(1<<30);
	pub const PI:Self=Self(-1<<31);
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
			(theta_max.0 as u32)
			.wrapping_sub(theta_min.0 as u32)
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
		Unit32(unsafe{((self.0 as f64*ANGLE32_TO_FLOAT64_RADIANS).cos()*UNIT32_ONE_FLOAT64).to_int_unchecked()})
	}
	#[inline]
	pub fn sin(&self)->Unit32{
		//TODO: fix this rounding towards 0
		Unit32(unsafe{((self.0 as f64*ANGLE32_TO_FLOAT64_RADIANS).sin()*UNIT32_ONE_FLOAT64).to_int_unchecked()})
	}
	*/
}
const ANGLE32_TO_FLOAT64_RADIANS:f64=std::f64::consts::PI/((1i64<<31) as f64);
impl Into<f32> for Angle32{
	#[inline]
	fn into(self)->f32{
		(self.0 as f64*ANGLE32_TO_FLOAT64_RADIANS) as f32
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
const UNIT32_ONE_FLOAT64=((1<<30) as f64);
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
	pub const MAX:Self=Self(i64::MAX);
	pub const MIN:Self=Self(i64::MIN);
	#[inline]
	pub const fn int(num:i32)->Self{
		Self(Self::ONE.0*num as i64)
	}
	#[inline]
	pub const fn raw(num:i64)->Self{
		Self(num)
	}
	#[inline]
	pub const fn get(&self)->i64{
		self.0
	}
	#[inline]
	pub fn sqrt(&self)->Self{
		Planar64(unsafe{(((self.0 as i128)<<32) as f64).sqrt().to_int_unchecked()})
	}
	#[inline]
	pub const fn signum_i64(&self)->i64{
		((self.0&(1<<63)!=0) as i64)*2-1
	}
}
const PLANAR64_ONE_FLOAT32:f32=(1u64<<32) as f32;
const PLANAR64_CONVERT_TO_FLOAT32:f32=1.0/PLANAR64_ONE_FLOAT32;
const PLANAR64_ONE_FLOAT64:f64=(1u64<<32) as f64;
impl Into<f32> for Planar64{
	#[inline]
	fn into(self)->f32{
		self.0 as f32*PLANAR64_CONVERT_TO_FLOAT32
	}
}
impl From<Ratio64> for Planar64{
	#[inline]
	fn from(ratio:Ratio64)->Self{
		Self((((ratio.num as i128)<<32)/(ratio.den as i128)) as i64)
	}
}
#[derive(Debug)]
pub enum Planar64TryFromFloatError{
	Nan,
	Infinite,
	Subnormal,
	HighlyNegativeExponent,
	HighlyPositiveExponent,
}
impl TryFrom<f32> for Planar64{
	type Error=Planar64TryFromFloatError;
	#[inline]
	fn try_from(value:f32)->Result<Self,Self::Error>{
		match value.classify(){
			std::num::FpCategory::Nan=>Err(Self::Error::Nan),
			std::num::FpCategory::Infinite=>Err(Self::Error::Infinite),
			std::num::FpCategory::Zero=>Ok(Self::ZERO),
			std::num::FpCategory::Subnormal
			|std::num::FpCategory::Normal=>{
				let planar=value*PLANAR64_ONE_FLOAT32;
				if planar<(i64::MIN as f32)||(i64::MAX as f32)<planar{
					Err(Self::Error::HighlyPositiveExponent)
				}else{
					Ok(Planar64(unsafe{planar.to_int_unchecked()}))
				}
			}
		}
	}
}
impl TryFrom<f64> for Planar64{
	type Error=Planar64TryFromFloatError;
	#[inline]
	fn try_from(value:f64)->Result<Self,Self::Error>{
		match value.classify(){
			std::num::FpCategory::Nan=>Err(Self::Error::Nan),
			std::num::FpCategory::Infinite=>Err(Self::Error::Infinite),
			std::num::FpCategory::Zero=>Ok(Self::ZERO),
			std::num::FpCategory::Subnormal
			|std::num::FpCategory::Normal=>{
				let planar=value*PLANAR64_ONE_FLOAT64;
				if planar<(i64::MIN as f64)||(i64::MAX as f64)<planar{
					Err(Self::Error::HighlyPositiveExponent)
				}else{
					Ok(Planar64(unsafe{planar.to_int_unchecked()}))
				}
			}
		}
	}
}
impl std::fmt::Display for Planar64{
	fn fmt(&self,f:&mut std::fmt::Formatter<'_>)->std::fmt::Result{
		write!(f,"{:.3}",
			Into::<f32>::into(*self),
		)
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
impl std::ops::AddAssign<Planar64> for Planar64{
	#[inline]
	fn add_assign(&mut self,rhs:Self){
		*self=*self+rhs;
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
		Planar64(((self.0 as i128*rhs.0 as i128)>>32) as i64)
	}
}
impl std::ops::Mul<Time> for Planar64{
	type Output=Planar64;
	#[inline]
	fn mul(self,rhs:Time)->Self::Output{
		Planar64(((self.0 as i128*rhs.0 as i128)/1_000_000_000) as i64)
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
		Planar64((((self.0 as i128)<<32)/(rhs.0 as i128)) as i64)
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
	pub const ONE:Self=Self::int(1,1,1);
	pub const X:Self=Self::int(1,0,0);
	pub const Y:Self=Self::int(0,1,0);
	pub const Z:Self=Self::int(0,0,1);
	pub const NEG_X:Self=Self::int(-1,0,0);
	pub const NEG_Y:Self=Self::int(0,-1,0);
	pub const NEG_Z:Self=Self::int(0,0,-1);
	pub const MIN:Self=Planar64Vec3(glam::I64Vec3::MIN);
	pub const MAX:Self=Planar64Vec3(glam::I64Vec3::MAX);
	#[inline]
	pub const fn new(x:Planar64,y:Planar64,z:Planar64)->Self{
		Self(glam::i64vec3(x.0,y.0,z.0))
	}
	#[inline]
	pub const fn int(x:i32,y:i32,z:i32)->Self{
		Self(glam::i64vec3((x as i64)<<32,(y as i64)<<32,(z as i64)<<32))
	}
	#[inline]
	pub const fn raw(x:i64,y:i64,z:i64)->Self{
		Self(glam::i64vec3(x,y,z))
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
		)>>32) as i64)
	}
	#[inline]
	pub fn dot128(&self,rhs:Self)->i128{
		(self.0.x as i128)*(rhs.0.x as i128)+
		(self.0.y as i128)*(rhs.0.y as i128)+
		(self.0.z as i128)*(rhs.0.z as i128)
	}
	#[inline]
	pub fn cross(&self,rhs:Self)->Planar64Vec3{
		Planar64Vec3(glam::i64vec3(
			(((self.0.y as i128)*(rhs.0.z as i128)-(self.0.z as i128)*(rhs.0.y as i128))>>32) as i64,
			(((self.0.z as i128)*(rhs.0.x as i128)-(self.0.x as i128)*(rhs.0.z as i128))>>32) as i64,
			(((self.0.x as i128)*(rhs.0.y as i128)-(self.0.y as i128)*(rhs.0.x as i128))>>32) as i64,
		))
	}
	#[inline]
	pub fn walkable(&self,slope:Planar64,up:Self)->bool{
		let y=self.dot(up);
		let x=self.cross(up).length();
		x*slope<y
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
	#[inline]
	fn into(self)->glam::Vec3{
		glam::vec3(
			self.0.x as f32,
			self.0.y as f32,
			self.0.z as f32,
		)*PLANAR64_CONVERT_TO_FLOAT32
	}
}
impl TryFrom<[f32;3]> for Planar64Vec3{
	type Error=Planar64TryFromFloatError;
	#[inline]
	fn try_from(value:[f32;3])->Result<Self,Self::Error>{
		Ok(Self(glam::i64vec3(
			Planar64::try_from(value[0])?.0,
			Planar64::try_from(value[1])?.0,
			Planar64::try_from(value[2])?.0,
		)))
	}
}
impl TryFrom<glam::Vec3A> for Planar64Vec3{
	type Error=Planar64TryFromFloatError;
	#[inline]
	fn try_from(value:glam::Vec3A)->Result<Self,Self::Error>{
		Ok(Self(glam::i64vec3(
			Planar64::try_from(value.x)?.0,
			Planar64::try_from(value.y)?.0,
			Planar64::try_from(value.z)?.0,
		)))
	}
}
impl std::fmt::Display for Planar64Vec3{
	fn fmt(&self,f:&mut std::fmt::Formatter<'_>)->std::fmt::Result{
		write!(f,"{:.3},{:.3},{:.3}",
			Into::<f32>::into(self.x()),Into::<f32>::into(self.y()),Into::<f32>::into(self.z()),
		)
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
impl std::ops::Mul<Planar64Vec3> for Planar64Vec3{
	type Output=Planar64Vec3;
	#[inline]
	fn mul(self, rhs: Planar64Vec3) -> Self::Output {
		Planar64Vec3(glam::i64vec3(
			(((self.0.x as i128)*(rhs.0.x as i128))>>32) as i64,
			(((self.0.y as i128)*(rhs.0.y as i128))>>32) as i64,
			(((self.0.z as i128)*(rhs.0.z as i128))>>32) as i64
		))
	}
}
impl std::ops::Mul<Planar64> for Planar64Vec3{
	type Output=Planar64Vec3;
	#[inline]
	fn mul(self, rhs: Planar64) -> Self::Output {
		Planar64Vec3(glam::i64vec3(
			(((self.0.x as i128)*(rhs.0 as i128))>>32) as i64,
			(((self.0.y as i128)*(rhs.0 as i128))>>32) as i64,
			(((self.0.z as i128)*(rhs.0 as i128))>>32) as i64
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
impl std::ops::Div<Planar64> for Planar64Vec3{
	type Output=Planar64Vec3;
	#[inline]
	fn div(self,rhs:Planar64)->Self::Output{
		Planar64Vec3(glam::i64vec3(
			(((self.0.x as i128)<<32)/(rhs.0 as i128)) as i64,
			(((self.0.y as i128)<<32)/(rhs.0 as i128)) as i64,
			(((self.0.z as i128)<<32)/(rhs.0 as i128)) as i64,
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
#[derive(Clone,Copy,Hash,Eq,PartialEq)]
pub struct Planar64Mat3{
	x_axis:Planar64Vec3,
	y_axis:Planar64Vec3,
	z_axis:Planar64Vec3,
}
impl Default for Planar64Mat3{
	#[inline]
	fn default() -> Self {
		Self{
			x_axis:Planar64Vec3::X,
			y_axis:Planar64Vec3::Y,
			z_axis:Planar64Vec3::Z,
		}
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
	pub const fn int_from_cols_array(array:[i32;9])->Self{
		Self{
			x_axis:Planar64Vec3::int(array[0],array[1],array[2]),
			y_axis:Planar64Vec3::int(array[3],array[4],array[5]),
			z_axis:Planar64Vec3::int(array[6],array[7],array[8]),
		}
	}
	#[inline]
	pub const fn from_diagonal(diagonal:Planar64Vec3)->Self{
		Self{
			x_axis:Planar64Vec3::raw(diagonal.0.x,0,0),
			y_axis:Planar64Vec3::raw(0,diagonal.0.y,0),
			z_axis:Planar64Vec3::raw(0,0,diagonal.0.z),
		}
	}
	#[inline]
	pub fn from_rotation_yx(yaw:Angle32,pitch:Angle32)->Self{
		let xtheta=yaw.0 as f64*ANGLE32_TO_FLOAT64_RADIANS;
		let (xs,xc)=xtheta.sin_cos();
		let (xc,xs)=(xc*PLANAR64_ONE_FLOAT64,xs*PLANAR64_ONE_FLOAT64);
		let ytheta=pitch.0 as f64*ANGLE32_TO_FLOAT64_RADIANS;
		let (ys,yc)=ytheta.sin_cos();
		let (yc,ys)=(yc*PLANAR64_ONE_FLOAT64,ys*PLANAR64_ONE_FLOAT64);
		//TODO: fix this rounding towards 0
		let (xc,xs):(i64,i64)=(unsafe{xc.to_int_unchecked()},unsafe{xs.to_int_unchecked()});
		let (yc,ys):(i64,i64)=(unsafe{yc.to_int_unchecked()},unsafe{ys.to_int_unchecked()});
		Self::from_cols(
			Planar64Vec3(glam::i64vec3(xc,0,-xs)),
			Planar64Vec3(glam::i64vec3(((xs as i128*ys as i128)>>32) as i64,yc,((xc as i128*ys as i128)>>32) as i64)),
			Planar64Vec3(glam::i64vec3(((xs as i128*yc as i128)>>32) as i64,-ys,((xc as i128*yc as i128)>>32) as i64)),
		)
	}
	#[inline]
	pub fn from_rotation_y(angle:Angle32)->Self{
		let theta=angle.0 as f64*ANGLE32_TO_FLOAT64_RADIANS;
		let (s,c)=theta.sin_cos();
		let (c,s)=(c*PLANAR64_ONE_FLOAT64,s*PLANAR64_ONE_FLOAT64);
		//TODO: fix this rounding towards 0
		let (c,s):(i64,i64)=(unsafe{c.to_int_unchecked()},unsafe{s.to_int_unchecked()});
		Self::from_cols(
			Planar64Vec3(glam::i64vec3(c,0,-s)),
			Planar64Vec3::Y,
			Planar64Vec3(glam::i64vec3(s,0,c)),
		)
	}
	#[inline]
	pub const fn inverse(&self)->Self{
		let det=(
			-self.x_axis.0.z as i128*self.y_axis.0.y as i128*self.z_axis.0.x as i128
			+self.x_axis.0.y as i128*self.y_axis.0.z as i128*self.z_axis.0.x as i128
			+self.x_axis.0.z as i128*self.y_axis.0.x as i128*self.z_axis.0.y as i128
			-self.x_axis.0.x as i128*self.y_axis.0.z as i128*self.z_axis.0.y as i128
			-self.x_axis.0.y as i128*self.y_axis.0.x as i128*self.z_axis.0.z as i128
			+self.x_axis.0.x as i128*self.y_axis.0.y as i128*self.z_axis.0.z as i128
			)>>32;
		Self{
			x_axis:Planar64Vec3::raw((((-(self.y_axis.0.z as i128*self.z_axis.0.y as i128)+self.y_axis.0.y as i128*self.z_axis.0.z as i128)<<32)/det) as i64,(((self.x_axis.0.z as i128*self.z_axis.0.y as i128-self.x_axis.0.y as i128*self.z_axis.0.z as i128)<<32)/det) as i64,(((-(self.x_axis.0.z as i128*self.y_axis.0.y as i128)+self.x_axis.0.y as i128*self.y_axis.0.z as i128)<<32)/det) as i64),
			y_axis:Planar64Vec3::raw((((self.y_axis.0.z as i128*self.z_axis.0.x as i128-self.y_axis.0.x as i128*self.z_axis.0.z as i128)<<32)/det) as i64,(((-(self.x_axis.0.z as i128*self.z_axis.0.x as i128)+self.x_axis.0.x as i128*self.z_axis.0.z as i128)<<32)/det) as i64,(((self.x_axis.0.z as i128*self.y_axis.0.x as i128-self.x_axis.0.x as i128*self.y_axis.0.z as i128)<<32)/det) as i64),
			z_axis:Planar64Vec3::raw((((-(self.y_axis.0.y as i128*self.z_axis.0.x as i128)+self.y_axis.0.x as i128*self.z_axis.0.y as i128)<<32)/det) as i64,(((self.x_axis.0.y as i128*self.z_axis.0.x as i128-self.x_axis.0.x as i128*self.z_axis.0.y as i128)<<32)/det) as i64,(((-(self.x_axis.0.y as i128*self.y_axis.0.x as i128)+self.x_axis.0.x as i128*self.y_axis.0.y as i128)<<32)/det) as i64),
		}
	}
	#[inline]
	pub const fn inverse_times_det(&self)->Self{
		Self{
			x_axis:Planar64Vec3::raw(((-(self.y_axis.0.z as i128*self.z_axis.0.y as i128)+self.y_axis.0.y as i128*self.z_axis.0.z as i128)>>32) as i64,((self.x_axis.0.z as i128*self.z_axis.0.y as i128-self.x_axis.0.y as i128*self.z_axis.0.z as i128)>>32) as i64,((-(self.x_axis.0.z as i128*self.y_axis.0.y as i128)+self.x_axis.0.y as i128*self.y_axis.0.z as i128)>>32) as i64),
			y_axis:Planar64Vec3::raw(((self.y_axis.0.z as i128*self.z_axis.0.x as i128-self.y_axis.0.x as i128*self.z_axis.0.z as i128)>>32) as i64,((-(self.x_axis.0.z as i128*self.z_axis.0.x as i128)+self.x_axis.0.x as i128*self.z_axis.0.z as i128)>>32) as i64,((self.x_axis.0.z as i128*self.y_axis.0.x as i128-self.x_axis.0.x as i128*self.y_axis.0.z as i128)>>32) as i64),
			z_axis:Planar64Vec3::raw(((-(self.y_axis.0.y as i128*self.z_axis.0.x as i128)+self.y_axis.0.x as i128*self.z_axis.0.y as i128)>>32) as i64,((self.x_axis.0.y as i128*self.z_axis.0.x as i128-self.x_axis.0.x as i128*self.z_axis.0.y as i128)>>32) as i64,((-(self.x_axis.0.y as i128*self.y_axis.0.x as i128)+self.x_axis.0.x as i128*self.y_axis.0.y as i128)>>32) as i64),
		}
	}
	#[inline]
	pub const fn transpose(&self)->Self{
		Self{
			x_axis:Planar64Vec3::raw(self.x_axis.0.x,self.y_axis.0.x,self.z_axis.0.x),
			y_axis:Planar64Vec3::raw(self.x_axis.0.y,self.y_axis.0.y,self.z_axis.0.y),
			z_axis:Planar64Vec3::raw(self.x_axis.0.z,self.y_axis.0.z,self.z_axis.0.z),
		}
	}
	#[inline]
	pub const fn determinant(&self)->Planar64{
		Planar64(((
			-self.x_axis.0.z as i128*self.y_axis.0.y as i128*self.z_axis.0.x as i128
			+self.x_axis.0.y as i128*self.y_axis.0.z as i128*self.z_axis.0.x as i128
			+self.x_axis.0.z as i128*self.y_axis.0.x as i128*self.z_axis.0.y as i128
			-self.x_axis.0.x as i128*self.y_axis.0.z as i128*self.z_axis.0.y as i128
			-self.x_axis.0.y as i128*self.y_axis.0.x as i128*self.z_axis.0.z as i128
			+self.x_axis.0.x as i128*self.y_axis.0.y as i128*self.z_axis.0.z as i128
		)>>64) as i64)
	}
}
impl Into<glam::Mat3> for Planar64Mat3{
	#[inline]
	fn into(self)->glam::Mat3{
		glam::Mat3::from_cols(
			self.x_axis.into(),
			self.y_axis.into(),
			self.z_axis.into(),
		)
	}
}
impl TryFrom<glam::Mat3A> for Planar64Mat3{
	type Error=Planar64TryFromFloatError;
	#[inline]
	fn try_from(value:glam::Mat3A)->Result<Self,Self::Error>{
		Ok(Self{
			x_axis:Planar64Vec3::try_from(value.x_axis)?,
			y_axis:Planar64Vec3::try_from(value.y_axis)?,
			z_axis:Planar64Vec3::try_from(value.z_axis)?,
		})
	}
}
impl std::fmt::Display for Planar64Mat3{
	fn fmt(&self,f:&mut std::fmt::Formatter<'_>)->std::fmt::Result{
		write!(f,"\n{:.3},{:.3},{:.3}\n{:.3},{:.3},{:.3}\n{:.3},{:.3},{:.3}",
			Into::<f32>::into(self.x_axis.x()),Into::<f32>::into(self.x_axis.y()),Into::<f32>::into(self.x_axis.z()),
			Into::<f32>::into(self.y_axis.x()),Into::<f32>::into(self.y_axis.y()),Into::<f32>::into(self.y_axis.z()),
			Into::<f32>::into(self.z_axis.x()),Into::<f32>::into(self.z_axis.y()),Into::<f32>::into(self.z_axis.z()),
		)
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
impl std::ops::Div<i64> for Planar64Mat3{
	type Output=Planar64Mat3;
	#[inline]
	fn div(self,rhs:i64)->Self::Output{
		Planar64Mat3{
			x_axis:self.x_axis/rhs,
			y_axis:self.y_axis/rhs,
			z_axis:self.z_axis/rhs,
		}
	}
}

///[-1.0,1.0] = [-2^32,2^32]
#[derive(Clone,Copy,Default,Hash,Eq,PartialEq)]
pub struct Planar64Affine3{
	pub matrix3:Planar64Mat3,//includes scale above 1
	pub translation:Planar64Vec3,
}

impl Planar64Affine3{
	#[inline]
	pub fn new(matrix3:Planar64Mat3,translation:Planar64Vec3)->Self{
		Self{matrix3,translation}
	}
	#[inline]
	pub fn transform_point3(&self,point:Planar64Vec3) -> Planar64Vec3{
		Planar64Vec3(
			self.translation.0
			+(self.matrix3.x_axis*point.x()).0
			+(self.matrix3.y_axis*point.y()).0
			+(self.matrix3.z_axis*point.z()).0
		)
	}
}
impl Into<glam::Mat4> for Planar64Affine3{
	#[inline]
	fn into(self)->glam::Mat4{
		glam::Mat4::from_cols_array(&[
			self.matrix3.x_axis.0.x as f32,self.matrix3.x_axis.0.y as f32,self.matrix3.x_axis.0.z as f32,0.0,
			self.matrix3.y_axis.0.x as f32,self.matrix3.y_axis.0.y as f32,self.matrix3.y_axis.0.z as f32,0.0,
			self.matrix3.z_axis.0.x as f32,self.matrix3.z_axis.0.y as f32,self.matrix3.z_axis.0.z as f32,0.0,
			self.translation.0.x as f32,self.translation.0.y as f32,self.translation.0.z as f32,PLANAR64_ONE_FLOAT32
		])*PLANAR64_CONVERT_TO_FLOAT32
	}
}
impl TryFrom<glam::Affine3A> for Planar64Affine3{
	type Error=Planar64TryFromFloatError;
	fn try_from(value: glam::Affine3A)->Result<Self, Self::Error> {
		Ok(Self{
			matrix3:Planar64Mat3::try_from(value.matrix3)?,
			translation:Planar64Vec3::try_from(value.translation)?
		})
	}
}
impl std::fmt::Display for Planar64Affine3{
	fn fmt(&self,f:&mut std::fmt::Formatter<'_>)->std::fmt::Result{
		write!(f,"translation: {:.3},{:.3},{:.3}\nmatrix3:\n{:.3},{:.3},{:.3}\n{:.3},{:.3},{:.3}\n{:.3},{:.3},{:.3}",
			Into::<f32>::into(self.translation.x()),Into::<f32>::into(self.translation.y()),Into::<f32>::into(self.translation.z()),
			Into::<f32>::into(self.matrix3.x_axis.x()),Into::<f32>::into(self.matrix3.x_axis.y()),Into::<f32>::into(self.matrix3.x_axis.z()),
			Into::<f32>::into(self.matrix3.y_axis.x()),Into::<f32>::into(self.matrix3.y_axis.y()),Into::<f32>::into(self.matrix3.y_axis.z()),
			Into::<f32>::into(self.matrix3.z_axis.x()),Into::<f32>::into(self.matrix3.z_axis.y()),Into::<f32>::into(self.matrix3.z_axis.z()),
		)
	}
}

#[test]
fn test_sqrt(){
	let r=Planar64::int(400);
	assert_eq!(1717986918400,r.get());
	let s=r.sqrt();
	assert_eq!(85899345920,s.get());
}
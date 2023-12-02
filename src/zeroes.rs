//find roots of polynomials
use crate::integer::Planar64;

#[inline]
pub fn zeroes2(a0:Planar64,a1:Planar64,a2:Planar64) -> Vec<Planar64>{
	if a2==Planar64::ZERO{
		return zeroes1(a0, a1);
	}
	let radicand=a1.get() as i128*a1.get() as i128-a2.get() as i128*a0.get() as i128*4;
	if 0<radicand {
		//start with f64 sqrt
		//failure case: 2^63 < sqrt(2^127)
		let planar_radicand=Planar64::raw(unsafe{(radicand as f64).sqrt().to_int_unchecked()});
		//TODO: one or two newtons
		//sort roots ascending and avoid taking the difference of large numbers
		match (Planar64::ZERO<a2,Planar64::ZERO<a1){
			(true, true )=>vec![(-a1-planar_radicand)/(a2*2),(a0*2)/(-a1-planar_radicand)],
			(true, false)=>vec![(a0*2)/(-a1+planar_radicand),(-a1+planar_radicand)/(a2*2)],
			(false,true )=>vec![(a0*2)/(-a1-planar_radicand),(-a1-planar_radicand)/(a2*2)],
			(false,false)=>vec![(-a1+planar_radicand)/(a2*2),(a0*2)/(-a1+planar_radicand)],
		}
	} else if radicand==0 {
		return vec![a1/(a2*-2)];
	} else {
		return vec![];
	}
}
#[inline]
pub fn zeroes1(a0:Planar64,a1:Planar64) -> Vec<Planar64> {
	if a1==Planar64::ZERO{
		return vec![];
	}else{
		let q=((-a0.get() as i128)<<32)/(a1.get() as i128);
		if i64::MIN as i128<=q&&q<=i64::MAX as i128{
			return vec![Planar64::raw(q as i64)];
		}else{
			return vec![];
		}
	}
}
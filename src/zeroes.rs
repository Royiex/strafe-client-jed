//find roots of polynomials
use crate::integer::Planar64;

#[inline]
pub fn zeroes2(a0:Planar64,a1:Planar64,a2:Planar64)->([Planar64;2],usize){
	if a2==Planar64::ZERO{
		let ([ret],ret_len)=zeroes1(a0,a1);
		return ([ret,Planar64::ZERO],ret_len);
	}
	let radicand=a1.get() as i128*a1.get() as i128-a2.get() as i128*a0.get() as i128*4;
	if 0<radicand{
		//start with f64 sqrt
		let planar_radicand=Planar64::raw(unsafe{(radicand as f64).sqrt().to_int_unchecked()});
		//TODO: one or two newtons
		if Planar64::ZERO<a2{
			([(-a1-planar_radicand)/(a2*2),(-a1+planar_radicand)/(a2*2)],2)
		}else{
			([(-a1+planar_radicand)/(a2*2),(-a1-planar_radicand)/(a2*2)],2)
		}
	}else if radicand==0{
		([a1/(a2*-2),Planar64::ZERO],1)
	}else{
		([Planar64::ZERO,Planar64::ZERO],0)
	}
}
#[inline]
pub fn zeroes1(a0:Planar64,a1:Planar64)->([Planar64;1],usize){
	if a1==Planar64::ZERO{
		return ([Planar64::ZERO],0);
	}else{
		return ([-a0/a1],1);
	}
}
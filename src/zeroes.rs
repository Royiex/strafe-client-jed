//find roots of polynomials
use crate::integer::Planar64;

#[inline]
pub fn zeroes2(a0:Planar64,a1:Planar64,a2:Planar64)->impl std::iter::IntoIterator{
	// if a2==Planar64::ZERO{
	// 	return zeroes1(a0,a1);
	// }
	let radicand=a1.get() as i128*a1.get() as i128-a2.get() as i128*a0.get() as i128*4;
	if 0<radicand{
		//start with f64 sqrt
		let planar_radicand=Planar64::raw(unsafe{(radicand as f64).sqrt().to_int_unchecked()});
		//TODO: one or two newtons
		if Planar64::ZERO<a2{
			[(-a1-planar_radicand)/(a2*2),(-a1+planar_radicand)/(a2*2)].into_iter()
		}else{
			[(-a1+planar_radicand)/(a2*2),(-a1-planar_radicand)/(a2*2)].into_iter()
		}
	}else if radicand==0{
		[a1/(a2*-2),Planar64::ZERO].into_iter()
	}else{
		[Planar64::ZERO,Planar64::ZERO].into_iter()
	}
}
#[inline]
pub fn zeroes1(a0:Planar64,a1:Planar64)->impl std::iter::IntoIterator{
	if a1==Planar64::ZERO{
		return [Planar64::ZERO].into_iter();
	}else{
		return [-a0/a1].into_iter();
	}
}
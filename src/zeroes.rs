//find roots of polynomials
pub fn zeroes2(a0:f32,a1:f32,a2:f32) -> Vec<f32>{
	if a2==0f32{
		return zeroes1(a0, a1);
	}
	let mut radicand=a1*a1-4f32*a2*a0;
	if 0f32<radicand {
		radicand=radicand.sqrt();
		if 0f32<a2 {
			return vec![(-a1-radicand)/(2f32*a2),(-a1+radicand)/(2f32*a2)];
		} else {
			return vec![(-a1+radicand)/(2f32*a2),(-a1-radicand)/(2f32*a2)];
		}
	} else if radicand==0f32 {
		return vec![-a1/(2f32*a2)];
	} else {
		return vec![];
	}
}
#[inline]
pub fn zeroes1(a0:f32,a1:f32) -> Vec<f32> {
	if a1==0f32{
		return vec![];
	} else {
		return vec![-a0/a1];
	}
}
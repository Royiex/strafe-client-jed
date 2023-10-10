type TIME=crate::physics::TIME;

pub struct Timescale{
	num:i64,
	den:std::num::NonZeroU64,
}

pub struct Paused{}
pub struct Unpaused{}
pub struct PausedScaled{scale:Timescale}
pub struct UnpausedScaled{scale:Timescale}

pub trait TimerState{}
impl TimerState for Paused{}
impl TimerState for Unpaused{}
impl TimerState for PausedScaled{}
impl TimerState for UnpausedScaled{}

pub trait IsPaused{}
impl IsPaused for Paused{}
impl IsPaused for PausedScaled{}

pub trait IsUnpaused{}
impl IsUnpaused for Unpaused{}
impl IsUnpaused for UnpausedScaled{}

pub trait IsScaled{}
impl IsScaled for PausedScaled{}
impl IsScaled for UnpausedScaled{}

pub trait IsUnscaled{}
impl IsUnscaled for Paused{}
impl IsUnscaled for Unpaused{}

pub struct Timer<State:TimerState>{
	offset:crate::physics::TIME,
	state:State,
}
fn get_offset(time:TIME,write_time:TIME)->TIME{
	write_time-time
}
fn get_offset_scaled(time:TIME,write_time:TIME,scale:&Timescale)->TIME{
	write_time-time*scale.num/scale.den.get() as i64
}

fn paused()->Timer<Paused>{
	Timer{
		offset:0,
		state:Paused{},
	}
}
fn unpaused()->Timer<Unpaused>{
	Timer{
		offset:0,
		state:Unpaused{},
	}
}
fn paused_scaled(scale:Timescale)->Timer<PausedScaled>{
	Timer{
		offset:0,
		state:PausedScaled{scale},
	}
}
fn unpaused_scaled(scale:Timescale)->Timer<UnpausedScaled>{
	Timer{
		offset:0,
		state:UnpausedScaled{scale},
	}
}
impl Timer<Paused>{
	pub fn time(&self)->TIME{
		self.offset
	}
	pub fn unpause(self,time:TIME)->Timer<Unpaused>{
		Timer{
			offset:get_offset(time,self.time()),
			state:Unpaused{},
		}
	}
	pub fn set_time(&mut self,time:TIME,write_time:TIME){
		self.offset=get_offset(time,write_time);
	}
	pub fn set_scale(self,time:TIME,scale:Timescale)->Timer<PausedScaled>{
		Timer{
			offset:get_offset_scaled(time,self.time(),&scale),
			state:PausedScaled{scale},
		}
	}
}
impl Timer<Unpaused>{
	pub fn time(&self,time:TIME)->TIME{
		self.offset+time
	}
	pub fn pause(self,time:TIME)->Timer<Paused>{
		Timer{
			offset:self.time(time),
			state:Paused{},
		}
	}
	pub fn set_time(&mut self,time:TIME,write_time:TIME){
		self.offset=get_offset(time,write_time);
	}
	pub fn set_scale(self,time:TIME,scale:Timescale)->Timer<UnpausedScaled>{
		Timer{
			offset:get_offset_scaled(time,self.time(time),&scale),
			state:UnpausedScaled{scale},
		}
	}
}
impl Timer<PausedScaled>{
	pub fn time(&self)->TIME{
		self.offset
	}
	pub fn unpause(self,time:TIME)->Timer<UnpausedScaled>{
		Timer{
			offset:get_offset_scaled(time,self.time(),&self.state.scale),
			state:UnpausedScaled{scale:self.state.scale},
		}
	}
	pub fn set_time(&mut self,time:TIME,write_time:TIME){
		self.offset=get_offset_scaled(time,write_time,&self.state.scale);
	}
	pub fn set_scale(self,time:TIME,scale:Timescale)->Timer<PausedScaled>{
		Timer{
			offset:get_offset_scaled(time,self.time(),&scale),
			state:PausedScaled{scale},
		}
	}
}
impl Timer<UnpausedScaled>{
	pub fn time(&self,time:TIME)->TIME{
		self.offset+time*self.state.scale.num/self.state.scale.den.get() as i64
	}
	pub fn pause(self,time:TIME)->Timer<PausedScaled>{
		Timer{
			offset:self.time(time),
			state:PausedScaled{scale:self.state.scale},
		}
	}
	pub fn set_time(&mut self,time:TIME,write_time:TIME){
		self.offset=get_offset_scaled(time,write_time,&self.state.scale);
	}
	pub fn set_scale(self,time:TIME,scale:Timescale)->Timer<UnpausedScaled>{
		Timer{
			offset:get_offset_scaled(time,self.time(time),&scale),
			//self.offset+time*self.state.scale.num/self.state.scale.den.get() as i64-time*scale.num/scale.den.get() as i64
			state:UnpausedScaled{scale},
		}
	}
}

#[test]
fn test_timer_unscaled(){
	const ONE_SECOND:TIME=1_000_000_000;
	let run_prepare=paused();

	let run_start=run_prepare.unpause(ONE_SECOND);
	let run_finish=run_start.pause(11*ONE_SECOND);

	assert_eq!(run_finish.time(),10*ONE_SECOND);
}
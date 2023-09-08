pub struct Body {
	pub position: glam::Vec3,//I64 where 2^32 = 1 u
	pub velocity: glam::Vec3,//I64 where 2^32 = 1 u/s
	pub time: TIMESTAMP,//nanoseconds x xxxxD!
}

pub struct PhysicsState {
	pub body: Body,
	pub time: i64,
	pub tick: u32,
	pub gravity: glam::Vec3,
	pub friction: f32,
	pub mv: f32,
	pub grounded: bool,
	pub walkspeed: f32,
}

pub type TIMESTAMP = i64;

const CONTROL_JUMP:u32 = 0b01000000;//temp
impl PhysicsState {
	pub fn run(&mut self, time: TIMESTAMP, control_dir: glam::Vec3, controls: u32){
		let target_tick = (time/10_000_000) as u32;//100t
		//the game code can run for 1 month before running out of ticks
		while self.tick<target_tick {
			self.tick += 1;
			let dt=0.01;
			let d=self.body.velocity.dot(control_dir);
			if d<self.mv {
				self.body.velocity+=(self.mv-d)*control_dir;
			}
			self.body.velocity+=self.gravity*dt;
			self.body.position+=self.body.velocity*dt;
			if self.body.position.y<0.0{
				self.body.position.y=0.0;
				self.body.velocity.y=0.0;
				self.grounded=true;
			}
			if self.grounded&&(controls&CONTROL_JUMP)!=0 {
				self.grounded=false;
				self.body.velocity+=glam::Vec3::new(0.0,0.715588/2.0*100.0,0.0);
			}
			if self.grounded {
				let applied_friction=self.friction*dt;
				let targetv=control_dir*self.walkspeed;
				let diffv=targetv-self.body.velocity;
				if applied_friction*applied_friction<diffv.length_squared() {
					self.body.velocity+=applied_friction*diffv.normalize();
				} else {
					self.body.velocity=targetv;
				}
			}
		}

		self.body.time=target_tick as TIMESTAMP*10_000_000;
	}

	pub fn extrapolate_position(&self, time: TIMESTAMP) -> glam::Vec3 {
		let dt=(time-self.body.time) as f64/1_000_000_000f64;
		self.body.position+self.body.velocity*(dt as f32)+self.gravity*((0.5*dt*dt) as f32)
	}
}
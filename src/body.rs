use crate::event::EventStruct;

pub struct Body {
	pub position: glam::Vec3,//I64 where 2^32 = 1 u
	pub velocity: glam::Vec3,//I64 where 2^32 = 1 u/s
	pub time: TIME,//nanoseconds x xxxxD!
}

pub struct PhysicsState {
	pub body: Body,
	//pub contacts: Vec<RelativeCollision>,
	pub time: TIME,
	pub strafe_tick_num: TIME,
	pub strafe_tick_den: TIME,
	pub tick: u32,
	pub mv: f32,
	pub walkspeed: f32,
	pub friction: f32,
	pub gravity: glam::Vec3,
	pub grounded: bool,
	pub jump_trying: bool,
}

pub type TIME = i64;

const CONTROL_JUMP:u32 = 0b01000000;//temp
impl PhysicsState {
	//delete this, we are tickless gamers
	pub fn run(&mut self, time: TIME, control_dir: glam::Vec3, controls: u32){
		let target_tick = (time*self.strafe_tick_num/self.strafe_tick_den) as u32;
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

		self.body.time=target_tick as TIME*self.strafe_tick_den/self.strafe_tick_num;
	}

	//delete this
	pub fn extrapolate_position(&self, time: TIME) -> glam::Vec3 {
		let dt=(time-self.body.time) as f64/1_000_000_000f64;
		self.body.position+self.body.velocity*(dt as f32)+self.gravity*((0.5*dt*dt) as f32)
	}

	fn next_strafe_event(&self) -> Option<EventStruct> {
		return Some(EventStruct{
			time:(self.time*self.strafe_tick_num/self.strafe_tick_den+1)*self.strafe_tick_den/self.strafe_tick_num,
			event:crate::event::EventEnum::StrafeTick
		});
	}
}

impl crate::event::EventTrait for PhysicsState {
	//this little next event function can cache its return value and invalidate the cached value by watching the State.
	fn next_event(&self) -> Option<EventStruct> {
		//JUST POLLING!!! NO MUTATION
		let mut best_event: Option<EventStruct> = None;
		let collect_event = |test_event:Option<EventStruct>|{
			match test_event {
				Some(unwrap_test_event) => match best_event {
					Some(unwrap_best_event) => if unwrap_test_event.time<unwrap_best_event.time {
						best_event=test_event;
					},
					None => best_event=test_event,
				},
				None => (),
			}
		};
		//autohop (already pressing spacebar; the signal to begin trying to jump is different)
		if self.grounded&&self.jump_trying {
			//scroll will be implemented with InputEvent::Jump(true) but it blocks setting self.jump_trying=true
			collect_event(Some(EventStruct{
				time:self.time,
				event:crate::event::EventEnum::Jump
			}));
		}
		//check for collision stop events with curent contacts
		for collision_data in self.contacts.iter() {
			collect_event(self.predict_collision(collision_data.model));
		}
		//check for collision start events (against every part in the game with no optimization!!)
		for &model in self.world.models {
			collect_event(self.predict_collision(&model));
		}
		//check to see when the next strafe tick is
		collect_event(self.next_strafe_event());
		best_event
	}
}

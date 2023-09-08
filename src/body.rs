pub struct Body {
	pub position: glam::Vec3,//I64 where 2^32 = 1 u
	pub velocity: glam::Vec3,//I64 where 2^32 = 1 u/s
	pub time: TIME,//nanoseconds x xxxxD!
}

pub struct PhysicsState {
	pub body: Body,
	//pub contacts: Vec<RelativeCollision>,
	pub time: TIME,
	pub strafe_tick_rate: TIME,
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

		self.body.time=target_tick as TIME*10_000_000;
	}

	//delete this
	pub fn extrapolate_position(&self, time: TIME) -> glam::Vec3 {
		let dt=(time-self.body.time) as f64/1_000_000_000f64;
		self.body.position+self.body.velocity*(dt as f32)+self.gravity*((0.5*dt*dt) as f32)
	}

	fn next_strafe_event(&self) -> Option<crate::event::EventStruct> {
		return Some(crate::event::EventStruct{
			time:self.time/self.strafe_tick_rate*self.strafe_tick_rate,//this is floor(n) I need ceil(n)+1
			event:crate::event::EventEnum::StrafeTick
		});
	}
}

impl crate::event::EventTrait for PhysicsState {
	//this little next event function can cache its return value and invalidate the cached value by watching the State.
	fn next_event(&self) -> Option<crate::event::EventStruct> {
		//JUST POLLING!!! NO MUTATION
		let mut best_event: Option<crate::event::EventStruct> = None;
		let collect_event = |test_event:Option<crate::event::EventStruct>|{
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
		//check to see if yee need to jump (this is not the way lol)
		if self.grounded&&self.jump_trying {
			//scroll will be implemented with InputEvent::InstantJump rather than InputEvent::Jump(true)
			collect_event(Some(crate::event::EventStruct{
				time:self.time,
				event:crate::event::EventEnum::Jump
			}));
		}
		//check for collision stop events with curent contacts
		for collision_data in self.contacts.iter() {
			collect_event(self.model.predict_collision(collision_data.model));
		}
		//check for collision start events (against every part in the game with no optimization!!)
		for &model in self.world.models {
			collect_event(self.model.predict_collision(&model));
		}
		//check to see when the next strafe tick is
		collect_event(self.next_strafe_event());
		best_event
	}
}

//something that implements body + hitbox can predict collision
impl crate::sweep::PredictCollision for Model {
	fn predict_collision(&self,other:&Model) -> Option<crate::event::EventStruct> {
		//math!
		None
	}
}
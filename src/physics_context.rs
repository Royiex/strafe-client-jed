use crate::integer::Time;
use crate::physics::{MouseState,PhysicsInputInstruction};
use crate::instruction::{TimedInstruction,InstructionConsumer};

#[derive(Debug)]
pub enum InputInstruction {
	MoveMouse(glam::IVec2),
	MoveRight(bool),
	MoveUp(bool),
	MoveBack(bool),
	MoveLeft(bool),
	MoveDown(bool),
	MoveForward(bool),
	Jump(bool),
	Zoom(bool),
	Reset,
	Render,
		//Idle: there were no input events, but the simulation is safe to advance to this timestep
		//for interpolation / networking / playback reasons, most playback heads will always want
		//to be 1 instruction ahead to generate the next state for interpolation.
}

pub struct RenderState{
	physics:crate::physics::PhysicsState,
	graphics:crate::graphics::GraphicsState,
}
impl RenderState{
	pub fn new(user_settings:&crate::settings::UserSettings,indexed_model_instances:crate::model::IndexedModelInstances){

		let mut physics=crate::physics::PhysicsState::default();
		physics.spawn(indexed_model_instances.spawn_point);
		physics.load_user_settings(user_settings);
		physics.generate_models(&indexed_model_instances);

		let mut graphics=Self::new_graphics_state();
		graphics.load_user_settings(user_settings);
		graphics.generate_models(indexed_model_instances);
		//manual reset
	}
	pub fn into_worker(mut self)->crate::worker::QNWorker<TimedInstruction<InputInstruction>>{
		let mut mouse_blocking=true;
		let mut last_mouse_time=self.physics.next_mouse.time;
		let mut timeline=std::collections::VecDeque::new();
		crate::worker::QNWorker::new(move |ins:TimedInstruction<InputInstruction>|{
			let mut render=false;
			if if let Some(phys_input)=match ins.instruction{
				InputInstruction::MoveMouse(m)=>{
					if mouse_blocking{
						//tell the game state which is living in the past about its future
						timeline.push_front(TimedInstruction{
							time:last_mouse_time,
							instruction:PhysicsInputInstruction::SetNextMouse(MouseState{time:ins.time,pos:m}),
						});
					}else{
						//mouse has just started moving again after being still for longer than 10ms.
						//replace the entire mouse interpolation state to avoid an intermediate state with identical m0.t m1.t timestamps which will divide by zero
						timeline.push_front(TimedInstruction{
							time:last_mouse_time,
							instruction:PhysicsInputInstruction::ReplaceMouse(
								MouseState{time:last_mouse_time,pos:self.physics.next_mouse.pos},
								MouseState{time:ins.time,pos:m}
							),
						});
						//delay physics execution until we have an interpolation target
						mouse_blocking=true;
					}
					last_mouse_time=ins.time;
					None
				},
				InputInstruction::MoveForward(s)=>Some(PhysicsInputInstruction::SetMoveForward(s)),
				InputInstruction::MoveLeft(s)=>Some(PhysicsInputInstruction::SetMoveLeft(s)),
				InputInstruction::MoveBack(s)=>Some(PhysicsInputInstruction::SetMoveBack(s)),
				InputInstruction::MoveRight(s)=>Some(PhysicsInputInstruction::SetMoveRight(s)),
				InputInstruction::MoveUp(s)=>Some(PhysicsInputInstruction::SetMoveUp(s)),
				InputInstruction::MoveDown(s)=>Some(PhysicsInputInstruction::SetMoveDown(s)),
				InputInstruction::Jump(s)=>Some(PhysicsInputInstruction::SetJump(s)),
				InputInstruction::Zoom(s)=>Some(PhysicsInputInstruction::SetZoom(s)),
				InputInstruction::Reset=>Some(PhysicsInputInstruction::Reset),
				InputInstruction::Render=>{render=true;Some(PhysicsInputInstruction::Idle)},
			}{
				//non-mouse event
				timeline.push_back(TimedInstruction{
					time:ins.time,
					instruction:phys_input,
				});
				
				if mouse_blocking{
					//assume the mouse has stopped moving after 10ms.
					//shitty mice are 125Hz which is 8ms so this should cover that.
					//setting this to 100us still doesn't print even though it's 10x lower than the polling rate,
					//so mouse events are probably not handled separately from drawing and fire right before it :(
					if Time::from_millis(10)<ins.time-self.physics.next_mouse.time{
						//push an event to extrapolate no movement from
						timeline.push_front(TimedInstruction{
							time:last_mouse_time,
							instruction:PhysicsInputInstruction::SetNextMouse(MouseState{time:ins.time,pos:self.physics.next_mouse.pos}),
						});
						last_mouse_time=ins.time;
						//stop blocking. the mouse is not moving so the physics does not need to live in the past and wait for interpolation targets.
						mouse_blocking=false;
						true
					}else{
						false
					}
				}else{
					//keep this up to date so that it can be used as a known-timestamp
					//that the mouse was not moving when the mouse starts moving again
					last_mouse_time=ins.time;
					true
				}
			}else{
				//mouse event
				true
			}{
				//empty queue
				while let Some(instruction)=timeline.pop_front(){
					self.physics.run(instruction.time);
					self.physics.process_instruction(TimedInstruction{
						time:instruction.time,
						instruction:crate::physics::PhysicsInstruction::Input(instruction.instruction),
					});
				}
			}
			if render{
				self.graphics.render();
			}
		})
	}
}
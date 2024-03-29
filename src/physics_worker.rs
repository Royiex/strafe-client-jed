use crate::physics::{MouseState,PhysicsInputInstruction};
use strafesnet_common::integer::Time;
use strafesnet_common::instruction::{TimedInstruction,InstructionConsumer};
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
}
pub enum Instruction{
	Input(InputInstruction),
	Render,
	Resize(winit::dpi::PhysicalSize<u32>,crate::settings::UserSettings),
	GenerateModels(crate::model::IndexedModelInstances),
	ClearModels,
	//Graphics(crate::graphics_worker::Instruction),
}

	pub fn new(mut physics:crate::physics::PhysicsState,mut graphics_worker:crate::compat_worker::INWorker<crate::graphics_worker::Instruction>)->crate::compat_worker::QNWorker<TimedInstruction<Instruction>>{
		let mut mouse_blocking=true;
		let mut last_mouse_time=physics.next_mouse.time;
		let mut timeline=std::collections::VecDeque::new();
		crate::compat_worker::QNWorker::new(move |ins:TimedInstruction<Instruction>|{
			if if let Some(phys_input)=match &ins.instruction{
				Instruction::Input(input_instruction)=>match input_instruction{
					&InputInstruction::MoveMouse(m)=>{
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
									MouseState{time:last_mouse_time,pos:physics.next_mouse.pos},
									MouseState{time:ins.time,pos:m}
								),
							});
							//delay physics execution until we have an interpolation target
							mouse_blocking=true;
						}
						last_mouse_time=ins.time;
						None
					},
					&InputInstruction::MoveForward(s)=>Some(PhysicsInputInstruction::SetMoveForward(s)),
					&InputInstruction::MoveLeft(s)=>Some(PhysicsInputInstruction::SetMoveLeft(s)),
					&InputInstruction::MoveBack(s)=>Some(PhysicsInputInstruction::SetMoveBack(s)),
					&InputInstruction::MoveRight(s)=>Some(PhysicsInputInstruction::SetMoveRight(s)),
					&InputInstruction::MoveUp(s)=>Some(PhysicsInputInstruction::SetMoveUp(s)),
					&InputInstruction::MoveDown(s)=>Some(PhysicsInputInstruction::SetMoveDown(s)),
					&InputInstruction::Jump(s)=>Some(PhysicsInputInstruction::SetJump(s)),
					&InputInstruction::Zoom(s)=>Some(PhysicsInputInstruction::SetZoom(s)),
					InputInstruction::Reset=>Some(PhysicsInputInstruction::Reset),
				},
				Instruction::GenerateModels(_)=>Some(PhysicsInputInstruction::Idle),
				Instruction::ClearModels=>Some(PhysicsInputInstruction::Idle),
				Instruction::Resize(_,_)=>Some(PhysicsInputInstruction::Idle),
				Instruction::Render=>Some(PhysicsInputInstruction::Idle),
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
					if Time::from_millis(10)<ins.time-physics.next_mouse.time{
						//push an event to extrapolate no movement from
						timeline.push_front(TimedInstruction{
							time:last_mouse_time,
							instruction:PhysicsInputInstruction::SetNextMouse(MouseState{time:ins.time,pos:physics.next_mouse.pos}),
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
					physics.run(instruction.time);
					physics.process_instruction(TimedInstruction{
						time:instruction.time,
						instruction:crate::physics::PhysicsInstruction::Input(instruction.instruction),
					});
				}
			}
			match ins.instruction{
				Instruction::Render=>{
					graphics_worker.send(crate::graphics_worker::Instruction::Render(physics.output(),ins.time,physics.next_mouse.pos)).unwrap();
				},
				Instruction::Resize(size,user_settings)=>{
					graphics_worker.send(crate::graphics_worker::Instruction::Resize(size,user_settings)).unwrap();
				},
				Instruction::GenerateModels(indexed_model_instances)=>{
					physics.generate_models(&indexed_model_instances);
					physics.spawn(indexed_model_instances.spawn_point);
					graphics_worker.send(crate::graphics_worker::Instruction::GenerateModels(indexed_model_instances)).unwrap();
				},
				Instruction::ClearModels=>{
					physics.clear();
					graphics_worker.send(crate::graphics_worker::Instruction::ClearModels).unwrap();
				},
				_=>(),
			}
		})
	}
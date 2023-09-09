pub struct TimedInstruction<I> {
	pub time: crate::body::TIME,
	pub instruction: I,
}

pub trait InstructionEmitter<I> {
	fn next_instruction(&self, time:crate::body::TIME) -> Option<TimedInstruction<I>>;
}
pub trait InstructionConsumer<I> {
	fn process_instruction(&mut self, instruction:TimedInstruction<I>);
}

//PROPER PRIVATE FIELDS!!!
pub struct InstructionCollector<I> {
	time: crate::body::TIME,
	instruction: Option<I>,
}
impl<I> InstructionCollector<I> {
	pub fn new(time:crate::body::TIME) -> Self {
		Self{
			time,
			instruction:None
		}
	}

	pub fn collect(&mut self,instruction:Option<TimedInstruction<I>>){
		match instruction {
			Some(unwrap_instruction) => {
				if unwrap_instruction.time<self.time {
					self.time=unwrap_instruction.time;
					self.instruction=Some(unwrap_instruction.instruction);
				}
			},
			None => (),
		}
	}
	pub fn instruction(self) -> Option<TimedInstruction<I>> {
		//STEAL INSTRUCTION AND DESTROY INSTRUCTIONCOLLECTOR
		match self.instruction {
			Some(instruction)=>Some(TimedInstruction{
				time:self.time,
				instruction
			}),
			None => None,
		}
	}
}
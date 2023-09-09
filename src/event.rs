pub struct TimedInstruction<I> {
	pub time: crate::body::TIME,
	pub instruction: I,
}

pub trait InstructionEmitter<I> {
	fn next_instruction(&self) -> Option<TimedInstruction<I>>;
}
pub trait InstructionConsumer<I> {
	fn process_instruction(&mut self, instruction:TimedInstruction<I>);
}

//PROPER PRIVATE FIELDS!!!
pub struct InstructionCollector<I> {
	instruction: Option<TimedInstruction<I>>,
}
impl<I> InstructionCollector<I> {
	pub fn new() -> Self {
		Self{instruction:None}
	}

	pub fn collect(&mut self,instruction:Option<TimedInstruction<I>>){
		match &instruction {
			Some(unwrap_instruction) => match &self.instruction {
				Some(unwrap_best_instruction) => if unwrap_instruction.time<unwrap_best_instruction.time {
					self.instruction=instruction;
				},
				None => self.instruction=instruction,
			},
			None => (),
		}
	}
	pub fn instruction(self) -> Option<TimedInstruction<I>> {
		//STEAL INSTRUCTION AND DESTROY INSTRUCTIONCOLLECTOR
		return self.instruction
	}
}
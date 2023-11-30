use crate::integer::Time;

#[derive(Debug)]
pub struct TimedInstruction<I>{
	pub time:Time,
	pub instruction:I,
}

pub trait InstructionEmitter<I>{
	fn next_instruction(&self,time_limit:Time)->Option<TimedInstruction<I>>;
}
pub trait InstructionConsumer<I>{
	fn process_instruction(&mut self, instruction:TimedInstruction<I>);
}

//PROPER PRIVATE FIELDS!!!
pub struct InstructionCollector<I>{
	time:Time,
	instruction:Option<I>,
}
impl<I> InstructionCollector<I>{
	pub fn new(time:Time)->Self{
		Self{
			time,
			instruction:None
		}
	}
	#[inline]
	pub fn time(&self)->Time{
		self.time
	}
	pub fn collect(&mut self,instruction:Option<TimedInstruction<I>>){
		match instruction{
			Some(unwrap_instruction)=>{
				if unwrap_instruction.time<self.time {
					self.time=unwrap_instruction.time;
					self.instruction=Some(unwrap_instruction.instruction);
				}
			},
			None=>(),
		}
	}
	pub fn instruction(self)->Option<TimedInstruction<I>>{
		//STEAL INSTRUCTION AND DESTROY INSTRUCTIONCOLLECTOR
		match self.instruction{
			Some(instruction)=>Some(TimedInstruction{
				time:self.time,
				instruction
			}),
			None=>None,
		}
	}
}
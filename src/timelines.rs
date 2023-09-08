type ORDER = u32;

pub struct Tracker {
	order: ORDER,
}

pub struct TimelineInstruction<I>{
	pub order: ORDER,//absolute ordering of instructions which can be used for sorting even when there are multiple simultaneous timestamps
	pub instruction: crate::instruction::TimedInstruction<I>,
}

pub struct Timeline<I>{
	instructions: std::collections::VecDeque<TimelineInstruction<I>>,
	trackers: Vec<Tracker>,//wrong
}

impl<I> Timeline<I>{
	pub fn new() -> Self {
		Self{
			instructions:std::collections::VecDeque::<TimelineInstruction<I>>::new(),
			trackers:Vec::<Tracker>::new(),
		}
	}

	pub fn len(&self) -> usize {
		return self.instructions.len()
	}
	pub fn first(&self) -> Option<&TimelineInstruction<I>> {
		return self.instructions.get(0)
	}
	pub fn last(&self) -> Option<&TimelineInstruction<I>> {
		return self.instructions.get(self.instructions.len()-1)
	}
	pub fn append(&mut self,instruction:TimelineInstruction<I>){
		let i=self.instructions.len();
		self.instructions.push_back(instruction);
		for tracker in self.trackers.iter() {
			tracker.set_active(true);
		}
	}
	pub fn get_index_after_time(&mut self,time:crate::body::TIME) -> usize{
		self.instructions.partition_point(|ins|ins.instruction.time<time)
	}
	pub fn get_index_after_order(&mut self,order:ORDER) -> usize{
		self.instructions.partition_point(|ins|ins.order<order)
	}
}
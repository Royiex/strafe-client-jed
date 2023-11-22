use crate::integer::{Time,Planar64Vec3};
use crate::instruction::TimedInstruction;

/// A C0 continuous trajectory change.
enum TrajectoryC0{
	TargetPointFuture(Planar64Vec3,Time),
	TargetVelocityFuture(Planar64Vec3,Time),
	UseAcceleration(Planar64Vec3),
}

/// A C1 continuous trajectory change.
enum TrajectoryC1{
	TargetPointVelocityFuture(Planar64Vec3,Planar64Vec3,Time),
	UseVelocityAcceleration(Planar64Vec3,Planar64Vec3),
	TargetPointAcceleration(Planar64Vec3,Planar64Vec3),
}

struct Trajectory{
	position:Planar64Vec3,
	velocity:Planar64Vec3,
	acceleration:Planar64Vec3,
}
impl Trajectory{
	pub fn into_body(self,time:Time)->crate::physics::Body{
		crate::physics::Body::new(self.position,self.velocity,self.acceleration,time)
	}
}

//literally option... Option<Option<T>> doesn't seem ergonomic but is effectively what's needed
enum Edit<T>{
	Change(T),
	Unchanged,
}

/// Each property is specified as either changed or staying the same
struct EditAttributes{}
struct EditStyleModifiers{}

enum Instruction{
	/// Keep Position, edit Velocity and Acceleration
	ContinueTrajectoryC0(TrajectoryC0),
	/// Keep Position and Velocity, edit acceleration
	ContinueTrajectoryC1(TrajectoryC1),
	/// Teleport to a new Position, Velocity, and Acceleration
	NewTrajectory(Trajectory),
	/// Change the object transform (teleport)  This is also how the object must rotate!
	EditTransform(crate::integer::Planar64Affine3),
	/// Change specific attributes
	EditAttributes(EditAttributes),
	/// Change specific style modifiers
	EditStyleModifiers(EditStyleModifiers),
}

// The moving platform script init function returns this object's initial state
struct Dynamic{
	/// somehow allow a mutable state function to control the whole thing
	pub next_instruction:Option<TimedInstruction<Instruction>>,
	pub update:Box<dyn FnMut(TimedInstruction<Instruction>)->Option<TimedInstruction<Instruction>>>,
}
impl crate::instruction::InstructionEmitter<Instruction> for Dynamic{
	fn next_instruction(&self,time_limit:Time)->Option<TimedInstruction<Instruction>>{
		self.next_instruction
	}
}
impl crate::instruction::InstructionConsumer<Instruction> for Dynamic{
	fn process_instruction(&mut self,instruction:TimedInstruction<Instruction>){
		self.next_instruction=(self.update)(instruction);
	}
}
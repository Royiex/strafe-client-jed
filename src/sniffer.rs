//file format "sniff"

/* spec

//begin global header

//global metadata (32 bytes)
b"SNFB"
u32 format_version
u64 priming_bytes
	//how many bytes of the file must be read to guarantee all of the expected
	//format-specific metadata is available to facilitate streaming the remaining contents
	//used by the database to guarantee that it serves at least the bare minimum
u128 resource_uuid
	//identifies the file from anywhere for any other file

//global block layout (variable size)
u64 num_blocks
for block_id in 0..num_blocks{
	u64 first_byte
}

//end global header

//begin blocks

//each block is compressed with zstd or gz or something

*/

/* block types
BLOCK_MAP_HEADER:
StyleInfoOverrides style_info_overrides
//bvh goes here
u64 num_nodes
//node 0 parent node is implied to be None
for node_id in 1..num_nodes{
	u64 parent_node
}
//block 0 is the current block, not part of the map data
u64 num_spacial_blocks
for block_id in 1..num_spacial_blocks{
	u64 node_id
	u64 block_id
	Aabb block_extents
}
//ideally spacial blocks are sorted from distance to start zone
//texture blocks are inserted before the first spacial block they are used in

BLOCK_MAP_RESOURCE:
//an individual one of the following:
	- model (IndexedModel)
	- shader (compiled SPIR-V)
	- image (JpegXL)
	- sound (Opus)
	- video (AV1)
	- animation (Trey thing)

BLOCK_MAP_OBJECT:
//an individual one of the following:
	- model instance
	- located resource
//for a list of resources, parse the object.

BLOCK_BOT_HEADER:
u128 map_resource_uuid //which map is this bot running
u128 time_resource_uuid //resource database time
//don't include style info in bot header because it's in the physics state
//blocks are laid out in chronological order, but indices may jump around.
u64 num_segments
for _ in 0..num_segments{
	i64 time //physics_state timestamp
	u64 block_id
}

BLOCK_BOT_SEGMENT:
//format version indicates what version of these structures to use
PhysicsState physics_state 
//to read, greedily decode instructions until eof
loop{
	//delta encode as much as possible (time,mousepos)
	//strafe ticks are implied
	//physics can be implied in an input-only bot file
	TimedInstruction<PhysicsInstruction> instruction
}

BLOCK_DEMO_HEADER:
//timeline of loading maps, player equipment, bots
*/
struct PhysicsInputInstructionDeltaState{
	mouse_pos:glam::IVec2,
	time:crate::integer::Time,
}

//everything must be 4 byte aligned, it's all going to be compressed so don't think too had about saving less than 4 bytes
//8B - 24B
fn write_input_instruction<W:std::io::Write>(state:&mut PhysicsInputInstructionDeltaState,w:&mut W,ins:&crate::instruction::TimedInstruction<crate::physics::PhysicsInputInstruction>){
	let dt=ins.time-state.time;
	//TODO: insert idle instruction if gap is over u32 nanoseconds
	//OR: end the data block! the full state at the start of the next block will contain an absolute timestamp
	w.write(&(dt.nanos() as u32).to_le_bytes());//4B
	let parity=match &ins.instruction{
		crate::physics::PhysicsInputInstruction::SetMoveRight(true)
		|crate::physics::PhysicsInputInstruction::SetMoveUp(true)
		|crate::physics::PhysicsInputInstruction::SetMoveBack(true)
		|crate::physics::PhysicsInputInstruction::SetMoveLeft(true)
		|crate::physics::PhysicsInputInstruction::SetMoveDown(true)
		|crate::physics::PhysicsInputInstruction::SetMoveForward(true)
		|crate::physics::PhysicsInputInstruction::SetJump(true)
		|crate::physics::PhysicsInputInstruction::SetZoom(true)=>1u32<<31,
		crate::physics::PhysicsInputInstruction::SetMoveRight(false)
		|crate::physics::PhysicsInputInstruction::SetMoveUp(false)
		|crate::physics::PhysicsInputInstruction::SetMoveBack(false)
		|crate::physics::PhysicsInputInstruction::SetMoveLeft(false)
		|crate::physics::PhysicsInputInstruction::SetMoveDown(false)
		|crate::physics::PhysicsInputInstruction::SetMoveForward(false)
		|crate::physics::PhysicsInputInstruction::SetJump(false)
		|crate::physics::PhysicsInputInstruction::SetZoom(false)
		|crate::physics::PhysicsInputInstruction::ReplaceMouse(_,_)
		|crate::physics::PhysicsInputInstruction::SetNextMouse(_)
		|crate::physics::PhysicsInputInstruction::Reset
		|crate::physics::PhysicsInputInstruction::Idle=>0u32,
	};
	//instruction id packed with game control parity bit.  This could be 1 byte but it ruins the alignment
	w.write(&(ins.instruction as u32|parity).to_le_bytes());//4B
	match &ins.instruction{
		crate::physics::PhysicsInputInstruction::ReplaceMouse(m0,m1)=>{//16B
			let dm0=m0.pos-state.mouse_pos;
			w.write(&(dm0.x as i16).to_le_bytes());
			w.write(&(dm0.y as i16).to_le_bytes());
			w.write(&((m0.time-ins.time).nanos() as u32).to_le_bytes());
			let dm1=m1.pos-m0.pos;
			w.write(&(dm1.x as i16).to_le_bytes());
			w.write(&(dm1.y as i16).to_le_bytes());
			w.write(&((m1.time-m0.time).nanos() as u32).to_le_bytes());
			state.mouse_pos=m1.pos;
		},
		crate::physics::PhysicsInputInstruction::SetNextMouse(m)=>{//8B
			let dm=m.pos-state.mouse_pos;
			w.write(&(dm.x as i16).to_le_bytes());
			w.write(&(dm.y as i16).to_le_bytes());
			w.write(&((m.time-state.time).nanos() as u32).to_le_bytes());
			state.mouse_pos=m.pos;
		},
		//0B
		crate::physics::PhysicsInputInstruction::SetMoveRight(_)
		|crate::physics::PhysicsInputInstruction::SetMoveUp(_)
		|crate::physics::PhysicsInputInstruction::SetMoveBack(_)
		|crate::physics::PhysicsInputInstruction::SetMoveLeft(_)
		|crate::physics::PhysicsInputInstruction::SetMoveDown(_)
		|crate::physics::PhysicsInputInstruction::SetMoveForward(_)
		|crate::physics::PhysicsInputInstruction::SetJump(_)
		|crate::physics::PhysicsInputInstruction::SetZoom(_)
		|crate::physics::PhysicsInputInstruction::Reset
		|crate::physics::PhysicsInputInstruction::Idle=>(),
	}
	state.time=ins.time;
}
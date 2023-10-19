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
struct InputInstructionCodecState{
	mouse_pos:glam::IVec2,
	time:crate::integer::Time,
}
//8B - 12B
impl InputInstructionCodecState{
	pub fn encode(&mut self,ins:&crate::instruction::TimedInstruction<crate::physics::InputInstruction>)->([u8;12],usize){
		let dt=ins.time-self.time;
		self.time=ins.time;
		let mut data=[0u8;12];
		[data[0],data[1],data[2],data[3]]=(dt.nanos() as u32).to_le_bytes();//4B
		//instruction id packed with game control parity bit.  This could be 1 byte but it ruins the alignment
		[data[4],data[5],data[6],data[7]]=ins.instruction.id().to_le_bytes();//4B
		match &ins.instruction{
			&crate::physics::InputInstruction::MoveMouse(m)=>{//4B
				let dm=m-self.mouse_pos;
				[data[8],data[9]]=(dm.x as i16).to_le_bytes();
				[data[10],data[11]]=(dm.y as i16).to_le_bytes();
				self.mouse_pos=m;
				(data,12)
			},
			//0B
			crate::physics::InputInstruction::MoveRight(_)
			|crate::physics::InputInstruction::MoveUp(_)
			|crate::physics::InputInstruction::MoveBack(_)
			|crate::physics::InputInstruction::MoveLeft(_)
			|crate::physics::InputInstruction::MoveDown(_)
			|crate::physics::InputInstruction::MoveForward(_)
			|crate::physics::InputInstruction::Jump(_)
			|crate::physics::InputInstruction::Zoom(_)
			|crate::physics::InputInstruction::Reset
			|crate::physics::InputInstruction::Idle=>(data,8),
		}
	}
}

//everything must be 4 byte aligned, it's all going to be compressed so don't think too had about saving less than 4 bytes
//TODO: Omit (mouse only?) instructions that don't surround an actual physics instruction
fn write_input_instruction<W:std::io::Write>(state:&mut InputInstructionCodecState,w:&mut W,ins:&crate::instruction::TimedInstruction<crate::physics::InputInstruction>)->Result<usize,std::io::Error>{
	//TODO: insert idle instruction if gap is over u32 nanoseconds
	//TODO: don't write idle instructions
	//OR: end the data block! the full state at the start of the next block will contain an absolute timestamp
	let (data,size)=state.encode(ins);
	w.write(&data[0..size])//8B-12B
}
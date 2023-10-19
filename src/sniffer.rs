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

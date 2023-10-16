use crate::model::{Color4,TextureCoordinate,IndexedModel,IndexedPolygon,IndexedGroup,IndexedVertex};
use crate::integer::Planar64Vec3;

#[derive(Debug)]
pub enum Primitives{
	Sphere,
	Cube,
	Cylinder,
	Wedge,
	CornerWedge,
}
#[derive(Hash,PartialEq,Eq)]
pub enum CubeFace{
	Right,
	Top,
	Back,
	Left,
	Bottom,
	Front,
}
const CUBE_DEFAULT_TEXTURE_COORDS:[TextureCoordinate;4]=[
	TextureCoordinate::new(0.0,0.0),
	TextureCoordinate::new(1.0,0.0),
	TextureCoordinate::new(1.0,1.0),
	TextureCoordinate::new(0.0,1.0),
];
const CUBE_DEFAULT_VERTICES:[Planar64Vec3;8]=[
	Planar64Vec3::int(-1,-1, 1),//0 left bottom back
	Planar64Vec3::int( 1,-1, 1),//1 right bottom back
	Planar64Vec3::int( 1, 1, 1),//2 right top back
	Planar64Vec3::int(-1, 1, 1),//3 left top back
	Planar64Vec3::int(-1, 1,-1),//4 left top front
	Planar64Vec3::int( 1, 1,-1),//5 right top front
	Planar64Vec3::int( 1,-1,-1),//6 right bottom front
	Planar64Vec3::int(-1,-1,-1),//7 left bottom front
];
const CUBE_DEFAULT_NORMALS:[Planar64Vec3;6]=[
	Planar64Vec3::int( 1, 0, 0),//CubeFace::Right
	Planar64Vec3::int( 0, 1, 0),//CubeFace::Top
	Planar64Vec3::int( 0, 0, 1),//CubeFace::Back
	Planar64Vec3::int(-1, 0, 0),//CubeFace::Left
	Planar64Vec3::int( 0,-1, 0),//CubeFace::Bottom
	Planar64Vec3::int( 0, 0,-1),//CubeFace::Front
];
const CUBE_DEFAULT_POLYS:[[[u32;3];4];6]=[
	// right (1, 0, 0)
	[
		[6,2,0],//[vertex,tex,norm]
		[5,1,0],
		[2,0,0],
		[1,3,0],
	],
	// top (0, 1, 0)
	[
		[5,3,1],
		[4,2,1],
		[3,1,1],
		[2,0,1],
	],
	// back (0, 0, 1)
	[
		[0,3,2],
		[1,2,2],
		[2,1,2],
		[3,0,2],
	],
	// left (-1, 0, 0)
	[
		[0,2,3],
		[3,1,3],
		[4,0,3],
		[7,3,3],
	],
	// bottom (0,-1, 0)
	[
		[1,1,4],
		[0,0,4],
		[7,3,4],
		[6,2,4],
	],
	// front (0, 0,-1)
	[
		[4,1,5],
		[5,0,5],
		[6,3,5],
		[7,2,5],
	],
];

#[derive(Hash,PartialEq,Eq)]
pub enum WedgeFace{
	Right,
	TopFront,
	Back,
	Left,
	Bottom,
}
const WEDGE_DEFAULT_NORMALS:[Planar64Vec3;5]=[
	Planar64Vec3::int( 1, 0, 0),//Wedge::Right
	Planar64Vec3::int( 0, 1,-1),//Wedge::TopFront
	Planar64Vec3::int( 0, 0, 1),//Wedge::Back
	Planar64Vec3::int(-1, 0, 0),//Wedge::Left
	Planar64Vec3::int( 0,-1, 0),//Wedge::Bottom
];
/*
local cornerWedgeVerticies = {
	Vector3.new(-1/2,-1/2,-1/2),7
	Vector3.new(-1/2,-1/2, 1/2),0
	Vector3.new( 1/2,-1/2,-1/2),6
	Vector3.new( 1/2,-1/2, 1/2),1
	Vector3.new( 1/2, 1/2,-1/2),5
}
*/
#[derive(Hash,PartialEq,Eq)]
pub enum CornerWedgeFace{
	Right,
	TopBack,
	TopLeft,
	Bottom,
	Front,
}
const CORNERWEDGE_DEFAULT_NORMALS:[Planar64Vec3;5]=[
	Planar64Vec3::int( 1, 0, 0),//CornerWedge::Right
	Planar64Vec3::int( 0, 1, 1),//CornerWedge::BackTop
	Planar64Vec3::int(-1, 1, 0),//CornerWedge::LeftTop
	Planar64Vec3::int( 0,-1, 0),//CornerWedge::Bottom
	Planar64Vec3::int( 0, 0,-1),//CornerWedge::Front
];
//HashMap fits this use case perfectly but feels like using a sledgehammer to drive a nail
pub fn unit_sphere()->crate::model::IndexedModel{
	let mut indexed_model=crate::model::generate_indexed_model_list_from_obj(obj::ObjData::load_buf(&include_bytes!("../models/suzanne.obj")[..]).unwrap(),Color4::ONE).remove(0);
	for pos in indexed_model.unique_pos.iter_mut(){
		*pos=*pos/2;
	}
	indexed_model
}
pub type CubeFaceDescription=std::collections::HashMap::<CubeFace,FaceDescription>;
pub fn unit_cube()->crate::model::IndexedModel{
	let mut t=CubeFaceDescription::new();
	t.insert(CubeFace::Right,FaceDescription::default());
	t.insert(CubeFace::Top,FaceDescription::default());
	t.insert(CubeFace::Back,FaceDescription::default());
	t.insert(CubeFace::Left,FaceDescription::default());
	t.insert(CubeFace::Bottom,FaceDescription::default());
	t.insert(CubeFace::Front,FaceDescription::default());
	generate_partial_unit_cube(t)
}
const TEAPOT_TRANSFORM:crate::integer::Planar64Mat3=crate::integer::Planar64Mat3::int_from_cols_array([0,1,0, -1,0,0, 0,0,1]);
pub fn unit_cylinder()->crate::model::IndexedModel{
	let mut indexed_model=crate::model::generate_indexed_model_list_from_obj(obj::ObjData::load_buf(&include_bytes!("../models/teapot.obj")[..]).unwrap(),Color4::ONE).remove(0);
	for pos in indexed_model.unique_pos.iter_mut(){
		*pos=TEAPOT_TRANSFORM*(*pos)/10;
	}
	indexed_model
}
pub type WedgeFaceDescription=std::collections::HashMap::<WedgeFace,FaceDescription>;
pub fn unit_wedge()->crate::model::IndexedModel{
	let mut t=WedgeFaceDescription::new();
	t.insert(WedgeFace::Right,FaceDescription::default());
	t.insert(WedgeFace::TopFront,FaceDescription::default());
	t.insert(WedgeFace::Back,FaceDescription::default());
	t.insert(WedgeFace::Left,FaceDescription::default());
	t.insert(WedgeFace::Bottom,FaceDescription::default());
	generate_partial_unit_wedge(t)
}
pub type CornerWedgeFaceDescription=std::collections::HashMap::<CornerWedgeFace,FaceDescription>;
pub fn unit_cornerwedge()->crate::model::IndexedModel{
	let mut t=CornerWedgeFaceDescription::new();
	t.insert(CornerWedgeFace::Right,FaceDescription::default());
	t.insert(CornerWedgeFace::TopBack,FaceDescription::default());
	t.insert(CornerWedgeFace::TopLeft,FaceDescription::default());
	t.insert(CornerWedgeFace::Bottom,FaceDescription::default());
	t.insert(CornerWedgeFace::Front,FaceDescription::default());
	generate_partial_unit_cornerwedge(t)
}

#[derive(Clone)]
pub struct FaceDescription{
	pub texture:Option<u32>,
	pub transform:glam::Affine2,
	pub color:Color4,
}
impl std::default::Default for FaceDescription{
	fn default()->Self {
		Self{
			texture:None,
			transform:glam::Affine2::IDENTITY,
			color:Color4::new(1.0,1.0,1.0,0.0),//zero alpha to hide the default texture
		}
	}
}
impl FaceDescription{
	pub fn new(texture:u32,transform:glam::Affine2,color:Color4)->Self{
		Self{texture:Some(texture),transform,color}
	}
	pub fn from_texture(texture:u32)->Self{
		Self{
			texture:Some(texture),
			transform:glam::Affine2::IDENTITY,
			color:Color4::ONE,
		}
	}
}
//TODO: it's probably better to use a shared vertex buffer between all primitives and use indexed rendering instead of generating a unique vertex buffer for each primitive.
//implementation: put all roblox primitives into one model.groups <- this won't work but I forget why
pub fn generate_partial_unit_cube(face_descriptions:CubeFaceDescription)->crate::model::IndexedModel{
	let mut generated_pos=Vec::new();
	let mut generated_tex=Vec::new();
	let mut generated_normal=Vec::new();
	let mut generated_color=Vec::new();
	let mut generated_vertices=Vec::new();
	let mut groups=Vec::new();
	let mut transforms=Vec::new();
	//note that on a cube every vertex is guaranteed to be unique, so there's no need to hash them against existing vertices.
	for (face,face_description) in face_descriptions.into_iter(){
		//assume that scanning short lists is faster than hashing.
		let transform_index=if let Some(transform_index)=transforms.iter().position(|&transform|transform==face_description.transform){
			transform_index
		}else{
			//create new transform_index
			let transform_index=transforms.len();
			transforms.push(face_description.transform);
			for tex in CUBE_DEFAULT_TEXTURE_COORDS{
				generated_tex.push(face_description.transform.transform_point2(tex));
			}
			transform_index
		} as u32;
		let color_index=if let Some(color_index)=generated_color.iter().position(|&color|color==face_description.color){
			color_index
		}else{
			//create new color_index
			let color_index=generated_color.len();
			generated_color.push(face_description.color);
			color_index
		} as u32;
		let face_id=match face{
			CubeFace::Right => 0,
			CubeFace::Top => 1,
			CubeFace::Back => 2,
			CubeFace::Left => 3,
			CubeFace::Bottom => 4,
			CubeFace::Front => 5,
		};
		//always push normal
		let normal_index=generated_normal.len() as u32;
		generated_normal.push(CUBE_DEFAULT_NORMALS[face_id]);
		//push vertices as they are needed
		groups.push(IndexedGroup{
			texture:face_description.texture,
			polys:vec![IndexedPolygon{
				vertices:CUBE_DEFAULT_POLYS[face_id].map(|tup|{
					let pos=CUBE_DEFAULT_VERTICES[tup[0] as usize];
					let pos_index=if let Some(pos_index)=generated_pos.iter().position(|&p|p==pos){
						pos_index
					}else{
						//create new pos_index
						let pos_index=generated_pos.len();
						generated_pos.push(pos);
						pos_index
					} as u32;
					//always push vertex
					let vertex=IndexedVertex{
						pos:pos_index,
						tex:tup[1]+4*transform_index,
						normal:normal_index,
						color:color_index,
					};
					let vert_index=generated_vertices.len();
					generated_vertices.push(vertex);
					vert_index as u32
				}).to_vec(),
			}],
		});
	}
	IndexedModel{
		unique_pos:generated_pos,
		unique_tex:generated_tex,
		unique_normal:generated_normal,
		unique_color:generated_color,
		unique_vertices:generated_vertices,
		groups,
		instances:Vec::new(),
	}
}
//don't think too hard about the copy paste because this is all going into the map tool eventually...
pub fn generate_partial_unit_wedge(face_descriptions:WedgeFaceDescription)->crate::model::IndexedModel{
	let wedge_default_polys=vec![
		// right (1, 0, 0)
		vec![
			[6,2,0],//[vertex,tex,norm]
			[2,0,0],
			[1,3,0],
		],
		// FrontTop (0, 1, -1)
		vec![
			[3,1,1],
			[2,0,1],
			[6,3,1],
			[7,2,1],
		],
		// back (0, 0, 1)
		vec![
			[0,3,2],
			[1,2,2],
			[2,1,2],
			[3,0,2],
		],
		// left (-1, 0, 0)
		vec![
			[0,2,3],
			[3,1,3],
			[7,3,3],
		],
		// bottom (0,-1, 0)
		vec![
			[1,1,4],
			[0,0,4],
			[7,3,4],
			[6,2,4],
		],
	];
	let mut generated_pos=Vec::new();
	let mut generated_tex=Vec::new();
	let mut generated_normal=Vec::new();
	let mut generated_color=Vec::new();
	let mut generated_vertices=Vec::new();
	let mut groups=Vec::new();
	let mut transforms=Vec::new();
	//note that on a cube every vertex is guaranteed to be unique, so there's no need to hash them against existing vertices.
	for (face,face_description) in face_descriptions.into_iter(){
		//assume that scanning short lists is faster than hashing.
		let transform_index=if let Some(transform_index)=transforms.iter().position(|&transform|transform==face_description.transform){
			transform_index
		}else{
			//create new transform_index
			let transform_index=transforms.len();
			transforms.push(face_description.transform);
			for tex in CUBE_DEFAULT_TEXTURE_COORDS{
				generated_tex.push(face_description.transform.transform_point2(tex));
			}
			transform_index
		} as u32;
		let color_index=if let Some(color_index)=generated_color.iter().position(|&color|color==face_description.color){
			color_index
		}else{
			//create new color_index
			let color_index=generated_color.len();
			generated_color.push(face_description.color);
			color_index
		} as u32;
		let face_id=match face{
			WedgeFace::Right => 0,
			WedgeFace::TopFront => 1,
			WedgeFace::Back => 2,
			WedgeFace::Left => 3,
			WedgeFace::Bottom => 4,
		};
		//always push normal
		let normal_index=generated_normal.len() as u32;
		generated_normal.push(WEDGE_DEFAULT_NORMALS[face_id]);
		//push vertices as they are needed
		groups.push(IndexedGroup{
			texture:face_description.texture,
			polys:vec![IndexedPolygon{
				vertices:wedge_default_polys[face_id].iter().map(|tup|{
					let pos=CUBE_DEFAULT_VERTICES[tup[0] as usize];
					let pos_index=if let Some(pos_index)=generated_pos.iter().position(|&p|p==pos){
						pos_index
					}else{
						//create new pos_index
						let pos_index=generated_pos.len();
						generated_pos.push(pos);
						pos_index
					} as u32;
					//always push vertex
					let vertex=IndexedVertex{
						pos:pos_index,
						tex:tup[1]+4*transform_index,
						normal:normal_index,
						color:color_index,
					};
					let vert_index=generated_vertices.len();
					generated_vertices.push(vertex);
					vert_index as u32
				}).collect(),
			}],
		});
	}
	IndexedModel{
		unique_pos:generated_pos,
		unique_tex:generated_tex,
		unique_normal:generated_normal,
		unique_color:generated_color,
		unique_vertices:generated_vertices,
		groups,
		instances:Vec::new(),
	}
}

pub fn generate_partial_unit_cornerwedge(face_descriptions:CornerWedgeFaceDescription)->crate::model::IndexedModel{
	let cornerwedge_default_polys=vec![
		// right (1, 0, 0)
		vec![
			[6,2,0],//[vertex,tex,norm]
			[5,1,0],
			[1,3,0],
		],
		// BackTop (0, 1, 1)
		vec![
			[5,3,1],
			[0,1,1],
			[1,0,1],
		],
		// LeftTop (-1, 1, 0)
		vec![
			[5,3,2],
			[7,2,2],
			[0,1,2],
		],
		// bottom (0,-1, 0)
		vec![
			[1,1,3],
			[0,0,3],
			[7,3,3],
			[6,2,3],
		],
		// front (0, 0,-1)
		vec![
			[5,0,4],
			[6,3,4],
			[7,2,4],
		],
	];
	let mut generated_pos=Vec::new();
	let mut generated_tex=Vec::new();
	let mut generated_normal=Vec::new();
	let mut generated_color=Vec::new();
	let mut generated_vertices=Vec::new();
	let mut groups=Vec::new();
	let mut transforms=Vec::new();
	//note that on a cube every vertex is guaranteed to be unique, so there's no need to hash them against existing vertices.
	for (face,face_description) in face_descriptions.into_iter(){
		//assume that scanning short lists is faster than hashing.
		let transform_index=if let Some(transform_index)=transforms.iter().position(|&transform|transform==face_description.transform){
			transform_index
		}else{
			//create new transform_index
			let transform_index=transforms.len();
			transforms.push(face_description.transform);
			for tex in CUBE_DEFAULT_TEXTURE_COORDS{
				generated_tex.push(face_description.transform.transform_point2(tex));
			}
			transform_index
		} as u32;
		let color_index=if let Some(color_index)=generated_color.iter().position(|&color|color==face_description.color){
			color_index
		}else{
			//create new color_index
			let color_index=generated_color.len();
			generated_color.push(face_description.color);
			color_index
		} as u32;
		let face_id=match face{
			CornerWedgeFace::Right => 0,
			CornerWedgeFace::TopBack => 1,
			CornerWedgeFace::TopLeft => 2,
			CornerWedgeFace::Bottom => 3,
			CornerWedgeFace::Front => 4,
		};
		//always push normal
		let normal_index=generated_normal.len() as u32;
		generated_normal.push(CORNERWEDGE_DEFAULT_NORMALS[face_id]);
		//push vertices as they are needed
		groups.push(IndexedGroup{
			texture:face_description.texture,
			polys:vec![IndexedPolygon{
				vertices:cornerwedge_default_polys[face_id].iter().map(|tup|{
					let pos=CUBE_DEFAULT_VERTICES[tup[0] as usize];
					let pos_index=if let Some(pos_index)=generated_pos.iter().position(|&p|p==pos){
						pos_index
					}else{
						//create new pos_index
						let pos_index=generated_pos.len();
						generated_pos.push(pos);
						pos_index
					} as u32;
					//always push vertex
					let vertex=IndexedVertex{
						pos:pos_index,
						tex:tup[1]+4*transform_index,
						normal:normal_index,
						color:color_index,
					};
					let vert_index=generated_vertices.len();
					generated_vertices.push(vertex);
					vert_index as u32
				}).collect(),
			}],
		});
	}
	IndexedModel{
		unique_pos:generated_pos,
		unique_tex:generated_tex,
		unique_normal:generated_normal,
		unique_color:generated_color,
		unique_vertices:generated_vertices,
		groups,
		instances:Vec::new(),
	}
}

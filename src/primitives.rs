use crate::model::{IndexedModel, IndexedPolygon, IndexedGroup, IndexedVertex};

const CUBE_DEFAULT_TEXTURE_COORDS:[[f32;2];4]=[[0.0,0.0],[1.0,0.0],[1.0,1.0],[0.0,1.0]];
const CUBE_DEFAULT_VERTICES:[[f32;3];8]=[
	[-1.,-1., 1.],//0 left bottom back
	[ 1.,-1., 1.],//1 right bottom back
	[ 1., 1., 1.],//2 right top back
	[-1., 1., 1.],//3 left top back
	[-1., 1.,-1.],//4 left top front
	[ 1., 1.,-1.],//5 right top front
	[ 1.,-1.,-1.],//6 right bottom front
	[-1.,-1.,-1.],//7 left bottom front
];
const CUBE_DEFAULT_NORMALS:[[f32;3];6]=[
	[ 1., 0., 0.],//AabbFace::Right
	[ 0., 1., 0.],//AabbFace::Top
	[ 0., 0., 1.],//AabbFace::Back
	[-1., 0., 0.],//AabbFace::Left
	[ 0.,-1., 0.],//AabbFace::Bottom
	[ 0., 0.,-1.],//AabbFace::Front
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
pub fn the_unit_cube_lol() -> crate::model::IndexedModel{
	generate_partial_unit_cube([Some(FaceDescription::default());6])
}

#[derive(Copy,Clone)]
pub struct FaceDescription{
	pub texture:Option<u32>,
	pub transform:glam::Affine2,
	pub color:glam::Vec4,
}
impl std::default::Default for FaceDescription{
	fn default() -> Self {
		Self{
			texture:None,
			transform:glam::Affine2::IDENTITY,
			color:glam::Vec4::ONE,
		}
	}
}
impl FaceDescription{
	pub fn new(texture:u32,transform:glam::Affine2,color:glam::Vec4)->Self{
		Self{texture:Some(texture),transform,color}
	}
	pub fn from_texture(texture:u32)->Self{
		Self{
			texture:Some(texture),
			transform:glam::Affine2::IDENTITY,
			color:glam::Vec4::ONE,
		}
	}
}
//TODO: it's probably better to use a shared vertex buffer between all primitives and use indexed rendering instead of generating a unique vertex buffer for each primitive.
//implementation: put all roblox primitives into one model.groups
pub fn generate_partial_unit_cube(face_descriptions:[Option<FaceDescription>;6]) -> crate::model::IndexedModel{
	let mut generated_pos=Vec::<[f32;3]>::new();
	let mut generated_tex=Vec::new();
	let mut generated_normal=Vec::new();
	let mut generated_color=Vec::new();
	let mut generated_vertices=Vec::new();
	let mut groups=Vec::new();
	let mut transforms=Vec::new();
	//note that on a cube every vertex is guaranteed to be unique, so there's no need to hash them against existing vertices.
	for (i,maybe_face_description) in face_descriptions.iter().enumerate(){
		if let Some(face_description)=maybe_face_description{
			//assume that scanning short lists is faster than hashing.
			let transform_index=if let Some(transform_index)=transforms.iter().position(|&transform|transform==face_description.transform){
				transform_index
			}else{
				//create new transform_index
				let transform_index=transforms.len();
				transforms.push(face_description.transform);
				for tex in CUBE_DEFAULT_TEXTURE_COORDS{
					generated_tex.push(*face_description.transform.transform_point2(glam::Vec2::from_array(tex)).as_ref());
				}
				transform_index
			} as u32;
			let color_index=if let Some(color_index)=generated_color.iter().position(|color|color==face_description.color.as_ref()){
				color_index
			}else{
				//create new color_index
				let color_index=generated_color.len();
				generated_color.push(*face_description.color.as_ref());
				color_index
			} as u32;
			//always push normal
			let normal_index=generated_normal.len() as u32;
			generated_normal.push(CUBE_DEFAULT_NORMALS[i]);
			//push vertices as they are needed
			groups.push(IndexedGroup{
				texture:face_description.texture,
				polys:vec![IndexedPolygon{
					vertices:CUBE_DEFAULT_POLYS[i].map(|tup|{
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

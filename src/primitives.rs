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
		[6,2,0],//[vert,tex,norm]
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
pub fn the_unit_cube_lol() -> crate::model::ModelData{
	generate_partial_unit_cube([Some(FaceDescription::default());6],None)
}

pub struct FaceDescription{
	transform:glam::Affine2,
	color:glam::Vec4,
}
impl std::default::Default for FaceDescription{
	fn default() -> Self {
		Self{
			transform:glam::Affine2::IDENTITY,
			color:glam::Vec4::ONE,
		}
	}
}
impl FaceDescription{
	pub fn new(transform:glam::Affine2,color:glam::Vec4)->Self{
		Self{transform,color}
	}
}
pub fn generate_partial_unit_cube(face_transforms:[Option<FaceDescription>;6],texture:Option<u32>) -> crate::model::ModelData{	//generate transformed vertices
	let mut generated_verts=Vec::new();
	let mut transforms=Vec::new();
	let mut generated_polys=Vec::new();
	for (i,maybe_transform) in face_transforms.iter().enumerate(){
		if let Some(transform)=maybe_transform{
			let transform_index=if let Some(transform_index)=transforms.iter().position(|&t|t==transform){
				transform_index
			}else{
				//create new transform_index
				let transform_index=transforms.len();
				transforms.push(transform);
				for vert in default_verts{
					generated_verts.push(*transform.transform_point2(glam::vec2(vert[0],vert[1])).as_ref());
				}
				transform_index
			};
			generated_polys.push(obj::SimplePolygon(
				default_polys[i].0.iter().map(
					|&v|obj::IndexTuple(v.0,Some(v.1.unwrap()+4*transform_index),v.2)
				).collect()
			));
		}
	}
	let mut vertices = Vec::new();
	let mut vertex_index = std::collections::HashMap::<obj::IndexTuple,u16>::new();
	let mut entities = Vec::new();
	for group in object.groups {
		let mut indices = Vec::new();
		for poly in group.polys {
			for end_index in 2..poly.0.len() {
				for &index in &[0, end_index - 1, end_index] {
					let vert = poly.0[index];
					if let Some(&i)=vertex_index.get(&vert){
						indices.push(i);
					}else{
						let i=vertices.len() as u16;
						vertices.push(Vertex {
							pos: data.position[vert.0],
							texture: data.texture[vert.1.unwrap()],
							normal: data.normal[vert.2.unwrap()],
							color,
						});
						vertex_index.insert(vert,i);
						indices.push(i);
					}
				}
			}
		}
		entities.push(indices);
	}
	ModelData {
		instances: Vec::new(),
		vertices,
		entities,
		texture,
	}
}

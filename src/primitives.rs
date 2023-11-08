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
const GON:u32=6;
const SPHERE_STUFF:([[[[u32;3];4];((GON-1)*(GON-1)) as usize];6])={
	const CUBE_EDGES:[[u32;2];12]=[[0,1],[0,3],[0,7],[1,2],[1,6],[2,3],[2,5],[3,4],[4,5],[4,7],[5,6],[6,7]];
	const fn get_pos_id(p:[[u32;3];4],tex_id:u32)->u32{
		if p[0][1]==tex_id{
			return p[0][0];
		}
		if p[1][1]==tex_id{
			return p[1][0];
		}
		if p[2][1]==tex_id{
			return p[2][0];
		}
		if p[3][1]==tex_id{
			return p[3][0];
		}
		panic!("tex missing")
	}
	const fn get_edge_id(v0:u32,v1:u32)->(u32,bool){
		(match if v0<v1{[v0,v1]}else{[v1,v0]}{
			[0,1]=>0,
			[0,3]=>1,
			[0,7]=>2,
			[1,2]=>3,
			[1,6]=>4,
			[2,3]=>5,
			[2,5]=>6,
			[3,4]=>7,
			[4,5]=>8,
			[4,7]=>9,
			[5,6]=>10,
			[6,7]=>11,
			_=>panic!(":)")
		},v0<v1)
	}
	const fn get_idx(face_id:u32,v00:u32,v10:u32,v11:u32,v01:u32,x:u32,y:u32)->u32{
		if x==0&&y==0{
			return v00
		}
		if x==0&&y==GON-1{
			return v01
		}
		if x==GON-1&&y==GON-1{
			return v11
		}
		if x==GON-1&&y==0{
			return v10
		}
		if x==0{
			//left edge
			let (edge_id,parity)=get_edge_id(v00,v01);
			if parity{
				return 8+edge_id*(GON-2)+(y-1)
			}else{
				return 8+edge_id*(GON-2)+(GON-(y-1))
			}
		}
		if x==GON-1{
			//right edge
			let (edge_id,parity)=get_edge_id(v10,v11);
			if parity{
				return 8+edge_id*(GON-2)+(y-1)
			}else{
				return 8+edge_id*(GON-2)+(GON-(y-1))
			}
		}
		if y==0{
			//top edge
			let (edge_id,parity)=get_edge_id(v00,v10);
			if parity{
				return 8+edge_id*(GON-2)+(x-1)
			}else{
				return 8+edge_id*(GON-2)+(GON-(x-1))
			}
		}
		if y==GON-1{
			//bottom edge
			let (edge_id,parity)=get_edge_id(v01,v11);
			if parity{
				return 8+edge_id*(GON-2)+(x-1)
			}else{
				return 8+edge_id*(GON-2)+(GON-(x-1))
			}
		}
		return 8+12*(GON-2)+face_id*(GON-2)*(GON-2)+(x-1)+(y-1)*(GON-2)
	}
	//topology (indexed polys)
	let mut polys=[[[[0u32;3];4];((GON-1)*(GON-1)) as usize];6];
	let mut face_id=0;
	while face_id<6{
		let p=CUBE_DEFAULT_POLYS[face_id];
		//vertex ids
		let v00=get_pos_id(p,0);//top left
		let v10=get_pos_id(p,1);//top right
		let v11=get_pos_id(p,2);//bottom right
		let v01=get_pos_id(p,3);//bottom left
		let mut i=0;
		while i<(GON-1)*(GON-1){
			let x=i as u32%GON;
			let y=i as u32/GON;
			let i00=get_idx(face_id as u32,v00,v10,v11,v01,x+0,y+0);
			let i10=get_idx(face_id as u32,v00,v10,v11,v01,x+1,y+0);
			let i11=get_idx(face_id as u32,v00,v10,v11,v01,x+1,y+1);
			let i01=get_idx(face_id as u32,v00,v10,v11,v01,x+0,y+1);
			//[pos,tex,norm]
			polys[face_id][i as usize][0]=[i00,(x+0)+(y+0)*GON,i00];
			polys[face_id][i as usize][1]=[i10,(x+1)+(y+0)*GON,i10];
			polys[face_id][i as usize][2]=[i11,(x+1)+(y+1)*GON,i11];
			polys[face_id][i as usize][3]=[i01,(x+0)+(y+1)*GON,i01];
			i+=1;
		}
		face_id+=1;
	}
	//verts
	const N_VERTS:usize=(8+12*(GON-2)+6*(GON-2)*(GON-2)) as usize;
	let mut verts=[Planar64Vec3::ZERO;N_VERTS];
	let mut i=0;
	while i<8{
		verts[i]=CUBE_DEFAULT_VERTICES[i].normalize();
		i+=1;
	}
	i=0;
	while i<12{
		//
	}
	i=0;
	while i<6{
		//
	}
	//tex
	let mut tex=[TextureCoordinate::new(0.0,0.0);(GON*GON) as usize];
	let mut i=0;
	while i<(GON*GON) as usize{
		let x=i as u32%GON;
		let y=i as u32/GON;
		tex[i]=TextureCoordinate::new(x as f32/(GON-1) as f32,y as f32/(GON-1) as f32);
		i+=1;
	}
	(verts,tex,polys)
};
pub fn unit_sphere()->crate::model::IndexedModel{
	let mut indexed_model=crate::model::generate_indexed_model_list_from_obj(obj::ObjData::load_buf(&include_bytes!("../models/suzanne.obj")[..]).unwrap(),Color4::ONE).remove(0);
	for pos in indexed_model.unique_pos.iter_mut(){
		*pos=*pos/2;
	}
	indexed_model
}
#[derive(Default)]
pub struct CubeFaceDescription([Option<FaceDescription>;6]);
impl CubeFaceDescription{
	pub fn insert(&mut self,index:CubeFace,value:FaceDescription){
		self.0[index as usize]=Some(value);
	}
	pub fn pairs(self)->std::iter::FilterMap<std::iter::Enumerate<std::array::IntoIter<Option<FaceDescription>,6>>,impl FnMut((usize,Option<FaceDescription>))->Option<(usize,FaceDescription)>>{
		self.0.into_iter().enumerate().filter_map(|v|v.1.map(|u|(v.0,u)))
	}
}
pub fn unit_cube()->crate::model::IndexedModel{
	let mut t=CubeFaceDescription::default();
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
#[derive(Default)]
pub struct WedgeFaceDescription([Option<FaceDescription>;5]);
impl WedgeFaceDescription{
	pub fn insert(&mut self,index:WedgeFace,value:FaceDescription){
		self.0[index as usize]=Some(value);
	}
	pub fn pairs(self)->std::iter::FilterMap<std::iter::Enumerate<std::array::IntoIter<Option<FaceDescription>,5>>,impl FnMut((usize,Option<FaceDescription>))->Option<(usize,FaceDescription)>>{
		self.0.into_iter().enumerate().filter_map(|v|v.1.map(|u|(v.0,u)))
	}
}
pub fn unit_wedge()->crate::model::IndexedModel{
	let mut t=WedgeFaceDescription::default();
	t.insert(WedgeFace::Right,FaceDescription::default());
	t.insert(WedgeFace::TopFront,FaceDescription::default());
	t.insert(WedgeFace::Back,FaceDescription::default());
	t.insert(WedgeFace::Left,FaceDescription::default());
	t.insert(WedgeFace::Bottom,FaceDescription::default());
	generate_partial_unit_wedge(t)
}
#[derive(Default)]
pub struct CornerWedgeFaceDescription([Option<FaceDescription>;5]);
impl CornerWedgeFaceDescription{
	pub fn insert(&mut self,index:CornerWedgeFace,value:FaceDescription){
		self.0[index as usize]=Some(value);
	}
	pub fn pairs(self)->std::iter::FilterMap<std::iter::Enumerate<std::array::IntoIter<Option<FaceDescription>,5>>,impl FnMut((usize,Option<FaceDescription>))->Option<(usize,FaceDescription)>>{
		self.0.into_iter().enumerate().filter_map(|v|v.1.map(|u|(v.0,u)))
	}
}
pub fn unit_cornerwedge()->crate::model::IndexedModel{
	let mut t=CornerWedgeFaceDescription::default();
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
	for (face_id,face_description) in face_descriptions.pairs(){
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
	for (face_id,face_description) in face_descriptions.pairs(){
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
	for (face_id,face_description) in face_descriptions.pairs(){
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

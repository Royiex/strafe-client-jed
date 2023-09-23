pub fn the_unit_cube_lol() -> obj::ObjData{
	obj::ObjData{
		position: vec![
			[-1.,-1., 1.],//left bottom back
			[ 1.,-1., 1.],//right bottom back
			[ 1., 1., 1.],//right top back
			[-1., 1., 1.],//left top back
			[-1., 1.,-1.],//left top front
			[ 1., 1.,-1.],//right top front
			[ 1.,-1.,-1.],//right bottom front
			[-1.,-1.,-1.],//left bottom front
		],
		texture: vec![[0.0,0.0],[1.0,0.0],[1.0,1.0],[0.0,1.0]],
		normal: vec![
			[1.,0.,0.],//AabbFace::Right
			[0.,1.,0.],//AabbFace::Top
			[0.,0.,1.],//AabbFace::Back
			[-1.,0.,0.],//AabbFace::Left
			[0.,-1.,0.],//AabbFace::Bottom
			[0.,0.,-1.],//AabbFace::Front
		],
		objects: vec![obj::Object{
			name: "Unit Cube".to_owned(),
			groups: vec![obj::Group{
				name: "Cube Vertices".to_owned(),
				index: 0,
				material: None,
				polys: vec![
					// back (0, 0, 1)
					obj::SimplePolygon(vec![
						obj::IndexTuple(0,Some(0),Some(2)),
						obj::IndexTuple(1,Some(1),Some(2)),
						obj::IndexTuple(2,Some(2),Some(2)),
						obj::IndexTuple(3,Some(3),Some(2)),
					]),
					// front (0, 0,-1)
					obj::SimplePolygon(vec![
						obj::IndexTuple(4,Some(0),Some(5)),
						obj::IndexTuple(5,Some(1),Some(5)),
						obj::IndexTuple(6,Some(2),Some(5)),
						obj::IndexTuple(7,Some(3),Some(5)),
					]),
					// right (1, 0, 0)
					obj::SimplePolygon(vec![
						obj::IndexTuple(6,Some(0),Some(0)),
						obj::IndexTuple(5,Some(1),Some(0)),
						obj::IndexTuple(2,Some(2),Some(0)),
						obj::IndexTuple(1,Some(3),Some(0)),
					]),
					// left (-1, 0, 0)
					obj::SimplePolygon(vec![
						obj::IndexTuple(0,Some(0),Some(3)),
						obj::IndexTuple(3,Some(1),Some(3)),
						obj::IndexTuple(4,Some(2),Some(3)),
						obj::IndexTuple(7,Some(3),Some(3)),
					]),
					// top (0, 1, 0)
					obj::SimplePolygon(vec![
						obj::IndexTuple(5,Some(1),Some(1)),
						obj::IndexTuple(4,Some(0),Some(1)),
						obj::IndexTuple(3,Some(3),Some(1)),
						obj::IndexTuple(2,Some(2),Some(1)),
					]),
					// bottom (0,-1, 0)
					obj::SimplePolygon(vec![
						obj::IndexTuple(1,Some(1),Some(4)),
						obj::IndexTuple(0,Some(0),Some(4)),
						obj::IndexTuple(7,Some(3),Some(4)),
						obj::IndexTuple(6,Some(2),Some(4)),
					]),
				],
			}]
		}],
		material_libs: Vec::new(),
	}
}
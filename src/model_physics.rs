use std::borrow::{Borrow,Cow};
use std::collections::{HashSet,HashMap};
use strafesnet_common::model::{self,MeshId,PolygonIter};
use strafesnet_common::zeroes;
use strafesnet_common::integer::{self,Planar64,Planar64Vec3};

pub trait UndirectedEdge{
	type DirectedEdge:Copy+DirectedEdge;
	fn as_directed(&self,parity:bool)->Self::DirectedEdge;
}
pub trait DirectedEdge{
	type UndirectedEdge:Copy+UndirectedEdge;
	fn as_undirected(&self)->Self::UndirectedEdge;
	fn parity(&self)->bool;
	//this is stupid but may work fine
	fn reverse(&self)-><<Self as DirectedEdge>::UndirectedEdge as UndirectedEdge>::DirectedEdge{
		self.as_undirected().as_directed(!self.parity())
	}
}

#[derive(Debug,Clone,Copy,Hash,id::Id,Eq,PartialEq)]
pub struct MeshVertId(u32);
#[derive(Debug,Clone,Copy,Hash,id::Id,Eq,PartialEq)]
pub struct MeshFaceId(u32);

#[derive(Debug,Clone,Copy,Hash,id::Id,Eq,PartialEq)]
pub struct SubmeshVertId(u32);
#[derive(Debug,Clone,Copy,Hash,id::Id,Eq,PartialEq)]
pub struct SubmeshEdgeId(u32);
/// DirectedEdgeId refers to an EdgeId when undirected.
#[derive(Debug,Clone,Copy,Hash,id::Id,Eq,PartialEq)]
pub struct SubmeshDirectedEdgeId(u32);
#[derive(Debug,Clone,Copy,Hash,id::Id,Eq,PartialEq)]
pub struct SubmeshFaceId(u32);

impl UndirectedEdge for SubmeshEdgeId{
	type DirectedEdge=SubmeshDirectedEdgeId;
	fn as_directed(&self,parity:bool)->SubmeshDirectedEdgeId{
		SubmeshDirectedEdgeId(self.0|((parity as u32)<<(u32::BITS-1)))
	}
}
impl DirectedEdge for SubmeshDirectedEdgeId{
	type UndirectedEdge=SubmeshEdgeId;
	fn as_undirected(&self)->SubmeshEdgeId{
		SubmeshEdgeId(self.0&!(1<<(u32::BITS-1)))
	}
	fn parity(&self)->bool{
		self.0&(1<<(u32::BITS-1))!=0
	}
}

//Vertex <-> Edge <-> Face -> Collide
pub enum FEV<F,E:DirectedEdge,V>{
	Face(F),
	Edge(E::UndirectedEdge),
	Vert(V),
}

//use Unit32 #[repr(C)] for map files
#[derive(Clone,Hash,Eq,PartialEq)]
struct Face{
	normal:Planar64Vec3,
	dot:Planar64,
}
struct Vert(Planar64Vec3);
pub trait MeshQuery<FACE:Clone,EDGE:Clone+DirectedEdge,VERT:Clone>{
	fn edge_n(&self,edge_id:EDGE::UndirectedEdge)->Planar64Vec3{
		let verts=self.edge_verts(edge_id);
		self.vert(verts[1].clone())-self.vert(verts[0].clone())
	}
	fn directed_edge_n(&self,directed_edge_id:EDGE)->Planar64Vec3{
		let verts=self.edge_verts(directed_edge_id.as_undirected());
		(self.vert(verts[1].clone())-self.vert(verts[0].clone()))*((directed_edge_id.parity() as i64)*2-1)
	}
	fn vert(&self,vert_id:VERT)->Planar64Vec3;
	fn face_nd(&self,face_id:FACE)->(Planar64Vec3,Planar64);
	fn face_edges(&self,face_id:FACE)->Cow<Vec<EDGE>>;
	fn edge_faces(&self,edge_id:EDGE::UndirectedEdge)->Cow<[FACE;2]>;
	fn edge_verts(&self,edge_id:EDGE::UndirectedEdge)->Cow<[VERT;2]>;
	fn vert_edges(&self,vert_id:VERT)->Cow<Vec<EDGE>>;
	fn vert_faces(&self,vert_id:VERT)->Cow<Vec<FACE>>;
}
struct FaceRefs{
	edges:Vec<SubmeshDirectedEdgeId>,
	//verts:Vec<VertId>,
}
struct EdgeRefs{
	faces:[SubmeshFaceId;2],//left, right
	verts:[SubmeshVertId;2],//bottom, top
}
struct VertRefs{
	faces:Vec<SubmeshFaceId>,
	edges:Vec<SubmeshDirectedEdgeId>,
}
pub struct PhysicsMeshData{
	//this contains all real and virtual faces used in both the complete mesh and convex submeshes
	//faces are sorted such that all faces that belong to the complete mesh appear first, and then
	//all remaining faces are virtual to operate internal logic of the face crawler
	//and cannot be part of a physics collision
	//virtual faces are only used in convex submeshes.
	faces:Vec<Face>,//MeshFaceId indexes this list
	verts:Vec<Vert>,//MeshVertId indexes this list
}
pub struct PhysicsMeshTopology{
	//mapping of local ids to PhysicsMeshData ids
	faces:Vec<MeshFaceId>,//SubmeshFaceId indexes this list
	verts:Vec<MeshVertId>,//SubmeshVertId indexes this list
	//all ids here are local to this object
	face_topology:Vec<FaceRefs>,
	edge_topology:Vec<EdgeRefs>,
	vert_topology:Vec<VertRefs>,
}
#[derive(Clone,Copy,Hash,id::Id,Eq,PartialEq)]
pub struct PhysicsMeshId(u32);
impl Into<MeshId> for PhysicsMeshId{
	fn into(self)->MeshId{
		MeshId::new(self.0)
	}
}
impl From<MeshId> for PhysicsMeshId{
	fn from(value:MeshId)->Self{
		Self::new(value.get())
	}
}
#[derive(Debug,Default,Clone,Copy,Hash,id::Id,Eq,PartialEq)]
pub struct PhysicsSubmeshId(u32);
pub struct PhysicsMesh{
	data:PhysicsMeshData,
	complete_mesh:PhysicsMeshTopology,
	//Most objects in roblox maps are already convex, so the list length is 0
	//as soon as the mesh is divided into 2 submeshes, the list length jumps to 2.
	//length 1 is unnecessary since the complete mesh would be a duplicate of the only submesh, but would still function properly
	submeshes:Vec<PhysicsMeshTopology>,
}
impl PhysicsMesh{
	pub fn unit_cube()->Self{
		//go go gadget debug print mesh
		let data=PhysicsMeshData{
			faces:vec![
				Face{normal:Planar64Vec3::raw( 4294967296, 0, 0),dot:Planar64::raw(4294967296)},
				Face{normal:Planar64Vec3::raw( 0, 4294967296, 0),dot:Planar64::raw(4294967296)},
				Face{normal:Planar64Vec3::raw( 0, 0, 4294967296),dot:Planar64::raw(4294967296)},
				Face{normal:Planar64Vec3::raw(-4294967296, 0, 0),dot:Planar64::raw(4294967296)},
				Face{normal:Planar64Vec3::raw( 0,-4294967296, 0),dot:Planar64::raw(4294967296)},
				Face{normal:Planar64Vec3::raw( 0, 0,-4294967296),dot:Planar64::raw(4294967296)}
			],
			verts:vec![
				Vert(Planar64Vec3::raw( 4294967296,-4294967296,-4294967296)),
				Vert(Planar64Vec3::raw( 4294967296, 4294967296,-4294967296)),
				Vert(Planar64Vec3::raw( 4294967296, 4294967296, 4294967296)),
				Vert(Planar64Vec3::raw( 4294967296,-4294967296, 4294967296)),
				Vert(Planar64Vec3::raw(-4294967296, 4294967296,-4294967296)),
				Vert(Planar64Vec3::raw(-4294967296, 4294967296, 4294967296)),
				Vert(Planar64Vec3::raw(-4294967296,-4294967296, 4294967296)),
				Vert(Planar64Vec3::raw(-4294967296,-4294967296,-4294967296))
			]
		};
		let mesh_topology=PhysicsMeshTopology{
			faces:(0..data.faces.len() as u32).map(MeshFaceId::new).collect(),
			verts:(0..data.verts.len() as u32).map(MeshVertId::new).collect(),
			face_topology:vec![
				FaceRefs{edges:vec![SubmeshDirectedEdgeId((9223372036854775808u64-(1<<63)+(1<<31)) as u32),SubmeshDirectedEdgeId((9223372036854775809u64-(1<<63)+(1<<31)) as u32),SubmeshDirectedEdgeId((9223372036854775810u64-(1<<63)+(1<<31)) as u32),SubmeshDirectedEdgeId(3)]},
				FaceRefs{edges:vec![SubmeshDirectedEdgeId((9223372036854775812u64-(1<<63)+(1<<31)) as u32),SubmeshDirectedEdgeId((9223372036854775813u64-(1<<63)+(1<<31)) as u32),SubmeshDirectedEdgeId(6),SubmeshDirectedEdgeId(1)]},
				FaceRefs{edges:vec![SubmeshDirectedEdgeId(7),SubmeshDirectedEdgeId(2),SubmeshDirectedEdgeId((9223372036854775814u64-(1<<63)+(1<<31)) as u32),SubmeshDirectedEdgeId((9223372036854775816u64-(1<<63)+(1<<31)) as u32)]},
				FaceRefs{edges:vec![SubmeshDirectedEdgeId(8),SubmeshDirectedEdgeId(5),SubmeshDirectedEdgeId((9223372036854775817u64-(1<<63)+(1<<31)) as u32),SubmeshDirectedEdgeId(10)]},
				FaceRefs{edges:vec![SubmeshDirectedEdgeId((9223372036854775815u64-(1<<63)+(1<<31)) as u32),SubmeshDirectedEdgeId((9223372036854775818u64-(1<<63)+(1<<31)) as u32),SubmeshDirectedEdgeId(11),SubmeshDirectedEdgeId((9223372036854775811u64-(1<<63)+(1<<31)) as u32)]},
				FaceRefs{edges:vec![SubmeshDirectedEdgeId(4),SubmeshDirectedEdgeId(0),SubmeshDirectedEdgeId((9223372036854775819u64-(1<<63)+(1<<31)) as u32),SubmeshDirectedEdgeId(9)]}
			],
			edge_topology:vec![
				EdgeRefs{faces:[SubmeshFaceId(0),SubmeshFaceId(5)],verts:[SubmeshVertId(0),SubmeshVertId(1)]}, 
				EdgeRefs{faces:[SubmeshFaceId(0),SubmeshFaceId(1)],verts:[SubmeshVertId(1),SubmeshVertId(2)]}, 
				EdgeRefs{faces:[SubmeshFaceId(0),SubmeshFaceId(2)],verts:[SubmeshVertId(2),SubmeshVertId(3)]}, 
				EdgeRefs{faces:[SubmeshFaceId(4),SubmeshFaceId(0)],verts:[SubmeshVertId(0),SubmeshVertId(3)]}, 
				EdgeRefs{faces:[SubmeshFaceId(1),SubmeshFaceId(5)],verts:[SubmeshVertId(1),SubmeshVertId(4)]}, 
				EdgeRefs{faces:[SubmeshFaceId(1),SubmeshFaceId(3)],verts:[SubmeshVertId(4),SubmeshVertId(5)]}, 
				EdgeRefs{faces:[SubmeshFaceId(2),SubmeshFaceId(1)],verts:[SubmeshVertId(2),SubmeshVertId(5)]}, 
				EdgeRefs{faces:[SubmeshFaceId(4),SubmeshFaceId(2)],verts:[SubmeshVertId(3),SubmeshVertId(6)]}, 
				EdgeRefs{faces:[SubmeshFaceId(2),SubmeshFaceId(3)],verts:[SubmeshVertId(5),SubmeshVertId(6)]}, 
				EdgeRefs{faces:[SubmeshFaceId(3),SubmeshFaceId(5)],verts:[SubmeshVertId(4),SubmeshVertId(7)]}, 
				EdgeRefs{faces:[SubmeshFaceId(4),SubmeshFaceId(3)],verts:[SubmeshVertId(6),SubmeshVertId(7)]}, 
				EdgeRefs{faces:[SubmeshFaceId(5),SubmeshFaceId(4)],verts:[SubmeshVertId(0),SubmeshVertId(7)]}
			],
			vert_topology:vec![
				VertRefs{faces:vec![SubmeshFaceId(0),SubmeshFaceId(4),SubmeshFaceId(5)],edges:vec![SubmeshDirectedEdgeId((9223372036854775811u64-(1<<63)+(1<<31)) as u32),SubmeshDirectedEdgeId((9223372036854775819u64-(1<<63)+(1<<31)) as u32),SubmeshDirectedEdgeId((9223372036854775808u64-(1<<63)+(1<<31)) as u32)]},
				VertRefs{faces:vec![SubmeshFaceId(0),SubmeshFaceId(5),SubmeshFaceId(1)],edges:vec![SubmeshDirectedEdgeId((9223372036854775812u64-(1<<63)+(1<<31)) as u32),SubmeshDirectedEdgeId(0),SubmeshDirectedEdgeId((9223372036854775809u64-(1<<63)+(1<<31)) as u32)]},
				VertRefs{faces:vec![SubmeshFaceId(0),SubmeshFaceId(2),SubmeshFaceId(1)],edges:vec![SubmeshDirectedEdgeId(1),SubmeshDirectedEdgeId((9223372036854775810u64-(1<<63)+(1<<31)) as u32),SubmeshDirectedEdgeId((9223372036854775814u64-(1<<63)+(1<<31)) as u32)]},
				VertRefs{faces:vec![SubmeshFaceId(0),SubmeshFaceId(2),SubmeshFaceId(4)],edges:vec![SubmeshDirectedEdgeId(2),SubmeshDirectedEdgeId(3),SubmeshDirectedEdgeId((9223372036854775815u64-(1<<63)+(1<<31)) as u32)]},
				VertRefs{faces:vec![SubmeshFaceId(3),SubmeshFaceId(5),SubmeshFaceId(1)],edges:vec![SubmeshDirectedEdgeId(4),SubmeshDirectedEdgeId((9223372036854775817u64-(1<<63)+(1<<31)) as u32),SubmeshDirectedEdgeId((9223372036854775813u64-(1<<63)+(1<<31)) as u32)]},
				VertRefs{faces:vec![SubmeshFaceId(2),SubmeshFaceId(3),SubmeshFaceId(1)],edges:vec![SubmeshDirectedEdgeId(5),SubmeshDirectedEdgeId(6),SubmeshDirectedEdgeId((9223372036854775816u64-(1<<63)+(1<<31)) as u32)]},
				VertRefs{faces:vec![SubmeshFaceId(2),SubmeshFaceId(3),SubmeshFaceId(4)],edges:vec![SubmeshDirectedEdgeId(7),SubmeshDirectedEdgeId(8),SubmeshDirectedEdgeId((9223372036854775818u64-(1<<63)+(1<<31)) as u32)]},
				VertRefs{faces:vec![SubmeshFaceId(4),SubmeshFaceId(3),SubmeshFaceId(5)],edges:vec![SubmeshDirectedEdgeId(10),SubmeshDirectedEdgeId(11),SubmeshDirectedEdgeId(9)]}
			]
		};
		Self{
			data,
			complete_mesh:mesh_topology,
			submeshes:Vec::new(),
		}
	}
	pub fn unit_cylinder()->Self{
		Self::unit_cube()
	}
	#[inline]
	pub const fn complete_mesh(&self)->&PhysicsMeshTopology{
		&self.complete_mesh
	}
	#[inline]
	pub const fn complete_mesh_view(&self)->PhysicsMeshView{
		PhysicsMeshView{
			data:&self.data,
			topology:self.complete_mesh(),
		}
	}
	#[inline]
	pub fn submeshes(&self)->&[PhysicsMeshTopology]{
		//the complete mesh is already a convex mesh when len()==0, len()==1 is invalid but will still work
		if self.submeshes.len()==0{
			std::slice::from_ref(&self.complete_mesh)
		}else{
			&self.submeshes.as_slice()
		}
	}
	#[inline]
	pub fn submesh_view(&self,submesh_id:PhysicsSubmeshId)->PhysicsMeshView{
		PhysicsMeshView{
			data:&self.data,
			topology:&self.submeshes()[submesh_id.get() as usize],
		}
	}
	pub fn submesh_views(&self)->impl Iterator<Item=PhysicsMeshView>{
		self.submeshes().iter().map(|topology|PhysicsMeshView{
			data:&self.data,
			topology,
		})
	}
}

//mesh builder code
#[derive(Default,Clone)]
struct VertRefGuy{
	edges:HashSet<SubmeshDirectedEdgeId>,
	faces:HashSet<SubmeshFaceId>,
}
#[derive(Clone,Hash,Eq,PartialEq)]
struct EdgeRefVerts([SubmeshVertId;2]);
impl EdgeRefVerts{
	const fn new(v0:SubmeshVertId,v1:SubmeshVertId)->(Self,bool){
		(if v0.0<v1.0{
			Self([v0,v1])
		}else{
			Self([v1,v0])
		},v0.0<v1.0)
	}
}
struct EdgeRefFaces([SubmeshFaceId;2]);
impl EdgeRefFaces{
	const fn new()->Self{
		Self([SubmeshFaceId(0);2])
	}
	fn push(&mut self,i:usize,face_id:SubmeshFaceId){
		self.0[i]=face_id;
	}
}
struct FaceRefEdges(Vec<SubmeshDirectedEdgeId>);
#[derive(Default)]
struct EdgePool{
	edge_guys:Vec<(EdgeRefVerts,EdgeRefFaces)>,
	edge_id_from_guy:HashMap<EdgeRefVerts,SubmeshEdgeId>,
}
impl EdgePool{
	fn push(&mut self,edge_ref_verts:EdgeRefVerts)->(&mut EdgeRefFaces,SubmeshEdgeId){
		let edge_id=if let Some(&edge_id)=self.edge_id_from_guy.get(&edge_ref_verts){
			edge_id
		}else{
			let edge_id=SubmeshEdgeId::new(self.edge_guys.len() as u32);
			self.edge_guys.push((edge_ref_verts.clone(),EdgeRefFaces::new()));
			self.edge_id_from_guy.insert(edge_ref_verts,edge_id);
			edge_id
		};
		(&mut unsafe{self.edge_guys.get_unchecked_mut(edge_id.get() as usize)}.1,edge_id)
	}
}
impl From<&model::Mesh> for PhysicsMesh{
	fn from(mesh:&model::Mesh)->Self{
		assert!(mesh.unique_pos.len()!=0,"Mesh cannot have 0 vertices");
		let verts=mesh.unique_pos.iter().copied().map(Vert).collect();
		let mut faces=Vec::new();
		let mut face_id_from_face=HashMap::new();
		let mut mesh_topologies:Vec<PhysicsMeshTopology>=mesh.physics_groups.iter().map(|physics_group|{
			//construct submesh
			let mut submesh_faces=Vec::new();//these contain a map from submeshId->meshId
			let mut submesh_verts=Vec::new();
			let mut submesh_vert_id_from_mesh_vert_id=HashMap::<MeshVertId,SubmeshVertId>::new();
			//lazy closure
			let mut get_submesh_vert_id=|vert_id:MeshVertId|{
				if let Some(&submesh_vert_id)=submesh_vert_id_from_mesh_vert_id.get(&vert_id){
					submesh_vert_id
				}else{
					let submesh_vert_id=SubmeshVertId::new(submesh_verts.len() as u32);
					submesh_verts.push(vert_id);
					submesh_vert_id_from_mesh_vert_id.insert(vert_id,submesh_vert_id);
					submesh_vert_id
				}
			};
			let mut edge_pool=EdgePool::default();
			let mut vert_ref_guys=vec![VertRefGuy::default();mesh.unique_pos.len()];
			let mut face_ref_guys=Vec::new();
			for polygon_group_id in &physics_group.groups{
				let polygon_group=&mesh.polygon_groups[polygon_group_id.get() as usize];
				for poly_vertices in polygon_group.polys(){
					let submesh_face_id=SubmeshFaceId::new(submesh_faces.len() as u32);
					//one face per poly
					let mut normal=Planar64Vec3::ZERO;
					let len=poly_vertices.len();
					let face_edges=poly_vertices.into_iter().enumerate().map(|(i,vert_id)|{
						let vert0_id=MeshVertId::new(mesh.unique_vertices[vert_id.get() as usize].pos.get() as u32);
						let vert1_id=MeshVertId::new(mesh.unique_vertices[poly_vertices[(i+1)%len].get() as usize].pos.get() as u32);
						//index submesh verts
						let submesh_vert0_id=get_submesh_vert_id(vert0_id);
						let submesh_vert1_id=get_submesh_vert_id(vert1_id);
						//https://www.khronos.org/opengl/wiki/Calculating_a_Surface_Normal (Newell's Method)
						let v0=mesh.unique_pos[vert0_id.get() as usize];
						let v1=mesh.unique_pos[vert1_id.get() as usize];
						normal+=Planar64Vec3::new(
							(v0.y()-v1.y())*(v0.z()+v1.z()),
							(v0.z()-v1.z())*(v0.x()+v1.x()),
							(v0.x()-v1.x())*(v0.y()+v1.y()),
						);
						//get/create edge and push face into it
						let (edge_ref_verts,is_sorted)=EdgeRefVerts::new(submesh_vert0_id,submesh_vert1_id);
						let (edge_ref_faces,edge_id)=edge_pool.push(edge_ref_verts);
						//polygon vertices as assumed to be listed clockwise
						//populate the edge face on the left or right depending on how the edge vertices got sorted
						edge_ref_faces.push(!is_sorted as usize,submesh_face_id);
						//index edges & face into vertices
						{
							let vert_ref_guy=unsafe{vert_ref_guys.get_unchecked_mut(submesh_vert0_id.get() as usize)};
							vert_ref_guy.edges.insert(edge_id.as_directed(is_sorted));
							vert_ref_guy.faces.insert(submesh_face_id);
							unsafe{vert_ref_guys.get_unchecked_mut(submesh_vert1_id.get() as usize)}.edges.insert(edge_id.as_directed(!is_sorted));
						}
						//return directed_edge_id
						edge_id.as_directed(is_sorted)
					}).collect();
					//choose precision loss randomly idk
					normal=normal/len as i64;
					let mut dot=Planar64::ZERO;
					for &v in poly_vertices{
						dot+=normal.dot(mesh.unique_pos[mesh.unique_vertices[v.get() as usize].pos.get() as usize]);
					}
					//assume face hash is stable, and there are no flush faces...
					let face=Face{normal,dot:dot/len as i64};
					let face_id=match face_id_from_face.get(&face){
						Some(&face_id)=>face_id,
						None=>{
							let face_id=MeshFaceId::new(faces.len() as u32);
							face_id_from_face.insert(face.clone(),face_id);
							faces.push(face);
							face_id
						}
					};
					submesh_faces.push(face_id);
					face_ref_guys.push(FaceRefEdges(face_edges));
				}
			}
			PhysicsMeshTopology{
				faces:submesh_faces,
				verts:submesh_verts,
				face_topology:face_ref_guys.into_iter().map(|face_ref_guy|{
					FaceRefs{edges:face_ref_guy.0}
				}).collect(),
				edge_topology:edge_pool.edge_guys.into_iter().map(|(edge_ref_verts,edge_ref_faces)|
					EdgeRefs{faces:edge_ref_faces.0,verts:edge_ref_verts.0}
				).collect(),
				vert_topology:vert_ref_guys.into_iter().map(|vert_ref_guy|
					VertRefs{
						edges:vert_ref_guy.edges.into_iter().collect(),
						faces:vert_ref_guy.faces.into_iter().collect(),
					}
				).collect(),
			}
		}).collect();
		Self{
			data:PhysicsMeshData{
				faces,
				verts,
			},
			complete_mesh:mesh_topologies.pop().unwrap(),
			submeshes:mesh_topologies,
		}
	}
}

pub struct PhysicsMeshView<'a>{
	data:&'a PhysicsMeshData,
	topology:&'a PhysicsMeshTopology,
}
impl MeshQuery<SubmeshFaceId,SubmeshDirectedEdgeId,SubmeshVertId> for PhysicsMeshView<'_>{
	fn face_nd(&self,face_id:SubmeshFaceId)->(Planar64Vec3,Planar64){
		let face_idx=self.topology.faces[face_id.get() as usize].get() as usize;
		(self.data.faces[face_idx].normal,self.data.faces[face_idx].dot)
	}
	//ideally I never calculate the vertex position, but I have to for the graphical meshes...
	fn vert(&self,vert_id:SubmeshVertId)->Planar64Vec3{
		let vert_idx=self.topology.verts[vert_id.get() as usize].get() as usize;
		self.data.verts[vert_idx].0
	}
	fn face_edges(&self,face_id:SubmeshFaceId)->Cow<Vec<SubmeshDirectedEdgeId>>{
		Cow::Borrowed(&self.topology.face_topology[face_id.get() as usize].edges)
	}
	fn edge_faces(&self,edge_id:SubmeshEdgeId)->Cow<[SubmeshFaceId;2]>{
		Cow::Borrowed(&self.topology.edge_topology[edge_id.get() as usize].faces)
	}
	fn edge_verts(&self,edge_id:SubmeshEdgeId)->Cow<[SubmeshVertId;2]>{
		Cow::Borrowed(&self.topology.edge_topology[edge_id.get() as usize].verts)
	}
	fn vert_edges(&self,vert_id:SubmeshVertId)->Cow<Vec<SubmeshDirectedEdgeId>>{
		Cow::Borrowed(&self.topology.vert_topology[vert_id.get() as usize].edges)
	}
	fn vert_faces(&self,vert_id:SubmeshVertId)->Cow<Vec<SubmeshFaceId>>{
		Cow::Borrowed(&self.topology.vert_topology[vert_id.get() as usize].faces)
	}
}

pub struct PhysicsMeshTransform{
	pub vertex:integer::Planar64Affine3,
	pub normal:integer::Planar64Mat3,
	pub det:Planar64,
}
impl PhysicsMeshTransform{
	pub const fn new(transform:integer::Planar64Affine3)->Self{
		Self{
			normal:transform.matrix3.inverse_times_det().transpose(),
			det:transform.matrix3.determinant(),
			vertex:transform,
		}
	}
}

pub struct TransformedMesh<'a>{
	view:PhysicsMeshView<'a>,
	transform:&'a PhysicsMeshTransform,
}
impl TransformedMesh<'_>{
	pub fn new<'a>(
		view:PhysicsMeshView<'a>,
		transform:&'a PhysicsMeshTransform,
	)->TransformedMesh<'a>{
		TransformedMesh{
			view,
			transform,
		}
	}
	pub fn verts<'a>(&'a self)->impl Iterator<Item=Planar64Vec3>+'a{
		self.view.data.verts.iter().map(|&Vert(pos)|self.transform.vertex.transform_point3(pos))
	}
	fn farthest_vert(&self,dir:Planar64Vec3)->SubmeshVertId{
		let mut best_dot=Planar64::MIN;
		let mut best_vert=SubmeshVertId(0);
		//this happens to be well-defined.  there are no virtual virtices
		for (i,vert_id) in self.view.topology.verts.iter().enumerate(){
			let p=self.transform.vertex.transform_point3(self.view.data.verts[vert_id.get() as usize].0);
			let d=dir.dot(p);
			if best_dot<d{
				best_dot=d;
				best_vert=SubmeshVertId::new(i as u32);
			}
		}
		best_vert
	}
}
impl MeshQuery<SubmeshFaceId,SubmeshDirectedEdgeId,SubmeshVertId> for TransformedMesh<'_>{
	fn face_nd(&self,face_id:SubmeshFaceId)->(Planar64Vec3,Planar64){
		let (n,d)=self.view.face_nd(face_id);
		let transformed_n=self.transform.normal*n;
		let transformed_d=d+transformed_n.dot(self.transform.vertex.translation)/self.transform.det;
		(transformed_n/self.transform.det,transformed_d)
	}
	fn vert(&self,vert_id:SubmeshVertId)->Planar64Vec3{
		self.transform.vertex.transform_point3(self.view.vert(vert_id))
	}
	#[inline]
	fn face_edges(&self,face_id:SubmeshFaceId)->Cow<Vec<SubmeshDirectedEdgeId>>{
		self.view.face_edges(face_id)
	}
	#[inline]
	fn edge_faces(&self,edge_id:SubmeshEdgeId)->Cow<[SubmeshFaceId;2]>{
		self.view.edge_faces(edge_id)
	}
	#[inline]
	fn edge_verts(&self,edge_id:SubmeshEdgeId)->Cow<[SubmeshVertId;2]>{
		self.view.edge_verts(edge_id)
	}
	#[inline]
	fn vert_edges(&self,vert_id:SubmeshVertId)->Cow<Vec<SubmeshDirectedEdgeId>>{
		self.view.vert_edges(vert_id)
	}
	#[inline]
	fn vert_faces(&self,vert_id:SubmeshVertId)->Cow<Vec<SubmeshFaceId>>{
		self.view.vert_faces(vert_id)
	}
}

//Note that a face on a minkowski mesh refers to a pair of fevs on the meshes it's summed from
//(face,vertex)
//(edge,edge)
//(vertex,face)
#[derive(Clone,Copy)]
pub enum MinkowskiVert{
	VertVert(SubmeshVertId,SubmeshVertId),
}
#[derive(Clone,Copy)]
pub enum MinkowskiEdge{
	VertEdge(SubmeshVertId,SubmeshEdgeId),
	EdgeVert(SubmeshEdgeId,SubmeshVertId),
	//EdgeEdge when edges are parallel
}
impl UndirectedEdge for MinkowskiEdge{
	type DirectedEdge=MinkowskiDirectedEdge;
	fn as_directed(&self,parity:bool)->Self::DirectedEdge{
		match self{
			MinkowskiEdge::VertEdge(v0,e1)=>MinkowskiDirectedEdge::VertEdge(*v0,e1.as_directed(parity)),
			MinkowskiEdge::EdgeVert(e0,v1)=>MinkowskiDirectedEdge::EdgeVert(e0.as_directed(parity),*v1),
		}
	}
}
#[derive(Clone,Copy)]
pub enum MinkowskiDirectedEdge{
	VertEdge(SubmeshVertId,SubmeshDirectedEdgeId),
	EdgeVert(SubmeshDirectedEdgeId,SubmeshVertId),
	//EdgeEdge when edges are parallel
}
impl DirectedEdge for MinkowskiDirectedEdge{
	type UndirectedEdge=MinkowskiEdge;
	fn as_undirected(&self)->Self::UndirectedEdge{
		match self{
			MinkowskiDirectedEdge::VertEdge(v0,e1)=>MinkowskiEdge::VertEdge(*v0,e1.as_undirected()),
			MinkowskiDirectedEdge::EdgeVert(e0,v1)=>MinkowskiEdge::EdgeVert(e0.as_undirected(),*v1),
		}
	}
	fn parity(&self)->bool{
		match self{
			MinkowskiDirectedEdge::VertEdge(_,e)
			|MinkowskiDirectedEdge::EdgeVert(e,_)=>e.parity(),
		}
	}
}
#[derive(Debug,Clone,Copy,Hash,Eq,PartialEq)]
pub enum MinkowskiFace{
	VertFace(SubmeshVertId,SubmeshFaceId),
	EdgeEdge(SubmeshEdgeId,SubmeshEdgeId,bool),
	FaceVert(SubmeshFaceId,SubmeshVertId),
	//EdgeFace
	//FaceEdge
	//FaceFace
}

pub struct MinkowskiMesh<'a>{
	mesh0:TransformedMesh<'a>,
	mesh1:TransformedMesh<'a>,
}

//infinity fev algorithm state transition
enum Transition{
	Done,//found closest vert, no edges are better
	Vert(MinkowskiVert),//transition to vert
}
enum EV{
	Vert(MinkowskiVert),
	Edge(MinkowskiEdge),
}

impl MinkowskiMesh<'_>{
	pub fn minkowski_sum<'a>(mesh0:TransformedMesh<'a>,mesh1:TransformedMesh<'a>)->MinkowskiMesh<'a>{
		MinkowskiMesh{
			mesh0,
			mesh1,
		}
	}
	fn farthest_vert(&self,dir:Planar64Vec3)->MinkowskiVert{
		MinkowskiVert::VertVert(self.mesh0.farthest_vert(dir),self.mesh1.farthest_vert(-dir))
	}
	fn next_transition_vert(&self,vert_id:MinkowskiVert,best_distance_squared:&mut Planar64,infinity_dir:Planar64Vec3,point:Planar64Vec3)->Transition{
		let mut best_transition=Transition::Done;
		for &directed_edge_id in self.vert_edges(vert_id).iter(){
			let edge_n=self.directed_edge_n(directed_edge_id);
			//is boundary uncrossable by a crawl from infinity
			let edge_verts=self.edge_verts(directed_edge_id.as_undirected());
			//select opposite vertex
			let test_vert_id=edge_verts[directed_edge_id.parity() as usize];
			//test if it's closer
			let diff=point-self.vert(test_vert_id);
			if zeroes::zeroes1(edge_n.dot(diff),edge_n.dot(infinity_dir)).len()==0{
				let distance_squared=diff.dot(diff);
				if distance_squared<*best_distance_squared{
					best_transition=Transition::Vert(test_vert_id);
					*best_distance_squared=distance_squared;
				}
			}
		}
		best_transition
	}
	fn final_ev(&self,vert_id:MinkowskiVert,best_distance_squared:&mut Planar64,infinity_dir:Planar64Vec3,point:Planar64Vec3)->EV{
		let mut best_transition=EV::Vert(vert_id);
		let diff=point-self.vert(vert_id);
		for &directed_edge_id in self.vert_edges(vert_id).iter(){
			let edge_n=self.directed_edge_n(directed_edge_id);
			//is boundary uncrossable by a crawl from infinity
			//check if time of collision is outside Time::MIN..Time::MAX
			let d=edge_n.dot(diff);
			if zeroes::zeroes1(d,edge_n.dot(infinity_dir)).len()==0{
				//test the edge
				let edge_nn=edge_n.dot(edge_n);
				if Planar64::ZERO<=d&&d<=edge_nn{
					let distance_squared={
						let c=diff.cross(edge_n);
						c.dot(c)/edge_nn
					};
					if distance_squared<=*best_distance_squared{
						best_transition=EV::Edge(directed_edge_id.as_undirected());
						*best_distance_squared=distance_squared;
					}
				}
			}
		}
		best_transition
	}
	fn crawl_boundaries(&self,mut vert_id:MinkowskiVert,infinity_dir:Planar64Vec3,point:Planar64Vec3)->EV{
		let mut best_distance_squared={
			let diff=point-self.vert(vert_id);
			diff.dot(diff)
		};
		loop{
			match self.next_transition_vert(vert_id,&mut best_distance_squared,infinity_dir,point){
				Transition::Done=>return self.final_ev(vert_id,&mut best_distance_squared,infinity_dir,point),
				Transition::Vert(new_vert_id)=>vert_id=new_vert_id,
			}
		}
	}
	/// This function drops a vertex down to an edge or a face if the path from infinity did not cross any vertex-edge boundaries but the point is supposed to have already crossed a boundary down from a vertex
	fn infinity_fev(&self,infinity_dir:Planar64Vec3,point:Planar64Vec3)->FEV::<MinkowskiFace,MinkowskiDirectedEdge,MinkowskiVert>{
		//start on any vertex
		//cross uncrossable vertex-edge boundaries until you find the closest vertex or edge
		//cross edge-face boundary if it's uncrossable
		match self.crawl_boundaries(self.farthest_vert(infinity_dir),infinity_dir,point){
			//if a vert is returned, it is the closest point to the infinity point
			EV::Vert(vert_id)=>FEV::<MinkowskiFace,MinkowskiDirectedEdge,MinkowskiVert>::Vert(vert_id),
			EV::Edge(edge_id)=>{
				//cross to face if the boundary is not crossable and we are on the wrong side
				let edge_n=self.edge_n(edge_id);
				// point is multiplied by two because vert_sum sums two vertices.
				let delta_pos=point*2-{
					let &[v0,v1]=self.edge_verts(edge_id).borrow();
					self.vert(v0)+self.vert(v1)
				};
				for (i,&face_id) in self.edge_faces(edge_id).iter().enumerate(){
					let face_n=self.face_nd(face_id).0;
					//edge-face boundary nd, n facing out of the face towards the edge
					let boundary_n=face_n.cross(edge_n)*(i as i64*2-1);
					let boundary_d=boundary_n.dot(delta_pos);
					//check if time of collision is outside Time::MIN..Time::MAX
					//infinity_dir can always be treated as a velocity
					if (boundary_d)<=Planar64::ZERO&&zeroes::zeroes1(boundary_d,boundary_n.dot(infinity_dir)*2).len()==0{
						//both faces cannot pass this condition, return early if one does.
						return FEV::<MinkowskiFace,MinkowskiDirectedEdge,MinkowskiVert>::Face(face_id);
					}
				}
				FEV::<MinkowskiFace,MinkowskiDirectedEdge,MinkowskiVert>::Edge(edge_id)
			},
		}
	}
	fn closest_fev_not_inside(&self,mut infinity_body:crate::physics::Body)->Option<FEV::<MinkowskiFace,MinkowskiDirectedEdge,MinkowskiVert>>{
		infinity_body.infinity_dir().map_or(None,|dir|{
			let infinity_fev=self.infinity_fev(-dir,infinity_body.position);
			//a line is simpler to solve than a parabola
			infinity_body.velocity=dir;
			infinity_body.acceleration=Planar64Vec3::ZERO;
			//crawl in from negative infinity along a tangent line to get the closest fev
			match crate::face_crawler::crawl_fev(infinity_fev,self,&infinity_body,integer::Time::MIN,infinity_body.time){
				crate::face_crawler::CrawlResult::Miss(fev)=>Some(fev),
				crate::face_crawler::CrawlResult::Hit(_,_)=>None,
			}
		})
	}
	pub fn predict_collision_in(&self,relative_body:&crate::physics::Body,time_limit:integer::Time)->Option<(MinkowskiFace,integer::Time)>{
		self.closest_fev_not_inside(relative_body.clone()).map_or(None,|fev|{
			//continue forwards along the body parabola
			match crate::face_crawler::crawl_fev(fev,self,relative_body,relative_body.time,time_limit){
				crate::face_crawler::CrawlResult::Miss(_)=>None,
				crate::face_crawler::CrawlResult::Hit(face,time)=>Some((face,time)),
			}
		})
	}
	pub fn predict_collision_out(&self,relative_body:&crate::physics::Body,time_limit:integer::Time)->Option<(MinkowskiFace,integer::Time)>{
		//create an extrapolated body at time_limit
		let infinity_body=crate::physics::Body::new(
			relative_body.extrapolated_position(time_limit),
			-relative_body.extrapolated_velocity(time_limit),
			relative_body.acceleration,
			-time_limit,
		);
		self.closest_fev_not_inside(infinity_body).map_or(None,|fev|{
			//continue backwards along the body parabola
			match crate::face_crawler::crawl_fev(fev,self,&-relative_body.clone(),-time_limit,-relative_body.time){
				crate::face_crawler::CrawlResult::Miss(_)=>None,
				crate::face_crawler::CrawlResult::Hit(face,time)=>Some((face,-time)),//no need to test -time<time_limit because of the first step
			}
		})
	}
	pub fn predict_collision_face_out(&self,relative_body:&crate::physics::Body,time_limit:integer::Time,contact_face_id:MinkowskiFace)->Option<(MinkowskiEdge,integer::Time)>{
		//no algorithm needed, there is only one state and two cases (Edge,None)
		//determine when it passes an edge ("sliding off" case)
		let mut best_time=time_limit;
		let mut best_edge=None;
		let face_n=self.face_nd(contact_face_id).0;
		for &directed_edge_id in self.face_edges(contact_face_id).iter(){
			let edge_n=self.directed_edge_n(directed_edge_id);
			//f x e points in
			let n=face_n.cross(edge_n);
			let verts=self.edge_verts(directed_edge_id.as_undirected());
			let d=n.dot(self.vert(verts[0])+self.vert(verts[1]));
			//WARNING! d outside of *2
			for t in zeroes::zeroes2((n.dot(relative_body.position))*2-d,n.dot(relative_body.velocity)*2,n.dot(relative_body.acceleration)){
				let t=relative_body.time+integer::Time::from(t);
				if relative_body.time<t&&t<best_time&&n.dot(relative_body.extrapolated_velocity(t))<Planar64::ZERO{
					best_time=t;
					best_edge=Some(directed_edge_id);
					break;
				}
			}
		}
		best_edge.map(|e|(e.as_undirected(),best_time))
	}
}
impl MeshQuery<MinkowskiFace,MinkowskiDirectedEdge,MinkowskiVert> for MinkowskiMesh<'_>{
	fn face_nd(&self,face_id:MinkowskiFace)->(Planar64Vec3,Planar64){
		match face_id{
			MinkowskiFace::VertFace(v0,f1)=>{
				let (n,d)=self.mesh1.face_nd(f1);
				(-n,d-n.dot(self.mesh0.vert(v0)))
			},
			MinkowskiFace::EdgeEdge(e0,e1,parity)=>{
				let edge0_n=self.mesh0.edge_n(e0);
				let edge1_n=self.mesh1.edge_n(e1);
				let &[e0v0,e0v1]=self.mesh0.edge_verts(e0).borrow();
				let &[e1v0,e1v1]=self.mesh1.edge_verts(e1).borrow();
				let n=edge0_n.cross(edge1_n);
				let e0d=n.dot(self.mesh0.vert(e0v0)+self.mesh0.vert(e0v1));
				let e1d=n.dot(self.mesh1.vert(e1v0)+self.mesh1.vert(e1v1));
				(n*(parity as i64*4-2),(e0d-e1d)*(parity as i64*2-1))
			},
			MinkowskiFace::FaceVert(f0,v1)=>{
				let (n,d)=self.mesh0.face_nd(f0);
				(n,d-n.dot(self.mesh1.vert(v1)))
			},
		}
	}
	fn vert(&self,vert_id:MinkowskiVert)->Planar64Vec3{
		match vert_id{
			MinkowskiVert::VertVert(v0,v1)=>{
				self.mesh0.vert(v0)-self.mesh1.vert(v1)
			},
		}
	}
	fn face_edges(&self,face_id:MinkowskiFace)->Cow<Vec<MinkowskiDirectedEdge>>{
		match face_id{
			MinkowskiFace::VertFace(v0,f1)=>{
				Cow::Owned(self.mesh1.face_edges(f1).iter().map(|&edge_id1|{
					MinkowskiDirectedEdge::VertEdge(v0,edge_id1.reverse())
				}).collect())
			},
			MinkowskiFace::EdgeEdge(e0,e1,parity)=>{
				let e0v=self.mesh0.edge_verts(e0);
				let e1v=self.mesh1.edge_verts(e1);
				//could sort this if ordered edges are needed
				//probably just need to reverse this list according to parity
				Cow::Owned(vec![
					MinkowskiDirectedEdge::VertEdge(e0v[0],e1.as_directed(parity)),
					MinkowskiDirectedEdge::EdgeVert(e0.as_directed(!parity),e1v[0]),
					MinkowskiDirectedEdge::VertEdge(e0v[1],e1.as_directed(!parity)),
					MinkowskiDirectedEdge::EdgeVert(e0.as_directed(parity),e1v[1]),
				])
			},
			MinkowskiFace::FaceVert(f0,v1)=>{
				Cow::Owned(self.mesh0.face_edges(f0).iter().map(|&edge_id0|{
					MinkowskiDirectedEdge::EdgeVert(edge_id0,v1)
				}).collect())
			},
		}
	}
	fn edge_faces(&self,edge_id:MinkowskiEdge)->Cow<[MinkowskiFace;2]>{
		match edge_id{
			MinkowskiEdge::VertEdge(v0,e1)=>{
				//faces are listed backwards from the minkowski mesh
				let v0e=self.mesh0.vert_edges(v0);
				let &[e1f0,e1f1]=self.mesh1.edge_faces(e1).borrow();
				Cow::Owned([(e1f1,false),(e1f0,true)].map(|(edge_face_id1,face_parity)|{
					let mut best_edge=None;
					let mut best_d=Planar64::ZERO;
					let edge_face1_n=self.mesh1.face_nd(edge_face_id1).0;
					let edge_face1_nn=edge_face1_n.dot(edge_face1_n);
					for &directed_edge_id0 in v0e.iter(){
						let edge0_n=self.mesh0.directed_edge_n(directed_edge_id0);
						//must be behind other face.
						let d=edge_face1_n.dot(edge0_n);
						if d<Planar64::ZERO{
							let edge0_nn=edge0_n.dot(edge0_n);
							//divide by zero???
							let dd=d*d/(edge_face1_nn*edge0_nn);
							if best_d<dd{
								best_d=dd;
								best_edge=Some(directed_edge_id0);
							}
						}
					}
					best_edge.map_or(
						MinkowskiFace::VertFace(v0,edge_face_id1),
						|directed_edge_id0|MinkowskiFace::EdgeEdge(directed_edge_id0.as_undirected(),e1,directed_edge_id0.parity()^face_parity)
					)
				}))
			},
			MinkowskiEdge::EdgeVert(e0,v1)=>{
				//tracking index with an external variable because .enumerate() is not available
				let v1e=self.mesh1.vert_edges(v1);
				let &[e0f0,e0f1]=self.mesh0.edge_faces(e0).borrow();
				Cow::Owned([(e0f0,true),(e0f1,false)].map(|(edge_face_id0,face_parity)|{
					let mut best_edge=None;
					let mut best_d=Planar64::ZERO;
					let edge_face0_n=self.mesh0.face_nd(edge_face_id0).0;
					let edge_face0_nn=edge_face0_n.dot(edge_face0_n);
					for &directed_edge_id1 in v1e.iter(){
						let edge1_n=self.mesh1.directed_edge_n(directed_edge_id1);
						let d=edge_face0_n.dot(edge1_n);
						if d<Planar64::ZERO{
							let edge1_nn=edge1_n.dot(edge1_n);
							let dd=d*d/(edge_face0_nn*edge1_nn);
							if best_d<dd{
								best_d=dd;
								best_edge=Some(directed_edge_id1);
							}
						}
					}
					best_edge.map_or(
						MinkowskiFace::FaceVert(edge_face_id0,v1),
						|directed_edge_id1|MinkowskiFace::EdgeEdge(e0,directed_edge_id1.as_undirected(),directed_edge_id1.parity()^face_parity)
					)
				}))
			},
		}
	}
	fn edge_verts(&self,edge_id:MinkowskiEdge)->Cow<[MinkowskiVert;2]>{
		match edge_id{
			MinkowskiEdge::VertEdge(v0,e1)=>{
				Cow::Owned(self.mesh1.edge_verts(e1).map(|vert_id1|{
					MinkowskiVert::VertVert(v0,vert_id1)
				}))
			},
			MinkowskiEdge::EdgeVert(e0,v1)=>{
				Cow::Owned(self.mesh0.edge_verts(e0).map(|vert_id0|{
					MinkowskiVert::VertVert(vert_id0,v1)
				}))
			},
		}
	}
	fn vert_edges(&self,vert_id:MinkowskiVert)->Cow<Vec<MinkowskiDirectedEdge>>{
		match vert_id{
			MinkowskiVert::VertVert(v0,v1)=>{
				let mut edges=Vec::new();
				//detect shared volume when the other mesh is mirrored along a test edge dir
				let v0f=self.mesh0.vert_faces(v0);
				let v1f=self.mesh1.vert_faces(v1);
				let v0f_n:Vec<Planar64Vec3>=v0f.iter().map(|&face_id|self.mesh0.face_nd(face_id).0).collect();
				let v1f_n:Vec<Planar64Vec3>=v1f.iter().map(|&face_id|self.mesh1.face_nd(face_id).0).collect();
				let the_len=v0f.len()+v1f.len();
				for &directed_edge_id in self.mesh0.vert_edges(v0).iter(){
					let n=self.mesh0.directed_edge_n(directed_edge_id);
					let nn=n.dot(n);
					//make a set of faces
					let mut face_normals=Vec::with_capacity(the_len);
					//add mesh0 faces as-is
					face_normals.clone_from(&v0f_n);
					for face_n in &v1f_n{
						//add reflected mesh1 faces
						face_normals.push(*face_n-n*(face_n.dot(n)*2/nn));
					}
					if is_empty_volume(face_normals){
						edges.push(MinkowskiDirectedEdge::EdgeVert(directed_edge_id,v1));
					}
				}
				for &directed_edge_id in self.mesh1.vert_edges(v1).iter(){
					let n=self.mesh1.directed_edge_n(directed_edge_id);
					let nn=n.dot(n);
					let mut face_normals=Vec::with_capacity(the_len);
					face_normals.clone_from(&v1f_n);
					for face_n in &v0f_n{
						face_normals.push(*face_n-n*(face_n.dot(n)*2/nn));
					}
					if is_empty_volume(face_normals){
						edges.push(MinkowskiDirectedEdge::VertEdge(v0,directed_edge_id));
					}
				}
				Cow::Owned(edges)
			},
		}
	}
	fn vert_faces(&self,_vert_id:MinkowskiVert)->Cow<Vec<MinkowskiFace>>{
		unimplemented!()
	}
}

fn is_empty_volume(normals:Vec<Planar64Vec3>)->bool{
	let len=normals.len();
	for i in 0..len-1{
		for j in i+1..len{
			let n=normals[i].cross(normals[j]);
			let mut d_comp=None;
			for k in 0..len{
				if k!=i&&k!=j{
					let d=n.dot(normals[k]);
					if let Some(comp)=&d_comp{
						if *comp*d<Planar64::ZERO{
							return true;
						}
					}else{
						d_comp=Some(d);
					}
				}
			}
		}
	}
	return false;
}

#[test]
fn test_is_empty_volume(){
	assert!(!is_empty_volume([Planar64Vec3::X,Planar64Vec3::Y,Planar64Vec3::Z].to_vec()));
	assert!(is_empty_volume([Planar64Vec3::X,Planar64Vec3::Y,Planar64Vec3::Z,Planar64Vec3::NEG_X].to_vec()));
}

#[test]
fn build_me_a_cube(){
	let mesh=PhysicsMesh::unit_cube();
	//println!("mesh={:?}",mesh);
}
use crate::aabb::Aabb;

//da algaritum
//lista boxens
//sort by {minx,maxx,miny,maxy,minz,maxz} (6 lists)
//find the sets that minimizes the sum of surface areas
//splitting is done when the minimum split sum of surface areas is larger than the node's own surface area

//start with bisection into octrees because a bad bvh is still 1000x better than no bvh
//sort the centerpoints on each axis (3 lists)
//bv is put into octant based on whether it is upper or lower in each list

pub struct BvhNode{
	children:Vec<Self>,
	models:Vec<u32>,
	aabb:Aabb,
}

pub fn generate_bvh(boxen:Vec<Aabb>)->BvhNode{
	generate_bvh_node(boxen.into_iter().enumerate().collect())
}

fn generate_bvh_node(boxen:Vec<(usize,Aabb)>)->BvhNode{
	let n=boxen.len();
	if n<20{
		let mut aabb=Aabb::new();
		let models=boxen.into_iter().map(|b|{aabb.join(&b.1);b.0 as u32}).collect();
		BvhNode{
			children:Vec::new(),
			models,
			aabb,
		}
	}else{
		let mut octant=std::collections::HashMap::with_capacity(n);//this ids which octant the boxen is put in
		let mut sort_x=Vec::with_capacity(n);
		let mut sort_y=Vec::with_capacity(n);
		let mut sort_z=Vec::with_capacity(n);
		for (i,aabb) in boxen.iter(){
			let center=aabb.center();
			octant.insert(*i,0);
			sort_x.push((*i,center.x));
			sort_y.push((*i,center.y));
			sort_z.push((*i,center.z));
		}
		sort_x.sort_by(|tup0,tup1|tup0.1.partial_cmp(&tup1.1).unwrap());
		sort_y.sort_by(|tup0,tup1|tup0.1.partial_cmp(&tup1.1).unwrap());
		sort_z.sort_by(|tup0,tup1|tup0.1.partial_cmp(&tup1.1).unwrap());
		let h=n/2;
		let median_x=sort_x[h].1;
		let median_y=sort_y[h].1;
		let median_z=sort_z[h].1;
		for (i,c) in sort_x{
			if median_x<c{
				octant.insert(i,octant[&i]+1<<0);
			}
		}
		for (i,c) in sort_y{
			if median_y<c{
				octant.insert(i,octant[&i]+1<<1);
			}
		}
		for (i,c) in sort_z{
			if median_z<c{
				octant.insert(i,octant[&i]+1<<2);
			}
		}
		//generate lists for unique octant values
		let mut list_list=Vec::with_capacity(8);
		let mut octant_list=Vec::with_capacity(8);
		for (i,aabb) in boxen.into_iter(){
			let octant_id=octant[&i];
			let list_id=if let Some(list_id)=octant_list.iter().position(|&id|id==octant_id){
				list_id
			}else{
				let list_id=list_list.len();
				octant_list.push(octant_id);
				list_list.push(Vec::new());
				list_id
			};
			list_list[list_id].push((i,aabb));
		}
		let mut aabb=Aabb::new();
		let children=list_list.into_iter().map(|b|{
			let node=generate_bvh_node(b);
			aabb.join(&node.aabb);
			node
		}).collect();
		BvhNode{
			children,
			models:Vec::new(),
			aabb,
		}
	}
}

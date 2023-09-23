fn class_is_a(class: &str, superclass: &str) -> bool {
	if class==superclass {
		return true
	}
	let class_descriptor=rbx_reflection_database::get().classes.get(class);
	if let Some(descriptor) = &class_descriptor {
		if let Some(class_super) = &descriptor.superclass {
			return class_is_a(&class_super, superclass)
		}
	}
	return false
}
fn recursive_collect_superclass(objects: &mut std::vec::Vec<rbx_dom_weak::types::Ref>,dom: &rbx_dom_weak::WeakDom, instance: &rbx_dom_weak::Instance, superclass: &str){
	for &referent in instance.children() {
		if let Some(c) = dom.get_by_ref(referent) {
			if class_is_a(c.class.as_str(), superclass) {
				objects.push(c.referent());//copy ref
			}
			recursive_collect_superclass(objects,dom,c,superclass);
		}
	}
}
fn get_texture_refs(dom:&rbx_dom_weak::WeakDom) -> Vec<rbx_dom_weak::types::Ref>{
	let mut objects = std::vec::Vec::new();
	recursive_collect_superclass(&mut objects, dom, dom.root(),"Decal");
	//get ids
	//clear vec
	//next class
	objects
}
pub fn get_objects(dom:rbx_dom_weak::WeakDom, superclass: &str) -> Result<std::vec::Vec<rbx_dom_weak::types::Ref>, Box<dyn std::error::Error>> {
	let mut objects = std::vec::Vec::new();
	recursive_collect_superclass(&mut objects, &dom, dom.root(),superclass);

	return Ok(objects)
}

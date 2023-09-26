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

pub fn get_objects(dom:rbx_dom_weak::WeakDom, superclass: &str) -> Result<std::vec::Vec<rbx_dom_weak::Instance>, Box<dyn std::error::Error>> {
	let mut objects = std::vec::Vec::<rbx_dom_weak::Instance>::new();
	//move matching instances into objects
	let (_,mut instances) = dom.into_raw();
	for (_,instance) in instances.drain() {
		if class_is_a(instance.class.as_str(), superclass) {
			objects.push(instance);
		}
	}

	return Ok(objects)
}

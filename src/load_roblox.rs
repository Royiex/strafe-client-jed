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

fn recursive_collect_objects(objects: &mut std::vec::Vec<rbx_dom_weak::types::Ref>,dom: &rbx_dom_weak::WeakDom, instance: &rbx_dom_weak::Instance, superclass: &str){
    for &referent in instance.children() {
        if let Some(c) = dom.get_by_ref(referent) {
            if class_is_a(c.class.as_str(), superclass) {
                objects.push(c.referent());//copy ref
            }
            recursive_collect_objects(objects,dom,c,superclass);
        }
    }
}

pub fn get_objects(buf_thing: std::io::BufReader<&[u8]>, superclass: &str) -> Result<(rbx_dom_weak::WeakDom,std::vec::Vec<rbx_dom_weak::types::Ref>), Box<dyn std::error::Error>> {
    // Using buffered I/O is recommended with rbx_binary
    let dom = rbx_binary::from_reader(buf_thing)?;

    let mut objects = std::vec::Vec::<rbx_dom_weak::types::Ref>::new();
    recursive_collect_objects(&mut objects, &dom, dom.root(), superclass);

    return Ok((dom,objects))
}

//! Describes a CSG hierarchy

use bevy::{math::Vec3};

struct CSGTree {
    root_node: CSGNode,
}

struct CSGNode
{
    node_type: CSGNodeType,
    left: Option<Box<CSGNode>>,
    right: Option<Box<CSGNode>>,
    local_translation: Vec3,
    translation: Vec3,
    //planes: Vec<Plane>,
}

enum CSGNodeType {
    Addition,
    Subtraction,
    Common,
    Brush,
}

impl CSGNode {
    fn new(branch_operator: CSGNodeType, left: Option<CSGNode>, right: Option<CSGNode>) -> Self {
        Self {
            node_type: branch_operator,
            left: left.map(Box::new),
            right: right.map(Box::new),
            local_translation: Vec3::ZERO,
            translation: Vec3::ZERO,
            //planes: vec![],
        }
    }
}


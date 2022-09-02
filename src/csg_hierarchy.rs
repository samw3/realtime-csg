//! Describes a CSG hierarchy

use crate::csg_geometry::{Aabb, Plane};
use bevy::math::Vec3;

pub struct CsgTree<'a> {
    root_node: CsgNode<'a>,
}

pub struct CsgNode<'a> {
    bounds: Aabb,
    node_type: CsgNodeType,
    left: Option<Box<CsgNode<'a>>>,
    right: Option<Box<CsgNode<'a>>>,
    parent: Option<&'a CsgNode<'a>>,
    local_translation: Vec3,
    translation: Vec3,
    planes: Vec<Plane>,
}

pub enum CsgNodeType {
    Addition,
    Subtraction,
    Common,
    Brush,
}

impl<'a> CsgNode<'a> {
    pub fn new(
        branch_operator: CsgNodeType,
        left: Option<CsgNode<'a>>,
        right: Option<CsgNode<'a>>,
    ) -> Self {
        Self {
            bounds: Default::default(),
            node_type: branch_operator,
            left: left.map(Box::new),
            right: right.map(Box::new),
            parent: None,
            local_translation: Vec3::ZERO,
            translation: Vec3::ZERO,
            planes: vec![],
        }
    }

    pub fn brush(planes: Vec<Plane>) -> Self {
        Self {
            bounds: Default::default(),
            node_type: CsgNodeType::Brush,
            left: None,
            right: None,
            parent: None,
            local_translation: Default::default(),
            translation: Default::default(),
            planes,
        }
    }
}

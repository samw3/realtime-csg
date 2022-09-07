//! Describes a CSG hierarchy

use crate::csg_geometry::{Aabb, Plane};
use bevy::math::Vec3;

pub struct CsgTreeIterator<'a> {
  tree: &'a CsgTree<'a>,
  stack: Vec<&'a CsgNode<'a>>,
}

impl<'a> CsgTreeIterator<'a> {
  fn new(tree: &'a CsgTree<'a>) -> Self {
    Self {
      tree,
      stack: vec![&tree.root_node],
    }
  }
}

/// Iterates over all nodes in the tree
impl<'a> Iterator for CsgTreeIterator<'a> {
  type Item = &'a CsgNode<'a>;

  fn next(&mut self) -> Option<Self::Item> {
    match self.stack.pop() {
      Some(node) => {
        match node.node_type {
          CsgNodeType::Brush => (),
          _ => {
            if let Some(left) = &node.left {
              self.stack.push(left);
            }
            if let Some(right) = &node.right {
              self.stack.push(right);
            }
          }
        }
        Some(node)
      }
      None => None,
    }
  }
}

pub struct CsgTree<'a> {
  root_node: CsgNode<'a>,
}

impl<'a> CsgTree<'a> {
  fn new(root_node: CsgNode<'a>) -> Self {
    Self { root_node }
  }
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

  pub fn add(left: Option<CsgNode<'a>>, right: Option<CsgNode<'a>>) -> Self {
    CsgNode::new(CsgNodeType::Addition, left, right)
  }

  pub fn sub(left: Option<CsgNode<'a>>, right: Option<CsgNode<'a>>) -> Self {
    CsgNode::new(CsgNodeType::Subtraction, left, right)
  }

  pub fn common(left: Option<CsgNode<'a>>, right: Option<CsgNode<'a>>) -> Self {
    CsgNode::new(CsgNodeType::Common, left, right)
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

#[cfg(test)]
mod tests {
  use super::*;
  use std::ptr;

  #[test]
  fn test_tree_iter_root_only() {
    let brush = CsgNode::brush(vec![]);
    let tree = CsgTree::new(brush);
    for node in CsgTreeIterator::new(&tree) {
      assert!(ptr::eq(node, &tree.root_node))
    }
  }

  #[test]
  fn test_tree_iter_3_nodes() {
    let left = Some(CsgNode::brush(vec![]));
    let right = Some(CsgNode::brush(vec![]));
    let add = CsgNode::add(left, right);
    let tree = CsgTree::new(add);
    {
      let iter = CsgTreeIterator::new(&tree);
      assert_eq!(3, iter.count());
    }
    {
      let mut iter = CsgTreeIterator::new(&tree);
      assert!(matches!(
        iter.next().unwrap().node_type,
        CsgNodeType::Addition
      ));
      assert!(matches!(iter.next().unwrap().node_type, CsgNodeType::Brush));
      assert!(matches!(iter.next().unwrap().node_type, CsgNodeType::Brush));
      assert!(matches!(iter.next(), None));
    }
  }

  #[test]
  fn test_tree_iter_5_nodes() {
    let left = Some(CsgNode::brush(vec![]));
    let right = Some(CsgNode::brush(vec![]));
    let sub = Some(CsgNode::sub(left, right));
    let right2 = Some(CsgNode::brush(vec![]));
    let add = CsgNode::add(sub, right2);
    let tree = CsgTree::new(add);
    {
      let iter = CsgTreeIterator::new(&tree);
      assert_eq!(5, iter.count());
    }
    {
      let mut iter = CsgTreeIterator::new(&tree);
      assert!(matches!(
        iter.next().unwrap().node_type,
        CsgNodeType::Addition
      ));
      assert!(matches!(iter.next().unwrap().node_type, CsgNodeType::Brush));
      assert!(matches!(
        iter.next().unwrap().node_type,
        CsgNodeType::Subtraction
      ));
      assert!(matches!(iter.next().unwrap().node_type, CsgNodeType::Brush));
      assert!(matches!(iter.next().unwrap().node_type, CsgNodeType::Brush));
      assert!(matches!(iter.next(), None));
    }
  }
}

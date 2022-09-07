use crate::csg_hierarchy::CsgNodeType;

struct CsgFile {
  tree: Node,
  instances: Vec<Node>,
}

enum Node {
  Branch(Branch),
  Brush,
  Instance,
}

type NodeRef = Option<Box<Node>>;

struct Branch {
  operator: CsgNodeType,
  left: NodeRef,
  right: NodeRef,
}

impl Branch {
  pub fn new(branch_operator: CsgNodeType) -> Self {
    Self {
      operator: branch_operator,
      left: None,
      right: None,
    }
  }
}

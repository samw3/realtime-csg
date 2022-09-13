//! Geometry

use bevy::math::{IVec3, Vec3};
use bevy::utils::default;
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use std::default;
use std::hash::{Hash, Hasher};
use std::ops::Sub;

pub const DISTANCE_EPSILON: f32 = 0.0001;
pub const NORMAL_EPSILON: f64 = 1.0 / 65535.0;

pub enum PlaneSideResult {
  Outside,
  Inside,
  Intersects,
}

#[derive(Clone, Deserialize, Serialize)]
pub struct Aabb {
  min: IVec3,
  max: IVec3,
}

impl Default for Aabb {
  fn default() -> Self {
    Aabb::new()
  }
}

impl Aabb {
  pub fn origin(&self) -> IVec3 {
    (self.max + self.min) / 2
  }
  pub fn extent(&self) -> IVec3 {
    self.max - self.min
  }

  pub fn new() -> Self {
    Self {
      min: IVec3::splat(i32::MAX),
      max: IVec3::splat(i32::MIN),
    }
  }

  pub fn is_empty(&self) -> bool {
    self.min.x >= self.max.x || self.min.y >= self.max.y || self.min.z >= self.max.z
  }

  pub fn clear(&mut self) {
    self.min = IVec3::splat(i32::MAX);
    self.max = IVec3::splat(i32::MIN);
  }

  pub fn add_vec3(&mut self, inp: Vec3) {
    if !inp.is_finite() || inp.is_nan() {
      panic!("Bad input Vec3");
    }
    self.min = self.min.min(inp.floor().as_ivec3());
    self.max = self.max.max(inp.ceil().as_ivec3());
  }

  pub fn add(&mut self, in_x: f32, in_y: f32, in_z: f32) {
    self.add_vec3(Vec3 {
      x: in_x,
      y: in_y,
      z: in_z,
    });
  }

  pub fn add_aabb(&mut self, inp: &Aabb) {
    self.min = self.min.min(inp.min);
    self.max = self.max.max(inp.max);
  }

  pub fn set(&mut self, bounds: &Aabb) {
    self.min = bounds.min;
    self.max = bounds.max;
  }

  pub fn translate(&mut self, by: Vec3) {
    self.min += by.floor().as_ivec3();
  }

  pub fn translated(&mut self, by: Vec3) -> Aabb {
    Aabb {
      min: self.min + by.as_ivec3(),
      max: self.max + by.as_ivec3(),
    }
  }

  pub fn set_translate(&mut self, by: &Aabb, translation: Vec3) {
    self.min = by.min + translation.as_ivec3();
    self.max = by.max + translation.as_ivec3();
  }

  pub fn is_outside(&self, other: &Aabb) -> bool {
    Aabb::is_outside_each_other(self, other)
  }

  pub fn is_outside_each_other(left: &Aabb, right: &Aabb) -> bool {
    (left.max.x - right.min.x) < 0
      || (left.min.x - right.max.x) > 0
      || (left.max.y - right.min.y) < 0
      || (left.min.y - right.max.y) > 0
      || (left.max.z - right.min.z) < 0
      || (left.min.z - right.max.z) > 0
  }

  pub fn is_outside_translate(left: &Aabb, translation: Vec3, right: &Aabb) -> bool {
    let translation = translation.as_ivec3();
    ((left.max.x + translation.x) - right.min.x) < 0
      || ((left.min.x + translation.x) - right.max.x) > 0
      || ((left.max.y + translation.y) - right.min.y) < 0
      || ((left.min.y + translation.y) - right.max.y) > 0
      || ((left.max.z + translation.z) - right.min.z) < 0
      || ((left.min.z + translation.z) - right.max.z) > 0
  }
}

#[derive(Clone)]
pub struct HalfEdge {
  next_index: i16,
  twin_index: i16,
  vertex_index: i16,
  polygon_index: i16,
}

#[derive(Debug, Clone, Copy)]
pub struct Plane {
  a: f32,
  b: f32,
  c: f32,
  d: f32,
}

impl Hash for Plane {
  fn hash<H: Hasher>(&self, state: &mut H) {
    self.a.to_bits().hash(state);
    self.b.to_bits().hash(state);
    self.c.to_bits().hash(state);
    self.d.to_bits().hash(state);
  }
}

impl PartialEq for Plane {
  fn eq(&self, other: &Self) -> bool {
    if self == other {
      return true;
    }
    self.d == other.d && self.a == other.a && self.b == other.b && self.c == other.c
  }
}

impl Plane {
  pub fn get_normal(&self) -> Vec3 {
    Vec3 {
      x: self.a,
      y: self.b,
      z: self.c,
    }
  }
  pub fn set_normal(&mut self, normal: Vec3) {
    self.a = normal.x;
    self.b = normal.y;
    self.c = normal.z;
  }

  pub fn point_on_plane(&self) -> Vec3 {
    self.get_normal() * self.d
  }
  pub fn from_normal(normal: Vec3, d: f32) -> Self {
    Self {
      a: normal.x,
      b: normal.y,
      c: normal.z,
      d,
    }
  }
  pub fn intersection_with_ray(&self, start: Vec3, end: Vec3) -> Vec3 {
    intersection_with_ray(start, end, self.distance(start), self.distance(end))
  }
  pub fn distance(&self, vertex: Vec3) -> f32 {
    self.a * vertex.x + self.b * vertex.y + self.c * vertex.z - self.d
  }
  pub fn distance_xyz(&self, x: f32, y: f32, z: f32) -> f32 {
    self.a * x + self.b * y + self.c * z - self.d
  }

  pub fn on_side_xyz(&self, x: f32, y: f32, z: f32) -> PlaneSideResult {
    on_side(self.distance_xyz(x, y, z))
  }

  pub fn on_side_vec3(&self, vertex: Vec3) -> PlaneSideResult {
    on_side(self.distance(vertex))
  }

  pub fn on_side(&self, bounds: Aabb) -> PlaneSideResult {
    let x = if self.a >= 0.0 { bounds.min.x } else { bounds.max.x } as f32;
    let y = if self.b >= 0.0 { bounds.min.y } else { bounds.max.y } as f32;
    let z = if self.c >= 0.0 { bounds.min.z } else { bounds.max.z } as f32;
    on_side(self.distance_xyz(x, y, z))
  }

  pub fn on_side_translate(&self, bounds: Aabb, translation: Vec3) -> PlaneSideResult {
    let backward_x = if self.a >= 0.0 { bounds.min.x } else { bounds.max.x } as f32;
    let backward_y = if self.b >= 0.0 { bounds.min.y } else { bounds.max.y } as f32;
    let backward_z = if self.c >= 0.0 { bounds.min.z } else { bounds.max.z } as f32;
    let distance = self.distance_xyz(
      backward_x + translation.x,
      backward_y + translation.y,
      backward_z + translation.z,
    );
    match on_side(distance) {
      PlaneSideResult::Outside => PlaneSideResult::Outside,
      _ => PlaneSideResult::Intersects,
    }
  }

  pub fn negated(&self) -> Plane {
    Plane {
      a: -self.a,
      b: -self.b,
      c: -self.c,
      d: -self.d,
    }
  }

  pub fn translate(&mut self, translation: Vec3) {
    self.d += (self.a * translation.x) + (self.b * translation.y) + (self.c * translation.z);
  }
}

pub fn intersection_with_planes(p1: &Plane, p2: &Plane, p3: &Plane) -> Vec3 {
  let (p1a, p1b, p1c, p1d) = (p1.a as f64, p1.b as f64, p1.c as f64, p1.d as f64);
  let (p2a, p2b, p2c, p2d) = (p2.a as f64, p2.b as f64, p2.c as f64, p2.d as f64);
  let (p3a, p3b, p3c, p3d) = (p3.a as f64, p3.b as f64, p3.c as f64, p3.d as f64);

  let bc1: f64 = (p1b * p3c) - (p3b * p1c);
  let bc2: f64 = (p2b * p1c) - (p1b * p2c);
  let bc3: f64 = (p3b * p2c) - (p2b * p3c);

  let ad1: f64 = (p1a * p3d) - (p3a * p1d);
  let ad2: f64 = (p2a * p1d) - (p1a * p2d);
  let ad3: f64 = (p3a * p2d) - (p2a * p3d);

  let x: f64 = -((p1d * bc3) + (p2d * bc1) + (p3d * bc2));
  let y: f64 = -((p1c * ad3) + (p2c * ad1) + (p3c * ad2));
  let z: f64 = (p1b * ad3) + (p2b * ad1) + (p3b * ad2);
  let w: f64 = -((p1a * bc3) + (p2a * bc1) + (p3a * bc2));

  // better to have detectable invalid values than to have really big values
  if w > -NORMAL_EPSILON && w < NORMAL_EPSILON {
    Vec3::splat(f32::NAN)
  } else {
    Vec3 {
      x: (x / w) as f32,
      y: (y / w) as f32,
      z: (z / w) as f32,
    }
  }
}

pub fn intersection_with_ray(start: Vec3, end: Vec3, start_dist: f32, end_dist: f32) -> Vec3 {
  let vector = end.sub(start);
  let length = end_dist - start_dist;
  let delta = end_dist / length;
  end - (delta * vector)
}

// These methods are designed for clarity and readability,
//	if speed is your concern do not use enums and use the floating point values directly!!
pub fn on_side_epsilon(distance: f32, epsilon: f32) -> PlaneSideResult {
  if distance > epsilon {
    PlaneSideResult::Outside
  } else if distance < -epsilon {
    PlaneSideResult::Inside
  } else {
    PlaneSideResult::Intersects
  }
}

pub fn on_side(distance: f32) -> PlaneSideResult {
  on_side_epsilon(distance, DISTANCE_EPSILON)
}

pub fn translated(plane: &Plane, translation: Vec3) -> Plane {
  Plane {
    a: plane.a,
    b: plane.b,
    c: plane.c,
    d: plane.d + plane.a * translation.x + plane.b * translation.y + plane.c * translation.z,
  }
}

pub fn translated_xyz(plane: &Plane, x: f32, y: f32, z: f32) -> Plane {
  Plane {
    a: plane.a,
    b: plane.b,
    c: plane.c,
    d: plane.d + plane.a * x + plane.b * y + plane.c * z,
  }
}

#[derive(Clone)]
pub enum PolygonCategory {
  Inside,
  Aligned,
  ReverseAligned,
  Outside,
}

#[derive(Clone)]
pub struct Polygon {
  first_index: i16,
  plane_index: i16,
  category: PolygonCategory,
  visible: bool,
  bounds: Aabb,
}

impl Polygon {
  fn new(plane_index: usize) -> Self {
    Self {
      first_index: 0,
      plane_index: plane_index as i16,
      category: PolygonCategory::Inside,
      visible: false,
      bounds: Default::default(),
    }
  }
}

impl Default for Polygon {
  fn default() -> Self {
    Self {
      first_index: -1,
      plane_index: -1,
      category: PolygonCategory::Aligned,
      visible: false,
      bounds: Aabb::new(),
    }
  }
}

pub enum PolygonSplitResult {
  CompletelyInside,  // Polygon is completely inside half-space defined by plane
  CompletelyOutside, // Polygon is completely outside half-space defined by plane

  Split, // Polygon has been split into two parts by plane

  PlaneAligned, // Polygon is aligned with cutting plane and the polygons' normal points in the same direction
  PlaneOppositeAligned, // Polygon is aligned with cutting plane and the polygons' normal points in the opposite direction
}

pub struct EdgeIntersection<'a> {
  plane_indices: [i16; 2],
  edge: &'a HalfEdge,
}

pub struct PointIntersection<'a> {
  edges: Vec<EdgeIntersection<'a>>,
  plane_indices: HashSet<i16>,
  vertex_index: i16,
}

impl<'a> PointIntersection<'a> {
  pub fn new(vertex_index: i16, planes: &Vec<i16>) -> Self {
    let mut plane_indices = HashSet::new();
    for plane in planes {
      plane_indices.insert(*plane);
    }
    Self {
      edges: vec![],
      plane_indices,
      vertex_index,
    }
  }
}

#[derive(Clone)]
pub struct CsgMesh {
  bounds: Aabb,
  polygons: Vec<Polygon>,
  edges: Vec<HalfEdge>,
  vertices: Vec<Vec3>,
  planes: Vec<Plane>,
}

pub fn create_from_planes(brush_planes: Vec<Plane>) -> CsgMesh {
  let mut planes: Vec<Plane> = brush_planes.clone();

  let mut point_intersections: Vec<PointIntersection> =
    Vec::with_capacity(planes.len() * planes.len());
  let mut intersecting_planes: Vec<i16> = vec![];
  let mut vertices: Vec<Vec3> = vec![];
  let mut edges: Vec<HalfEdge> = vec![];

  for plane_index_1 in 0..(planes.len() - 2) {
    let plane1 = &planes[plane_index_1];
    for plane_index_2 in (plane_index_1 + 1)..(planes.len() - 1) {
      let plane2 = &planes[plane_index_2];
      'skip_intersection: for plane_index3 in (plane_index_2 + 1)..(planes.len()) {
        let plane3 = &planes[plane_index3];
        let vertex = intersection_with_planes(plane1, plane2, plane3);
        if vertex.is_nan() || !vertex.is_finite() {
          continue;
        }
        intersecting_planes.clear();
        intersecting_planes.push(plane_index_1 as i16);
        intersecting_planes.push(plane_index_2 as i16);
        intersecting_planes.push(plane_index3 as i16);
        for plane_index4 in 0..planes.len() {
          if plane_index4 == plane_index_1
            || plane_index4 == plane_index_2
            || plane_index4 == plane_index3
          {
            continue;
          }
          let plane4 = &planes[plane_index4];
          let side = plane4.on_side_vec3(vertex);
          match side {
            PlaneSideResult::Intersects => {
              if plane_index4 < plane_index3 {
                continue 'skip_intersection;
              }
              intersecting_planes.push(plane_index4 as i16);
            }
            PlaneSideResult::Outside => {
              continue 'skip_intersection;
            }
            _ => {}
          }
          let vertex_index = vertices.len() as i16;
          vertices.push(vertex);
          point_intersections.push(PointIntersection::new(vertex_index, &intersecting_planes));
        }
      }
    }
  }
  let mut found_planes: [i16; 2] = default();
  // Find all our intersection edges which are formed by a pair of planes
  // (this could probably be done inside the previous loop)
  for i in 0..point_intersections.len() {
    let mut point_intersection_a = &mut point_intersections[i];
    for j in (i + 1)..point_intersections.len() {
      let point_intersection_b = &mut point_intersections[j];
      let planes_indices_a = &point_intersection_a.plane_indices;
      let planes_indices_b = &point_intersection_b.plane_indices;

      let mut found_plane_index = 0;
      for current_plane_index in planes_indices_a {
        if !planes_indices_b.contains(current_plane_index) {
          continue;
        }

        found_planes[found_plane_index] = *current_plane_index;
        found_plane_index += 1;

        if found_plane_index == 2 {
          break;
        }
      }

      // If foundPlaneIndex is 0 or 1 then either this combination does not exist,
      // or only goes trough one point
      if found_plane_index < 2 {
        continue;
      }

      // Create our found intersection edge
      let half_edge_a_index = edges.len() as i16;
      let half_edge_b_index = half_edge_a_index + 1;

      let mut half_edge_a = HalfEdge {
        next_index: 0,
        twin_index: half_edge_b_index,
        vertex_index: point_intersection_a.vertex_index,
        polygon_index: 0,
      };
      edges.push(half_edge_a);

      let mut half_edge_b = HalfEdge {
        next_index: 0,
        twin_index: half_edge_a_index,
        vertex_index: point_intersection_b.vertex_index,
        polygon_index: 0,
      };
      edges.push(half_edge_b);

      // Add it to our points
      point_intersection_a.edges.push(EdgeIntersection {
        plane_indices: [found_planes[0], found_planes[1]],
        edge: &edges[half_edge_a_index as usize],
      });
      point_intersection_b.edges.push(EdgeIntersection {
        plane_indices: [found_planes[0], found_planes[1]],
        edge: &edges[half_edge_b_index as usize],
      })
    }
  }

  let mut polygons = vec![];
  for i in 0..planes.len() {
    polygons.push(Polygon::new(i));
  }

  let mut bounds = Aabb::default();
  let mut direction = Vec3::default();
  let mut i = point_intersections.len() - 1;
  while i >= 0 {
    let mut point_intersection = &mut point_intersections[i];
    let mut point_edges = &mut point_intersection.edges;

    // Make sure that we have at least 2 edges ...
    // This may happen when a plane only intersects at a single edge.
    if point_edges.len() <= 2 {
      point_intersections.remove(i);
      continue;
    }

    let vertex_index = point_intersection.vertex_index;
    let vertex = vertices[vertex_index as usize];

    for j in 0..(point_edges.len() - 1) {
      let edge1 = &mut point_edges[j];
      for k in (j + 1)..point_edges.len() {
        let edge2 = &mut point_edges[k];
        let mut plane_index_1;
        let mut plane_index_2;
        // Determine if and which of our 2 planes are identical
        if edge1.plane_indices[0] == edge2.plane_indices[0] {
          plane_index_1 = 0;
          plane_index_2 = 0;
        } else if edge1.plane_indices[0] == edge2.plane_indices[1] {
          plane_index_1 = 0;
          plane_index_2 = 1;
        } else if edge1.plane_indices[1] == edge2.plane_indices[0] {
          plane_index_1 = 1;
          plane_index_2 = 0;
        } else if edge1.plane_indices[1] == edge2.plane_indices[0] {
          plane_index_1 = 1;
          plane_index_2 = 1;
        } else {
          continue;
        }

        let mut ingoing: &mut HalfEdge;
        let mut outgoing: &mut HalfEdge;
        let mut outgoing_index;

        let shared_plane = planes[edge1.plane_indices[plane_index_1] as usize];
        let edge1_plane = planes[edge1.plane_indices[1 - plane_index_1] as usize];
        let edge2_plane = planes[edge2.plane_indices[1 - plane_index_2] as usize];

        direction = shared_plane.get_normal().cross(edge1_plane.get_normal());

        // Determine the orientation of our two edges to determine
        // which edge is in-going, and which one is out-going
        if direction.dot(edge2_plane.get_normal()) < 0.0 {
          ingoing = &mut edge2.edge;
          outgoing_index = edge1.edge.twin_index;
          outgoing = &mut edges[outgoing_index as usize];
        } else {
          ingoing = &mut edge1.edge;
          outgoing_index = edge2.edge.twin_index;
          outgoing = &mut edges[outgoing_index as usize];
        }

        // Link the out-going half-edge to the in-going half-edge
        ingoing.next_index = outgoing_index;

        // Add reference to polygon to half-edge, and make sure our
        // polygon has a reference to a half-edge
        // Since a half-edge, in this case, serves as a circular
        // linked list this just works.
        let polygon_index = edge1.plane_indices[plane_index_1];

        ingoing.polygon_index = polygon_index;
        outgoing.polygon_index = polygon_index;

        let polygon = &mut polygons[polygon_index as usize];
        polygon.first_index = outgoing_index;
        polygon.bounds.add_vec3(vertex);
      }
    }
    bounds.add_vec3(vertex);
    i -= 1;
  }

  CsgMesh {
    bounds,
    polygons,
    edges,
    vertices,
    planes,
  }
}

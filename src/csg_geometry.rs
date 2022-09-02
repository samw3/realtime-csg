//! Geometry

use bevy::math::{IVec3, Vec3};
use std::hash::{Hash, Hasher};
use std::ops::Sub;

pub const DISTANCE_EPSILON: f32 = 0.0001;
pub const NORMAL_EPSILON: f64 = 1.0 / 65535.0;

pub enum PlaneSideResult {
    Outside,
    Inside,
    Intersects,
}

#[derive(Clone)]
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

pub enum PolygonCategory {
    Inside,
    Aligned,
    ReverseAligned,
    Outside,
}

pub struct Polygon {
    first_index: i16,
    plane_index: i16,
    category: PolygonCategory,
    visible: bool,
    bounds: Aabb,
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
    CompletelyInside,		// Polygon is completely inside half-space defined by plane
    CompletelyOutside,		// Polygon is completely outside half-space defined by plane

    Split,					// Polygon has been split into two parts by plane

    PlaneAligned,			// Polygon is aligned with cutting plane and the polygons' normal points in the same direction
    PlaneOppositeAligned	// Polygon is aligned with cutting plane and the polygons' normal points in the opposite direction
}
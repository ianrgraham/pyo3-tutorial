use nalgebra::{Point3, Vector3};
use ordered_float::OrderedFloat;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

// Define a point in 3D space with a unique ID
#[derive(Clone, Debug)]
struct Point {
    id: usize,
    point: Point3<f32>,
}

impl Point {
    // Computes the squared distance between two points
    fn distance_squared(&self, other: &Point) -> f32 {
        (other.point - self.point).magnitude_squared()
    }
}

#[derive(Clone)]
struct AabbTree {
    root: AabbNode,
}

// Define a node in the AABB tree
#[derive(Clone)]
struct AabbNode {
    aabb: Aabb,
    left: Option<Box<AabbNode>>,
    right: Option<Box<AabbNode>>,
    points: Vec<Point>,
}

// Define an axis-aligned bounding box (AABB)
#[derive(Clone)]
struct Aabb {
    min: Point3<f32>,
    max: Point3<f32>,
}

impl Aabb {
    // Compute the distance from a point to the AABB
    fn distance_to_point(&self, point: &Point3<f32>) -> f32 {
        let mut distance_squared = 0.0;
        for i in 0..3 {
            let v = point[i];
            if v < self.min[i] {
                distance_squared += (self.min[i] - v).powi(2);
            } else if v > self.max[i] {
                distance_squared += (v - self.max[i]).powi(2);
            }
        }
        distance_squared.sqrt()
    }

    // Tests if a point is contained within the AABB
    fn contains(&self, point: &Point) -> bool {
        point.point.x >= self.min.x && point.point.x <= self.max.x &&
        point.point.y >= self.min.y && point.point.y <= self.max.y &&
        point.point.z >= self.min.z && point.point.z <= self.max.z
    }

    // Computes the squared distance between a point and the closest point on the AABB
    fn distance_squared(&self, point: &Point) -> f32 {
        let mut d = 0.0;
        if point.point.x < self.min.x {
            d += (point.point.x - self.min.x).powi(2);
        } else if point.point.x > self.max.x {
            d += (point.point.x - self.max.x).powi(2);
        }
        if point.point.y < self.min.y {
            d += (point.point.y - self.min.y).powi(2);
        } else if point.point.y > self.max.y {
            d += (point.point.y - self.max.y).powi(2);
        }
        if point.point.z < self.min.z {
            d += (point.point.z - self.min.z).powi(2);
        } else if point.point.z > self.max.z {
            d += (point.point.z - self.max.z).powi(2);
        }
        d
    }

    // Computes the length of the longest axis of the AABB
    fn longest_axis(&self) -> usize {
        let extents = self.max - self.min;
        if extents.x > extents.y && extents.x > extents.z {
            0
        } else if extents.y > extents.z {
            1
        } else {
            2
        }
    }
}

// Define a priority queue for storing nodes during traversal of the AABB tree
#[derive(Eq, PartialEq)]
struct PriorityQueueEntry {
    distance_squared: OrderedFloat<f32>,
    node_id: usize,
}

impl Ord for PriorityQueueEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .distance_squared
            .partial_cmp(&self.distance_squared)
            .unwrap()
    }
}

impl PartialOrd for PriorityQueueEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// Define the nearest neighbor search algorithm using an AABB tree
fn nearest_neighbors_aabb_tree(points: &[Point], query_point: &Point, k: usize) -> Vec<Point> {
    let mut heap = BinaryHeap::new();
    let mut result = Vec::<Point>::new();

    // Build the AABB tree
    let root = build_aabb_tree(points, 0, points.len() - 1);

    // Traverse the AABB tree and find the nearest neighbors
    traverse_aabb_tree(&root, query_point, &mut heap, k);

    // Extract the nearest neighbors from the priority queue
    while let Some(PriorityQueueEntry { node_id, .. }) = heap.pop() {
        let node = &root[node_id];
        result.extend(node.points.iter());
    }

    // Sort the nearest neighbors by distance from the query point
    result.sort_by(|a, b| {
        a.distance_squared(&query_point)
            .partial_cmp(&b.distance_squared(&query_point))
            .unwrap()
    });

    // Return the k nearest neighbors
    result.into_iter().take(k).collect()
}

// Traverses an AABB tree in depth-first order
fn traverse_aabb_tree(node: &AabbNode, query_point: &Point, heap: &mut BinaryHeap<PriorityQueueEntry>, k: usize) {
    for point in &node.points {
        let distance_squared = OrderedFloat(-point.distance_squared(query_point));
        let node_id = point.id;
        let entry = PriorityQueueEntry{distance_squared, node_id};
        if heap.len() < k {
            heap.push(entry);
        } else if distance_squared > *heap.peek().unwrap().distance_squared {
            heap.pop();
            heap.push(entry);
        }
    }

    let mut further_node: Option<&Box<AabbNode>> = None;
    let mut closer_node: Option<&Box<AabbNode>> = None;
    if let Some(left_node) = &node.left {
        if left_node.aabb.contains(query_point) {
            closer_node = Some(left_node);
        } else {
            further_node = Some(left_node);
        }
    }
    if let Some(right_node) = &node.right {
        if right_node.aabb.contains(query_point) {
            closer_node = Some(right_node);
        } else {
            further_node = Some(right_node);
        }
    }
    
    if let Some(node) = closer_node {
        traverse_aabb_tree(node, query_point, heap, k);
    }
    
    if let Some(node) = further_node {
        if heap.len() < k || node.aabb.distance_squared(query_point) < -*heap.peek().unwrap().0 {
            traverse_aabb_tree(node, query_point, heap, k);
        }
    }
}

// Build an AABB tree for a set of points
fn build_aabb_tree(points: &[Point], start: usize, end: usize) -> Box<AabbNode> {
    let mut aabb = Aabb {
        min: Point3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
        max: Point3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY),
    };
    for i in start..=end {
        aabb.min.x = aabb.min.x.min(points[i].point.x);
        aabb.min.y = aabb.min.y.min(points[i].point.y);
        aabb.min.z = aabb.min.z.min(points[i].point.z);
        aabb.max.x = aabb.max.x.max(points[i].point.x);
        aabb.max.y = aabb.max.y.max(points[i].point.y);
        aabb.max.z = aabb.max.z.max(points[i].point.z);
    }
    if start == end {
        // Leaf node
        Box::new(AabbNode {
            aabb,
            left: None,
            right: None,
            points: vec![points[start].clone()],
        })
    } else {
        // Interior node
        let axis = aabb.longest_axis();
        let mid = (start + end) / 2;
        let mut sorted_points = points[start..=end].to_vec();
        sorted_points.sort_by(|a, b| a.point[axis].partial_cmp(&b.point[axis]).unwrap());
        let left = build_aabb_tree(&sorted_points, start, mid);
        let right = build_aabb_tree(&sorted_points, mid + 1, end);
        let mut points = Vec::new();
        points.extend_from_slice(&left.points);
        points.extend_from_slice(&right.points);
        Box::new(AabbNode {
            aabb,
            left: Some(left),
            right: Some(right),
            points,
        })
    }
}
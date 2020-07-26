use crate::provider::GridProvider;
use indexmap::IndexSet;
use nalgebra::Vector3;
use ordered_float::NotNan;
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::fmt::Debug;
use std::hash::{Hash, Hasher};

#[derive(Debug, Clone)]
struct Node {
    vector: Vector3<f32>,
    cost: f32,
    estimated_cost: f32,
    previous_node_index: usize,
}

/// Finds the path from one vector to another.
/// Returns list of vectors to achieve the goal.
pub fn find_path<G: GridProvider>(
    start: Vector3<f32>,
    goal: Vector3<f32>,
    grid_provider: G,
) -> Option<Vec<Vector3<f32>>> {
    let mut reachable = BinaryHeap::new();
    let mut explored = IndexSet::new();

    let start_estimated_distance = (start - &goal).magnitude();

    reachable.push(Node {
        vector: start,
        cost: 0.0,
        estimated_cost: start_estimated_distance,
        previous_node_index: 0,
    });

    while let Some(current) = reachable.pop() {
        if current.vector == goal {
            let mut path = Vec::new();
            reconstruct_path(&current, &explored, &mut path);
            path.reverse();

            return Some(path);
        }

        let (explored_node_index, _) = explored.insert_full(current.clone());

        for (adjacent_vector, cost) in adjacent_vectors(&current.vector, &grid_provider) {
            let estimated_distance = (adjacent_vector - &goal).magnitude();

            let mut adjacent_node = Node {
                vector: adjacent_vector,
                cost,
                estimated_cost: estimated_distance,
                previous_node_index: explored_node_index,
            };

            if explored.contains(&adjacent_node) {
                continue;
            }

            let new_cost = current.cost + cost;

            if adjacent_node.cost > new_cost {
                adjacent_node.cost = new_cost;
            }

            reachable.push(adjacent_node);
        }
    }

    None
}

fn adjacent_vectors<G: GridProvider>(
    vector: &Vector3<f32>,
    grid_provider: &G,
) -> Vec<(Vector3<f32>, f32)> {
    let mut adjacent = Vec::new();

    for x in -1..=1 {
        for y in -1..=1 {
            for z in -1..=1 {
                if x == 0 && y == 0 && z == 0 {
                    continue;
                }

                let adjacent_vector = Vector3::new(x as f32, y as f32, z as f32) + vector;

                if !grid_provider.is_obstacle(adjacent_vector) {
                    adjacent.push((adjacent_vector, 1.0))
                }
            }
        }
    }

    adjacent
}

fn reconstruct_path(node: &Node, explored: &IndexSet<Node>, path: &mut Vec<Vector3<f32>>) {
    path.push(node.vector);

    if node.previous_node_index != 0 {
        let previous_node = explored.get_index(node.previous_node_index).unwrap();
        reconstruct_path(&previous_node, explored, path);
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        let total_cost1 = self.cost + self.estimated_cost;
        let total_cost2 = other.cost + other.estimated_cost;

        // This how we convert max binary heap to min binary heap.
        total_cost2.partial_cmp(&total_cost1)
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap()
    }
}

impl Eq for Node {}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.vector.eq(&other.vector)
    }
}

impl Hash for Node {
    fn hash<H: Hasher>(&self, state: &mut H) {
        unsafe {
            let x_non_nan = NotNan::unchecked_new(self.vector.x);
            let y_non_nan = NotNan::unchecked_new(self.vector.y);
            let z_non_nan = NotNan::unchecked_new(self.vector.z);

            x_non_nan.hash(state);
            y_non_nan.hash(state);
            z_non_nan.hash(state);
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::pathfinding::find_path;
    use crate::provider::GridProvider;
    use nalgebra::Vector3;

    struct SimpleGridProvider {}

    impl GridProvider for SimpleGridProvider {
        fn is_obstacle(&self, _vector: Vector3<f32>) -> bool {
            false
        }
    }

    #[test]
    fn test_simple_straight_path() {
        let start = Vector3::new(0.0, 0.0, 0.0);
        let goal = Vector3::new(10.0, 0.0, 0.0);

        let grid_provider = SimpleGridProvider {};

        let path = find_path(start, goal, grid_provider).unwrap();
        assert_eq!(path.len(), 10);

        for i in 1..11 {
            let vec = path.get(i - 1).unwrap();

            assert_eq!(vec.x, 1.0 * i as f32);
            assert_eq!(vec.y, 0.0);
            assert_eq!(vec.z, 0.0);
        }
    }

    #[test]
    fn test_simple_horizontal_path() {
        let start = Vector3::new(0.0, 0.0, 0.0);
        let goal = Vector3::new(-10.0, -10.0, 0.0);

        let grid_provider = SimpleGridProvider {};

        let path = find_path(start, goal, grid_provider).unwrap();
        assert_eq!(path.len(), 10);

        for i in 1..11 {
            let vec = path.get(i - 1).unwrap();

            assert_eq!(vec.x, -1.0 * i as f32);
            assert_eq!(vec.y, -1.0 * i as f32);
            assert_eq!(vec.z, 0.0);
        }
    }

    struct WalledGridProvider {}

    impl GridProvider for WalledGridProvider {
        fn is_obstacle(&self, vector: Vector3<f32>) -> bool {
            if vector.x == 5.0 {
                if 5.0 > vector.y.abs() {
                    return true;
                }
            }

            if vector.z != 0.0 {
                return true;
            }

            false
        }
    }

    #[test]
    fn test_walled_path() {
        let start = Vector3::new(0.0, 0.0, 0.0);
        let goal = Vector3::new(10.0, 0.0, 0.0);

        let grid_provider = WalledGridProvider {};

        let path = find_path(start, goal, grid_provider).unwrap();
        assert_eq!(path.len(), 13);

        // #
        // - #
        // - - #
        // - - - #
        // - - - - #
        // + + + + + #
        // - # # # #
        // #
        // #
        // #

        for i in 1..4 {
            let vec = path.get(i - 1).unwrap();

            assert_eq!(vec.x, 1.0 * i as f32);
            assert_eq!(vec.y, 0.0);
            assert_eq!(vec.z, 0.0);
        }

        for i in 1..5 {
            let vec = path.get(2 + i).unwrap();

            assert_eq!(vec.x, 4.0);
            assert_eq!(vec.y, 1.0 * i as f32);
            assert_eq!(vec.z, 0.0);
        }

        for i in 1..7 {
            let vec = path.get(6 + i).unwrap();

            assert_eq!(vec.x, 4.0 + 1.0 * i as f32);
            assert_eq!(vec.y, 6.0 - 1.0 * i as f32);
            assert_eq!(vec.z, 0.0);
        }
    }
}

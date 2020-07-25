use indexmap::IndexSet;
use nalgebra::Vector3;
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::fmt::Debug;
use std::hash::{Hash, Hasher};

#[derive(Debug, Clone)]
struct Node<L> {
    local_vector: L,
    cost: f32,
    estimated_cost: f32,
    previous_node_index: usize,
}

pub trait LocalVectorProvider<L: Distance + Ord + Eq + Hash + Clone + Debug> {
    /// Converts global vector to local vector.
    fn get_local_vector(&self, vector: Vector3<f32>) -> Option<L>;

    /// Returns a list of vacant neighbours with the cost of moving.
    fn get_adjacent_vectors(&self, local_vector: &L) -> Vec<(L, f32)>;

    /// Converts local vector to global vector.
    fn to_global_vector(&self, local_vector: &L) -> Vector3<f32>;
}

#[derive(Debug)]
pub enum PathfindingError {
    /// The goal can't be reached.
    NoPath,
    /// Local vector provider was not able to convert global vector to local vector.
    /// This may happen when trying find path out of map bounds.
    UnresolvedGlobalVector,
}

/// Finds the path from one vector to another.
/// Returns list of vectors to achieve the goal.
pub fn find_path<L: Distance + Ord + Eq + Hash + Clone + Debug, LP: LocalVectorProvider<L>>(
    start: Vector3<f32>,
    goal: Vector3<f32>,
    local_vector_provider: LP,
) -> Result<Vec<Vector3<f32>>, PathfindingError> {
    let mut reachable = BinaryHeap::new();
    let mut explored = IndexSet::new();

    let start_local_vector_opt = local_vector_provider.get_local_vector(start);
    let goal_local_vector_opt = local_vector_provider.get_local_vector(goal);

    match (start_local_vector_opt, goal_local_vector_opt) {
        (Some(start_local_vector), Some(goal_local_vector)) => {
            let start_estimated_distance = start_local_vector.distance(&goal_local_vector);

            reachable.push(Node {
                local_vector: start_local_vector,
                cost: 0.0,
                estimated_cost: start_estimated_distance,
                previous_node_index: 0,
            });

            while let Some(current) = reachable.pop() {
                if current.local_vector == goal_local_vector {
                    let mut path = Vec::new();
                    reconstruct_path(&current, local_vector_provider, &explored, &mut path);
                    path.reverse();

                    return Ok(path);
                }

                let (explored_node_index, _) = explored.insert_full(current.clone());

                for (adjacent_local_vector, cost) in local_vector_provider
                    .get_adjacent_vectors(&current.local_vector)
                    .into_iter()
                {
                    let estimated_distance = adjacent_local_vector.distance(&goal_local_vector);

                    let mut adjacent_node = Node {
                        local_vector: adjacent_local_vector,
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

            Err(PathfindingError::NoPath)
        }
        _ => Err(PathfindingError::UnresolvedGlobalVector),
    }
}

fn reconstruct_path<L: Distance + Ord + Eq + Hash + Clone + Debug, LP: LocalVectorProvider<L>>(
    node: &Node<L>,
    local_vector_provider: LP,
    explored: &IndexSet<Node<L>>,
    path: &mut Vec<Vector3<f32>>,
) {
    path.push(local_vector_provider.to_global_vector(&node.local_vector));

    if node.previous_node_index != 0 {
        let previous_node = explored.get_index(node.previous_node_index).unwrap();
        reconstruct_path(&previous_node, local_vector_provider, explored, path);
    }
}

pub trait Distance {
    fn distance(&self, other: &Self) -> f32;
}

impl<L: Distance + Ord + Eq + Hash + Debug> PartialOrd for Node<L> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        let total_cost1 = self.cost + self.estimated_cost;
        let total_cost2 = other.cost + other.estimated_cost;

        total_cost2.partial_cmp(&total_cost1)
    }
}

impl<L: Distance + Ord + Eq + Hash + Debug> Ord for Node<L> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap()
    }
}

impl<L: Distance + Ord + Eq + Hash + Debug> Eq for Node<L> {}

impl<L: Distance + Ord + Eq + Hash + Debug> PartialEq for Node<L> {
    fn eq(&self, other: &Self) -> bool {
        self.local_vector.eq(&other.local_vector)
    }
}

impl<L: Distance + Ord + Eq + Hash + Debug> Hash for Node<L> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.local_vector.hash(state)
    }
}

#[cfg(test)]
mod tests {
    use crate::pathfinding::{find_path, Distance, LocalVectorProvider};
    use nalgebra::Vector3;

    #[derive(Hash, Ord, PartialOrd, Eq, PartialEq, Clone, Debug)]
    struct TestLocalVector {
        x: i32,
        y: i32,
        z: i32,
    }

    impl TestLocalVector {
        fn add(&self, x: i32, y: i32, z: i32) -> Self {
            TestLocalVector {
                x: self.x + x,
                y: self.y + y,
                z: self.z + z,
            }
        }
    }

    impl Distance for TestLocalVector {
        fn distance(&self, other: &Self) -> f32 {
            let x = (self.x - other.x) as f32;
            let y = (self.y - other.y) as f32;
            let z = (self.z - other.z) as f32;

            (x * x + y * y + z * z).sqrt()
        }
    }

    struct SimpleLocalVectorProvider {}

    impl LocalVectorProvider<TestLocalVector> for SimpleLocalVectorProvider {
        fn get_local_vector(&self, vector: Vector3<f32>) -> Option<TestLocalVector> {
            Some(TestLocalVector {
                x: vector.x as i32,
                y: vector.y as i32,
                z: vector.z as i32,
            })
        }

        fn get_adjacent_vectors(
            &self,
            local_vector: &TestLocalVector,
        ) -> Vec<(TestLocalVector, f32)> {
            let mut adjacent = Vec::new();

            let local_vector1 = local_vector.add(1, 0, 0);
            let local_vector2 = local_vector.add(0, 1, 0);
            let local_vector3 = local_vector.add(1, 1, 0);
            let local_vector4 = local_vector.add(-1, -1, 0);
            let local_vector5 = local_vector.add(1, -1, 0);
            let local_vector6 = local_vector.add(-1, 1, 0);

            adjacent.push((local_vector1, 1.0));
            adjacent.push((local_vector2, 1.0));
            adjacent.push((local_vector3, 1.0));
            adjacent.push((local_vector4, 1.0));
            adjacent.push((local_vector5, 1.0));
            adjacent.push((local_vector6, 1.0));

            adjacent
        }

        fn to_global_vector(&self, local_vector: &TestLocalVector) -> Vector3<f32> {
            Vector3::new(
                local_vector.x as f32,
                local_vector.y as f32,
                local_vector.z as f32,
            )
        }
    }

    #[test]
    fn test_simple_straight_path() {
        let start = Vector3::new(0.0, 0.0, 0.0);
        let goal = Vector3::new(10.0, 0.0, 0.0);

        let simple_local_local_vector_provider = SimpleLocalVectorProvider {};

        let path = find_path(start, goal, simple_local_local_vector_provider).unwrap();
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

        let simple_local_local_vector_provider = SimpleLocalVectorProvider {};

        let path = find_path(start, goal, simple_local_local_vector_provider).unwrap();
        assert_eq!(path.len(), 10);

        for i in 1..11 {
            let vec = path.get(i - 1).unwrap();

            assert_eq!(vec.x, -1.0 * i as f32);
            assert_eq!(vec.y, -1.0 * i as f32);
            assert_eq!(vec.z, 0.0);
        }
    }

    struct WalledLocalVectorProvider {}

    impl LocalVectorProvider<TestLocalVector> for WalledLocalVectorProvider {
        fn get_local_vector(&self, vector: Vector3<f32>) -> Option<TestLocalVector> {
            Some(TestLocalVector {
                x: vector.x as i32,
                y: vector.y as i32,
                z: vector.z as i32,
            })
        }

        fn get_adjacent_vectors(
            &self,
            local_vector: &TestLocalVector,
        ) -> Vec<(TestLocalVector, f32)> {
            fn add(local_vector: TestLocalVector, adjacent: &mut Vec<(TestLocalVector, f32)>) {
                if local_vector.x == 5 {
                    if 5 > local_vector.y.abs() {
                        return;
                    }
                }

                adjacent.push((local_vector, 1.0));
            }

            let mut adjacent = Vec::new();

            let local_vector1 = local_vector.add(1, 0, 0);
            let local_vector2 = local_vector.add(0, 1, 0);
            let local_vector3 = local_vector.add(1, 1, 0);
            let local_vector4 = local_vector.add(-1, -1, 0);
            let local_vector5 = local_vector.add(1, -1, 0);
            let local_vector6 = local_vector.add(-1, 1, 0);

            add(local_vector1, &mut adjacent);
            add(local_vector2, &mut adjacent);
            add(local_vector3, &mut adjacent);
            add(local_vector4, &mut adjacent);
            add(local_vector5, &mut adjacent);
            add(local_vector6, &mut adjacent);

            adjacent
        }

        fn to_global_vector(&self, local_vector: &TestLocalVector) -> Vector3<f32> {
            Vector3::new(
                local_vector.x as f32,
                local_vector.y as f32,
                local_vector.z as f32,
            )
        }
    }

    #[test]
    fn test_walled_path() {
        let start = Vector3::new(0.0, 0.0, 0.0);
        let goal = Vector3::new(10.0, 0.0, 0.0);

        let walled_local_local_vector_provider = WalledLocalVectorProvider {};

        let path = find_path(start, goal, walled_local_local_vector_provider).unwrap();
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

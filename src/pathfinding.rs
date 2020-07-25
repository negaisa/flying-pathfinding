use indexmap::IndexSet;
use nalgebra::Vector3;
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::fmt::Debug;
use std::hash::{Hash, Hasher};

pub trait Distance {
    fn distance(&self, other: &Self) -> f32;
}

#[derive(Debug, Clone)]
struct Node<L> {
    location: L,
    cost: f32,
    estimated_cost: f32,
    previous_node_index: usize,
}

pub trait LocalLocationProvider<L: Distance + Ord + Eq + Hash + Clone + Debug> {
    fn get_local_location(&self, vector: Vector3<f32>) -> Option<L>;

    fn find_adjacent(&self, location: &L) -> Vec<(L, f32)>;

    fn to_global_location(&self, location: &L) -> Vector3<f32>;
}

#[derive(Debug)]
pub enum PathfindingError {
    NoPath,
    UnknownLocation,
}

/// Finds the path from one vector to another.
/// Returns vec vectors to achieve the goal.
pub fn find_path<L: Distance + Ord + Eq + Hash + Clone + Debug, LP: LocalLocationProvider<L>>(
    start: Vector3<f32>,
    goal: Vector3<f32>,
    location_provider: LP,
) -> Result<Vec<Vector3<f32>>, PathfindingError> {
    let mut reachable = BinaryHeap::new();
    let mut explored = IndexSet::new();

    let start_location_opt = location_provider.get_local_location(start);
    let goal_location_opt = location_provider.get_local_location(goal);

    match (start_location_opt, goal_location_opt) {
        (Some(start_location), Some(goal_location)) => {
            let start_estimated_distance = start_location.distance(&goal_location);

            reachable.push(Node {
                location: start_location,
                cost: 0.0,
                estimated_cost: start_estimated_distance,
                previous_node_index: 0,
            });

            while let Some(current) = reachable.pop() {
                if current.location == goal_location {
                    let mut path = Vec::new();
                    reconstruct_path(&current, location_provider, &explored, &mut path);
                    path.reverse();

                    return Ok(path);
                }

                let (explored_node_index, _) = explored.insert_full(current.clone());

                for (adjacent_location, cost) in location_provider
                    .find_adjacent(&current.location)
                    .into_iter()
                {
                    let estimated_distance = adjacent_location.distance(&goal_location);

                    let mut adjacent_node = Node {
                        location: adjacent_location,
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
        _ => Err(PathfindingError::UnknownLocation),
    }
}

fn reconstruct_path<L: Distance + Ord + Eq + Hash + Clone + Debug, LP: LocalLocationProvider<L>>(
    node: &Node<L>,
    location_provider: LP,
    explored: &IndexSet<Node<L>>,
    path: &mut Vec<Vector3<f32>>,
) {
    path.push(location_provider.to_global_location(&node.location));

    if node.previous_node_index != 0 {
        let previous_node = explored.get_index(node.previous_node_index).unwrap();
        reconstruct_path(&previous_node, location_provider, explored, path);
    }
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
        self.location.eq(&other.location)
    }
}

impl<L: Distance + Ord + Eq + Hash + Debug> Hash for Node<L> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.location.hash(state)
    }
}

#[cfg(test)]
mod tests {
    use crate::pathfinding::{find_path, Distance, LocalLocationProvider};
    use nalgebra::Vector3;

    #[derive(Hash, Ord, PartialOrd, Eq, PartialEq, Clone, Debug)]
    struct TestLocalLocation {
        x: i32,
        y: i32,
        z: i32,
    }

    impl TestLocalLocation {
        fn add(&self, x: i32, y: i32, z: i32) -> Self {
            TestLocalLocation {
                x: self.x + x,
                y: self.y + y,
                z: self.z + z,
            }
        }
    }

    impl Distance for TestLocalLocation {
        fn distance(&self, other: &Self) -> f32 {
            let x = (self.x - other.x) as f32;
            let y = (self.y - other.y) as f32;
            let z = (self.z - other.z) as f32;

            (x * x + y * y + z * z).sqrt()
        }
    }

    struct SimpleLocalLocationProvider {}

    impl LocalLocationProvider<TestLocalLocation> for SimpleLocalLocationProvider {
        fn get_local_location(&self, vector: Vector3<f32>) -> Option<TestLocalLocation> {
            Some(TestLocalLocation {
                x: vector.x as i32,
                y: vector.y as i32,
                z: vector.z as i32,
            })
        }

        fn find_adjacent(&self, location: &TestLocalLocation) -> Vec<(TestLocalLocation, f32)> {
            let mut adjacent = Vec::new();

            let location1 = location.add(1, 0, 0);
            let location2 = location.add(0, 1, 0);
            let location3 = location.add(1, 1, 0);
            let location4 = location.add(-1, -1, 0);
            let location5 = location.add(1, -1, 0);
            let location6 = location.add(-1, 1, 0);

            adjacent.push((location1, 1.0));
            adjacent.push((location2, 1.0));
            adjacent.push((location3, 1.0));
            adjacent.push((location4, 1.0));
            adjacent.push((location5, 1.0));
            adjacent.push((location6, 1.0));

            adjacent
        }

        fn to_global_location(&self, location: &TestLocalLocation) -> Vector3<f32> {
            Vector3::new(location.x as f32, location.y as f32, location.z as f32)
        }
    }

    #[test]
    fn test_simple_straight_path() {
        let start = Vector3::new(0.0, 0.0, 0.0);
        let goal = Vector3::new(10.0, 0.0, 0.0);

        let simple_local_location_provider = SimpleLocalLocationProvider {};

        let path = find_path(start, goal, simple_local_location_provider).unwrap();
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

        let simple_local_location_provider = SimpleLocalLocationProvider {};

        let path = find_path(start, goal, simple_local_location_provider).unwrap();
        assert_eq!(path.len(), 10);

        for i in 1..11 {
            let vec = path.get(i - 1).unwrap();

            assert_eq!(vec.x, -1.0 * i as f32);
            assert_eq!(vec.y, -1.0 * i as f32);
            assert_eq!(vec.z, 0.0);
        }
    }

    struct WalledLocalLocationProvider {}

    impl LocalLocationProvider<TestLocalLocation> for WalledLocalLocationProvider {
        fn get_local_location(&self, vector: Vector3<f32>) -> Option<TestLocalLocation> {
            Some(TestLocalLocation {
                x: vector.x as i32,
                y: vector.y as i32,
                z: vector.z as i32,
            })
        }

        fn find_adjacent(&self, location: &TestLocalLocation) -> Vec<(TestLocalLocation, f32)> {
            fn add(location: TestLocalLocation, adjacent: &mut Vec<(TestLocalLocation, f32)>) {
                if location.x == 5 {
                    if 5 > location.y.abs() {
                        return;
                    }
                }

                adjacent.push((location, 1.0));
            }

            let mut adjacent = Vec::new();

            let location1 = location.add(1, 0, 0);
            let location2 = location.add(0, 1, 0);
            let location3 = location.add(1, 1, 0);
            let location4 = location.add(-1, -1, 0);
            let location5 = location.add(1, -1, 0);
            let location6 = location.add(-1, 1, 0);

            add(location1, &mut adjacent);
            add(location2, &mut adjacent);
            add(location3, &mut adjacent);
            add(location4, &mut adjacent);
            add(location5, &mut adjacent);
            add(location6, &mut adjacent);

            adjacent
        }

        fn to_global_location(&self, location: &TestLocalLocation) -> Vector3<f32> {
            Vector3::new(location.x as f32, location.y as f32, location.z as f32)
        }
    }

    #[test]
    fn test_walled_path() {
        let start = Vector3::new(0.0, 0.0, 0.0);
        let goal = Vector3::new(10.0, 0.0, 0.0);

        let walled_local_location_provider = WalledLocalLocationProvider {};

        let path = find_path(start, goal, walled_local_location_provider).unwrap();
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

use nalgebra::Vector3;

pub trait GridProvider {
    fn is_obstacle(&self, vector: Vector3<f32>) -> bool;
}

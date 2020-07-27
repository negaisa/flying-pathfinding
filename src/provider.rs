use crate::grid::Grid;
use nalgebra::Vector3;
use std::path::Path;

pub trait GridProvider {
    fn is_obstacle(&self, vector: Vector3<f32>) -> bool;
}

pub struct FolderGridProvider<'a, ATG, GTA, GFN>
where
    ATG: Fn(f32) -> u32,
    GTA: Fn(u32) -> f32,
    GFN: Fn(u32, u32) -> String,
{
    /// Folder where grid files located.
    grid_folder_path: &'a Path,
    /// Function to convert axis to grid id.
    axis_to_grid_id_fn: ATG,
    /// Function to convert grid id to axis.
    grid_id_to_axis_fn: GTA,
    /// Function to format grid format name.
    /// 1, 1 => grid_1_1.dat
    grid_file_name_fn: GFN,
}

impl<'a, ATG, GTA, GFN> FolderGridProvider<'a, ATG, GTA, GFN>
where
    ATG: Fn(f32) -> u32,
    GTA: Fn(u32) -> f32,
    GFN: Fn(u32, u32) -> String,
{
    pub fn new(
        grid_folder_path: &'a Path,
        axis_to_grid_id_fn: ATG,
        grid_id_to_axis_fn: GTA,
        grid_file_name_fn: GFN,
    ) -> Self {
        FolderGridProvider {
            grid_folder_path,
            axis_to_grid_id_fn,
            grid_id_to_axis_fn,
            grid_file_name_fn,
        }
    }
}

impl<'a, ATG, GTA, GFN> GridProvider for FolderGridProvider<'a, ATG, GTA, GFN>
where
    ATG: Fn(f32) -> u32,
    GTA: Fn(u32) -> f32,
    GFN: Fn(u32, u32) -> String,
{
    fn is_obstacle(&self, vector: Vector3<f32>) -> bool {
        let grid_x = (self.axis_to_grid_id_fn)(vector.x);
        let grid_y = (self.axis_to_grid_id_fn)(vector.y);
        let grid_file_name = (self.grid_file_name_fn)(grid_x, grid_y);
        let grid_path = self.grid_folder_path.join(&grid_file_name);

        match Grid::import(grid_path) {
            Ok(grid) if (vector.z >= 0.0 && vector.z <= grid.height as f32) => {
                let grid_start_x = (self.grid_id_to_axis_fn)(grid_x);
                let grid_start_y = (self.grid_id_to_axis_fn)(grid_y);

                let x = (grid_start_x - vector.x).round() as u32;
                let y = (grid_start_y - vector.y).round() as u32;
                let z = vector.z.round() as u32;

                grid.is_obstacle(x, y, z)
            }
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::provider::{FolderGridProvider, GridProvider};
    use nalgebra::Vector3;
    use std::path::Path;

    #[test]
    fn test_is_obstacle() {
        let axis_to_grid_id_fn = |axis: f32| (32.0 - (axis / 533.33)).floor() as u32;
        let grid_id_to_axis_fn = |grid_id| (32.0 - grid_id as f32) * 533.3;
        let grid_file_name_fn = |x, y| format!("grid_{}_{}.dat", x, y);
        let grid_folder_path = Path::new("test/map_1718");

        let grid_provider = FolderGridProvider::new(
            grid_folder_path,
            axis_to_grid_id_fn,
            grid_id_to_axis_fn,
            grid_file_name_fn,
        );

        assert!(grid_provider.is_obstacle(Vector3::new(-1604.0, 1163.0, 111.0)));
    }
}

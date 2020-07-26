use crate::grid::Grid;
use nalgebra::Vector3;
use std::path::Path;

pub trait GridProvider {
    fn is_obstacle(&self, vector: Vector3<f32>) -> bool;
}

pub struct FolderGridProvider<'a, ATT, GFN>
where
    ATT: Fn(f32) -> u32,
    GFN: Fn(u32, u32) -> String,
{
    /// Folder where grid files located.
    grid_folder_path: &'a Path,
    /// Width of grid.
    grid_width: u32,
    /// Function to convert axis to grid id.
    axis_to_grid_id_fn: ATT,
    /// Function to format grid format name.
    /// 1, 1 => grid_1_1.dat
    grid_file_name_fn: GFN,
}

impl<'a, ATT, GFN> FolderGridProvider<'a, ATT, GFN>
where
    ATT: Fn(f32) -> u32,
    GFN: Fn(u32, u32) -> String,
{
    pub fn new(
        grid_folder_path: &'a Path,
        grid_width: u32,
        axis_to_grid_id_fn: ATT,
        grid_file_name_fn: GFN,
    ) -> Self {
        FolderGridProvider {
            grid_folder_path,
            grid_width,
            axis_to_grid_id_fn,
            grid_file_name_fn,
        }
    }
}

impl<'a, ATT, GFN> GridProvider for FolderGridProvider<'a, ATT, GFN>
where
    ATT: Fn(f32) -> u32,
    GFN: Fn(u32, u32) -> String,
{
    fn is_obstacle(&self, vector: Vector3<f32>) -> bool {
        let grid_x = (self.axis_to_grid_id_fn)(vector.x);
        let grid_y = (self.axis_to_grid_id_fn)(vector.y);
        let grid_file_name = (self.grid_file_name_fn)(grid_x, grid_y);
        let grid_path = self.grid_folder_path.join(&grid_file_name);

        match Grid::import(grid_path) {
            Ok(grid) => grid.is_obstacle(0, 0, 0),
            Err(_) => false,
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
        let grid_file_name_fn = |x, y| format!("grid_{}_{}.dat", x, y);
        let grid_folder_path = Path::new("test/map_1718");

        let grid_provider =
            FolderGridProvider::new(grid_folder_path, 534, axis_to_grid_id_fn, grid_file_name_fn);

        grid_provider.is_obstacle(Vector3::new(375.0, -13904.166016, 1000.0));
    }
}

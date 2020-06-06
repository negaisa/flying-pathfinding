use bitvec::prelude::*;
use byteorder::{BigEndian, ReadBytesExt, WriteBytesExt};
use std::fs::{File, OpenOptions};
use std::io::{Error, Read, Write};
use std::path::Path;

pub struct Grid {
    pub width: u32,
    pub height: u32,
    data: BitVec<Lsb0, u8>,
}

impl Grid {
    pub fn new(width: u32, height: u32) -> Self {
        let length = (width * width * height) as usize;
        let length_with_padding = ((length as f32 / 8.0).ceil() * 8.0) as usize;

        let mut data = BitVec::with_capacity(length_with_padding);
        data.resize(length_with_padding, false);

        Grid {
            width,
            height,
            data,
        }
    }

    pub fn import<P: AsRef<Path>>(path: P) -> Result<Grid, Error> {
        let mut file = File::open(path)?;

        let width = file.read_u32::<BigEndian>()?;
        let height = file.read_u32::<BigEndian>()?;
        let length = width * width * height;

        let bytes = (length as f32 / 8.0).ceil();
        let mut vec = vec![0u8; bytes as usize];
        file.read_exact(&mut vec)?;

        let data = BitVec::from_vec(vec);

        let grid = Grid {
            width,
            height,
            data,
        };

        Ok(grid)
    }

    pub fn set_obstacle(&mut self, x: u32, y: u32, z: u32) {
        let index = Self::index(x, y, z, self.width);

        // It's safe because we already checked the bounds.
        unsafe {
            self.data.set_unchecked(index, true);
        }
    }

    pub fn is_obstacle(&self, x: u32, y: u32, z: u32) -> bool {
        let index = Self::index(x, y, z, self.width);

        // It's safe because we already checked the bounds.
        unsafe { *self.data.get_unchecked(index) }
    }

    fn index(x: u32, y: u32, z: u32, width: u32) -> usize {
        assert!(width > x, "X-axis coordinate {} is out of bounds", x);
        assert!(width > y, "Y-axis coordinate {} is out of bounds", y);
        assert!(width > z, "Z-axis coordinate {} is out of bounds", z);

        (x + width * (y + width * z)) as usize
    }

    pub fn export<P: AsRef<Path>>(self, path: P) -> Result<(), Error> {
        let mut file = OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open(path)?;

        file.write_u32::<BigEndian>(self.width)?;
        file.write_u32::<BigEndian>(self.height)?;
        file.write_all(self.data.as_slice())?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::Grid;
    use byteorder::{BigEndian, ReadBytesExt};
    use std::io::{Cursor, Read};
    use std::path::Path;
    use tempfile::NamedTempFile;

    #[test]
    fn test_export() {
        let mut temp_file = NamedTempFile::new().unwrap();

        let mut grid = Grid::new(3, 3);
        grid.set_obstacle(1, 1, 1);
        grid.set_obstacle(2, 2, 2);
        grid.export(temp_file.path()).unwrap();

        let length = temp_file.as_file().metadata().unwrap().len();
        let mut vec = vec![0u8; length as usize];
        temp_file.read_exact(&mut vec).unwrap();

        let mut cursor = Cursor::new(&vec);

        assert_eq!(vec.len(), 12);
        assert_eq!(cursor.read_u32::<BigEndian>().unwrap(), 3);
        assert_eq!(cursor.read_u32::<BigEndian>().unwrap(), 3);
        assert_eq!(cursor.read_u8().unwrap(), 0);
        assert_eq!(cursor.read_u8().unwrap(), 0b0010_0000);
        assert_eq!(cursor.read_u8().unwrap(), 0);
        assert_eq!(cursor.read_u8().unwrap(), 0b0000_0100);
    }

    #[test]
    fn test_import() {
        let path = Path::new("test/grid.dat");
        let grid = Grid::import(path).unwrap();

        assert_obstacle(&grid);
    }

    #[test]
    fn test_obstacle() {
        let mut grid = Grid::new(3, 3);

        grid.set_obstacle(1, 1, 1);
        grid.set_obstacle(2, 2, 2);

        assert_obstacle(&grid);
    }

    fn assert_obstacle(grid: &Grid) {
        assert!(grid.is_obstacle(1, 1, 1));
        assert!(grid.is_obstacle(2, 2, 2));

        for x in 0..3 {
            for y in 0..3 {
                for z in 0..3 {
                    if x == 1 && y == 1 && z == 1 {
                        continue;
                    }

                    if x == 2 && y == 2 && z == 2 {
                        continue;
                    }

                    assert!(!grid.is_obstacle(x, y, z))
                }
            }
        }
    }

    #[test]
    fn test_obstacles2() {
        let mut grid = Grid::new(12, 2);

        for (x, y, z) in obstacles2_data() {
            grid.set_obstacle(x, y, z);
        }

        for x in 0..grid.width {
            for y in 0..grid.width {
                for z in 0..grid.height {
                    let in_data = obstacles2_data().contains(&(x, y, z));
                    let in_grid = grid.is_obstacle(x, y, z);

                    assert_eq!(
                        in_data, in_grid,
                        "Data and grid not match for x: {}, y: {}, z: {}",
                        x, y, z
                    );
                }
            }
        }
    }

    fn obstacles2_data() -> Vec<(u32, u32, u32)> {
        vec![
            (1, 7, 1),
            (1, 8, 1),
            (2, 6, 1),
            (2, 7, 1),
            (2, 8, 1),
            (3, 5, 1),
            (3, 6, 1),
            (3, 7, 1),
            (3, 8, 1),
            (4, 4, 1),
            (4, 5, 1),
            (4, 6, 1),
            (4, 7, 1),
            (4, 8, 1),
            (5, 3, 1),
            (5, 4, 1),
            (5, 5, 1),
            (5, 6, 1),
            (5, 7, 1),
            (5, 8, 1),
            (6, 3, 1),
            (6, 4, 1),
            (6, 5, 1),
            (6, 6, 1),
            (6, 7, 1),
            (6, 8, 1),
            (7, 3, 1),
            (7, 4, 1),
            (7, 5, 1),
            (7, 6, 1),
            (7, 7, 1),
            (7, 8, 1),
            (8, 4, 1),
            (8, 5, 1),
            (8, 6, 1),
            (8, 7, 1),
            (8, 8, 1),
            (9, 5, 1),
            (9, 6, 1),
            (9, 7, 1),
            (9, 8, 1),
            (10, 6, 1),
            (10, 7, 1),
            (10, 8, 1),
            (11, 7, 1),
            (11, 8, 1),
        ]
    }
}

mod csg_hierarchy;
pub mod csg_geometry;
pub mod csg_serde;
mod csg_utility;
mod csg_categorization;

pub fn add(left: usize, right: usize) -> usize {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}

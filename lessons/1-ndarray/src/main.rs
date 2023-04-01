use ndarray::prelude::*;
use std::fmt::*;

#[derive(Debug)]
struct Point {
    x: i32,
    y: i32
}

impl Display for Point {
    fn fmt(&self, f: &mut Formatter) -> Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}

fn main() {

    let point = Point{x:1, y:2};
    println!("{:?}", point);
}

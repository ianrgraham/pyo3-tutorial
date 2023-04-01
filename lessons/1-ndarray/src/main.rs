use ndarray::prelude::*;

fn main() {
    let mut x = Array1::<f32>::zeros(10);

    for i in 0..x.len() {
        x[i] = i as f32;
    }

    println!("{x}");

    let x = array![[1.0f32, 2.], [3., 4.]];

    x.rows().into_iter().for_each(|row| {
        println!("{:?}", row);
    });
}

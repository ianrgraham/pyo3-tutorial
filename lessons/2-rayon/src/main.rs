use rayon::prelude::*;
use ndarray::prelude::*;

fn main() {
    let v = vec![1, 2, 3, 4, 5];

    let out: i32 = v.par_iter().sum();

    println!("{out}");

    let mut a = Array1::<f32>::zeros(100);

    a.par_iter_mut().for_each(|x| *x = 1.0);

    println!("{a}");
}

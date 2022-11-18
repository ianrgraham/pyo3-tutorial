use pyo3::prelude::*;
use ndarray::{prelude::*, par_azip};
use numpy::*;

#[pyfunction]
fn msd<'py>(py: Python<'py>, x: PyReadonlyArray3<f32>) -> &'py PyArray1<f32> {

    let x = x.as_array();
    let x_iter = x.outer_iter();
    let mut result = Array1::<f32>::zeros(x.shape()[0]);

    par_azip!((r in &mut result, x in x_iter) {
        *r = x.mapv(|x| x.powi(2)).sum();
    });

    result.into_pyarray(py)
}

#[pymodule]
fn science_calc(_py: Python<'_>, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(msd, m)?)?;
    Ok(())
}

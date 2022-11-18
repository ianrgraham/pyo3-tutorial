import numpy as np

def msd(x: np.ndarray) -> np.ndarray:
    """Mean-squared displacement.
    
    Arguments
    ----------
    x (numpy.ndarray): 3D array of positions with shape (n_frames, n_particles, 3).
    
    Returns
    -------
    msd (numpy.ndarray): 1D array of mean-squared displacements with shape (n_frames)."""
    ...
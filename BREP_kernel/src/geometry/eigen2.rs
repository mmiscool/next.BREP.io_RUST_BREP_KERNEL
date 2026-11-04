//! Symmetric two-dimensional eigen helpers.

pub(crate) fn eigenvalues(matrix: [[f64; 2]; 2]) -> (f64, f64) {
    let trace = matrix[0][0] + matrix[1][1];
    let determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    let discriminant = (trace * trace / 4.0 - determinant).max(0.0).sqrt();
    (trace / 2.0 + discriminant, trace / 2.0 - discriminant)
}

pub(crate) fn spectral_norm(matrix: [[f64; 2]; 2]) -> f64 {
    let (first, second) = eigenvalues(matrix);
    first.abs().max(second.abs())
}

pub(crate) fn eigenvector(matrix: [[f64; 2]; 2], lambda: f64) -> [f64; 2] {
    let candidates = [
        [matrix[0][1], lambda - matrix[0][0]],
        [lambda - matrix[1][1], matrix[0][1]],
    ];
    let mut best = [1.0, 0.0];
    let mut best_length = 0.0;
    for candidate in candidates {
        let length = (candidate[0] * candidate[0] + candidate[1] * candidate[1]).sqrt();
        if length > best_length {
            best_length = length;
            best = [candidate[0] / length, candidate[1] / length];
        }
    }
    if best_length == 0.0 {
        // The matrix is a multiple of the identity: every direction is an
        // eigenvector, and none is a null direction unless the multiple is 0.
        return [1.0, 0.0];
    }
    best
}

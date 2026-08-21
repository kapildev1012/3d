/**
 * Small dependency-free dense linear-algebra helpers used by the Drive
 * controller ports. Matrices are row-major JavaScript arrays.
 */

export const EPSILON = 1e-10;

export function zeros(rows, columns = rows) {
  return Array.from({ length: rows }, () => new Array(columns).fill(0));
}

export function identity(size) {
  const result = zeros(size);
  for (let i = 0; i < size; i++) result[i][i] = 1;
  return result;
}

export function diagonal(values) {
  const result = zeros(values.length);
  for (let i = 0; i < values.length; i++) result[i][i] = values[i];
  return result;
}

export function transpose(matrix) {
  if (!matrix.length) return [];
  return matrix[0].map((_, column) => matrix.map(row => row[column]));
}

export function add(left, right) {
  return left.map((row, i) => row.map((value, j) => value + right[i][j]));
}

export function subtract(left, right) {
  return left.map((row, i) => row.map((value, j) => value - right[i][j]));
}

export function scale(matrix, scalar) {
  return matrix.map(row => row.map(value => value * scalar));
}

export function addDiagonal(matrix, amount) {
  const result = matrix.map(row => row.slice());
  for (let i = 0; i < Math.min(result.length, result[0]?.length || 0); i++) {
    result[i][i] += amount;
  }
  return result;
}

export function multiply(left, right) {
  if (!left.length || !right.length) return [];
  const rows = left.length;
  const inner = right.length;
  const columns = right[0].length;
  const result = zeros(rows, columns);
  for (let i = 0; i < rows; i++) {
    for (let k = 0; k < inner; k++) {
      const value = left[i][k];
      if (Math.abs(value) < EPSILON) continue;
      for (let j = 0; j < columns; j++) result[i][j] += value * right[k][j];
    }
  }
  return result;
}

export function matVec(matrix, vector) {
  return matrix.map(row => row.reduce((sum, value, column) => sum + value * vector[column], 0));
}

export function vecAdd(left, right) {
  return left.map((value, i) => value + right[i]);
}

export function vecSubtract(left, right) {
  return left.map((value, i) => value - right[i]);
}

export function vecScale(vector, scalar) {
  return vector.map(value => value * scalar);
}

export function dot(left, right) {
  return left.reduce((sum, value, i) => sum + value * right[i], 0);
}

export function norm(vector) {
  return Math.sqrt(dot(vector, vector));
}

export function outer(left, right) {
  return left.map(a => right.map(b => a * b));
}

export function quadraticForm(vector, matrix) {
  return dot(vector, matVec(matrix, vector));
}

export function symmetrize(matrix) {
  return scale(add(matrix, transpose(matrix)), 0.5);
}

export function blockMatrix(topLeft, topRight, bottomLeft, bottomRight) {
  const top = topLeft.map((row, i) => row.concat(topRight[i]));
  const bottom = bottomLeft.map((row, i) => row.concat(bottomRight[i]));
  return top.concat(bottom);
}

export function blockDiagonal(...blocks) {
  const rows = blocks.reduce((sum, block) => sum + block.length, 0);
  const columns = blocks.reduce((sum, block) => sum + (block[0]?.length || 0), 0);
  const result = zeros(rows, columns);
  let rowOffset = 0;
  let columnOffset = 0;
  for (const block of blocks) {
    for (let i = 0; i < block.length; i++) {
      for (let j = 0; j < (block[0]?.length || 0); j++) {
        result[rowOffset + i][columnOffset + j] = block[i][j];
      }
    }
    rowOffset += block.length;
    columnOffset += block[0]?.length || 0;
  }
  return result;
}

/** Solve A X = B with partial-pivot Gaussian elimination. */
export function solve(matrix, rightHandSide, regularization = 1e-9) {
  const size = matrix.length;
  if (!size || matrix.some(row => row.length !== size)) throw new Error('solve requires a square matrix');
  const vectorInput = !Array.isArray(rightHandSide[0]);
  const rhs = vectorInput ? rightHandSide.map(value => [value]) : rightHandSide.map(row => row.slice());
  if (rhs.length !== size) throw new Error('solve dimension mismatch');
  const width = rhs[0]?.length || 0;
  const augmented = matrix.map((row, i) => row.slice().concat(rhs[i]));

  for (let pivot = 0; pivot < size; pivot++) {
    let best = pivot;
    for (let row = pivot + 1; row < size; row++) {
      if (Math.abs(augmented[row][pivot]) > Math.abs(augmented[best][pivot])) best = row;
    }
    if (best !== pivot) [augmented[pivot], augmented[best]] = [augmented[best], augmented[pivot]];
    if (Math.abs(augmented[pivot][pivot]) < regularization) {
      augmented[pivot][pivot] += augmented[pivot][pivot] < 0 ? -regularization : regularization;
    }
    const divisor = augmented[pivot][pivot];
    for (let column = pivot; column < size + width; column++) augmented[pivot][column] /= divisor;
    for (let row = 0; row < size; row++) {
      if (row === pivot) continue;
      const factor = augmented[row][pivot];
      if (Math.abs(factor) < EPSILON) continue;
      for (let column = pivot; column < size + width; column++) {
        augmented[row][column] -= factor * augmented[pivot][column];
      }
    }
  }

  const answer = augmented.map(row => row.slice(size));
  return vectorInput ? answer.map(row => row[0]) : answer;
}

export function inverse(matrix) {
  return solve(matrix, identity(matrix.length));
}

export function finiteDifferenceJacobian(fn, point, epsilon = 1e-5) {
  const baseline = fn(point);
  const result = zeros(baseline.length, point.length);
  for (let column = 0; column < point.length; column++) {
    const step = epsilon * Math.max(1, Math.abs(point[column]));
    const plus = point.slice();
    const minus = point.slice();
    plus[column] += step;
    minus[column] -= step;
    const high = fn(plus);
    const low = fn(minus);
    for (let row = 0; row < baseline.length; row++) result[row][column] = (high[row] - low[row]) / (2 * step);
  }
  return result;
}

export function clamp(value, minimum, maximum) {
  return Math.max(minimum, Math.min(maximum, value));
}

export function clampVector(vector, minimum, maximum) {
  return vector.map((value, i) => clamp(
    value,
    Array.isArray(minimum) ? minimum[i] : minimum,
    Array.isArray(maximum) ? maximum[i] : maximum
  ));
}

/** Finite-horizon discrete Riccati recursion used by the Drive LQR class. */
export function finiteHorizonRiccati(A, B, Q, R, horizon, terminalQ = Q) {
  const BT = transpose(B);
  const AT = transpose(A);
  const P = new Array(horizon + 1);
  const K = new Array(horizon);
  P[horizon] = terminalQ.map(row => row.slice());
  for (let k = horizon - 1; k >= 0; k--) {
    const PB = multiply(P[k + 1], B);
    const PA = multiply(P[k + 1], A);
    const S = add(R, multiply(BT, PB));
    K[k] = solve(addDiagonal(symmetrize(S), 1e-8), multiply(BT, PA));
    P[k] = symmetrize(add(Q, subtract(multiply(AT, PA), multiply(multiply(AT, PB), K[k]))));
  }
  return { P, K };
}

/** Bilinear (trapezoidal/Tustin) discretization used by the QP-MPC port. */
export function trapezoidalDiscretize(Acontinuous, Bcontinuous, dt) {
  const I = identity(Acontinuous.length);
  const left = subtract(I, scale(Acontinuous, dt / 2));
  const right = add(I, scale(Acontinuous, dt / 2));
  return {
    A: solve(left, right),
    B: solve(left, scale(Bcontinuous, dt))
  };
}

/** Jacobi rotations for the eigenvalues of a real symmetric matrix. */
export function symmetricEigenvalues(matrix, tolerance = 1e-10, maxIterations = 100) {
  const work = symmetrize(matrix);
  const size = work.length;
  for (let iteration = 0; iteration < maxIterations * Math.max(1, size); iteration++) {
    let p = 0;
    let q = 0;
    let largest = 0;
    for (let i = 0; i < size; i++) {
      for (let j = i + 1; j < size; j++) {
        if (Math.abs(work[i][j]) > largest) {
          largest = Math.abs(work[i][j]);
          p = i;
          q = j;
        }
      }
    }
    if (largest < tolerance) break;
    const angle = 0.5 * Math.atan2(2 * work[p][q], work[q][q] - work[p][p]);
    const cosine = Math.cos(angle);
    const sine = Math.sin(angle);
    const app = cosine * cosine * work[p][p] - 2 * sine * cosine * work[p][q] + sine * sine * work[q][q];
    const aqq = sine * sine * work[p][p] + 2 * sine * cosine * work[p][q] + cosine * cosine * work[q][q];
    for (let k = 0; k < size; k++) {
      if (k === p || k === q) continue;
      const akp = work[k][p];
      const akq = work[k][q];
      work[k][p] = work[p][k] = cosine * akp - sine * akq;
      work[k][q] = work[q][k] = sine * akp + cosine * akq;
    }
    work[p][p] = app;
    work[q][q] = aqq;
    work[p][q] = work[q][p] = 0;
  }
  return work.map((row, i) => row[i]).sort((a, b) => a - b);
}

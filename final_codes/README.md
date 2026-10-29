# Final Codes

Static FEM of a planar tensegrity (6 nodes, 3 struts, 6 springs) and
particle-filter identification of its parameters.

| File | Purpose |
| --- | --- |
| `original/` | the three files exactly as received (`Final Codes.rar`) |
| `MainFile.m` | particle filter: 100 load cases, 100 particles, true factors [0.8 1.2 0.7 1.2 0.8] |
| `forward_tensegrity.m` | form finding + loading with Newton iterations; returns output 1/0 and the node positions |
| `forward_tensegrity_datasim.m` | synthetic measurements at the true parameters |
| `f_resample_residual.m`, `f_resample_indices.m` | residual resampling (moved out of `MainFile.m`) |

The parameters are factors: `del_s1..del_s3` scale the strut rest lengths
(1, 0.4, 0.5 mm) and `del_f1`, `del_f2` scale the stiffness of spring types 1
and 2 (0.2 and 0.3 N/mm).

Run in MATLAB (`MainFile`) or GNU Octave (`npm run final:octave`, about 30 min).
`FINAL_EXP_NO=10 FINAL_PARTICLES=20` makes a quick run. The figure and the
estimates are saved here as `parameter_estimation.png` and `.mat`.

Changes from `original/` (the algorithm is unchanged):

1. Stability is checked with all eigenvalues of the reduced tangent stiffness.
   The original `eigs(K,1,'smallestabs') > 0` checks only the one closest to
   zero, and `'smallestabs'` does not exist in Octave.
2. A failed solve returns NaN positions instead of leaving the output
   unassigned, which stopped `MainFile.m` with an error.
3. The helper functions are separate files, and `plot` replaces `yline`, so the
   code runs in Octave.
4. The labels describe the parameters correctly. The plot and the `.mat` file
   are saved.

The JavaScript port (`js/paramIdentification.js`), its validation against
`original/` and the rover application are described in the main README under
"Parameter identification".

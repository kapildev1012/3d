# Controller league (cable-only, 132 runs, 20 s each, solid obstacles on)

| Rank | Controller | Runs | Success (%) | Progress (m) | v̄ (m/s) | RMSE (m) | CoT | ∫uᵀu | t_solve (ms) | max rod err (mm) | max \|ε\| (J) | assist |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | MPC + RL + LLM | 12 | 100 | 2.23 | 0.111 | 0.304 | 13.9 | 0.667 | 6.08 | 0.0388 | 5.0e+0 | 0 |
| 2 | MPC + RL | 12 | 100 | 2.23 | 0.111 | 0.304 | 13.9 | 0.667 | 6.18 | 0.0388 | 5.0e+0 | 0 |
| 3 | MPC + GNSS | 12 | 100 | 1.57 | 0.079 | 0.352 | 11.8 | 0.299 | 1.76 | 0.0259 | 2.4e+0 | 0 |
| 4 | SGT | 12 | 100 | 1.36 | 0.068 | 0.254 | 13.5 | 0.321 | 1.04 | 0.0210 | 2.0e-1 | 0 |
| 5 | SGT + shield | 12 | 83 | 1.39 | 0.070 | 0.287 | 17.3 | 0.386 | 5.45 | 0.2187 | 3.5e-1 | 0 |
| 6 | MPC + GPS | 12 | 83 | 1.18 | 0.059 | 0.195 | 8.5 | 0.264 | 0.84 | 0.0125 | 1.1e-1 | 0 |
| 7 | MPC + GPS + shield | 12 | 50 | 0.79 | 0.039 | 0.197 | 32.7 | 0.500 | 5.04 | 0.0000 | 1.0e-9 | 0 |
| 8 | QP-MPC | 12 | 0 | 0.06 | 0.003 | 0.086 | 144.0 | 1.831 | 0.94 | 0.0000 | 1.0e-9 | 0 |
| 9 | iLQR | 12 | 0 | 0.06 | 0.003 | 0.088 | 146.2 | 1.893 | 0.52 | 0.0000 | 1.0e-9 | 0 |
| 10 | Heuristic | 12 | 0 | 0.04 | 0.002 | 0.091 | 140.8 | 2.083 | 0.44 | 0.0000 | 1.0e-9 | 0 |
| 11 | LQR | 12 | 0 | 0.03 | 0.001 | 0.133 | 160.2 | 2.573 | 0.42 | 0.0000 | 1.0e-9 | 0 |

Ranking: success rate (advanced ≥ 1 m), then mean progress. RMSE alone favours controllers that barely move.

## Per cell: furthest progress

| Cell | Winner | Progress (m) | MPC + RL | MPC + RL + LLM | Best non-RL | its progress |
|---|---|---|---|---|---|---|
| L1 delay | MPC + RL + LLM | 2.22 | 2.22 | 2.22 | SGT + shield | 1.99 |
| L1 friction | MPC + RL + LLM | 2.04 | 2.04 | 2.04 | MPC + GNSS | 1.91 |
| L1 gaussian | MPC + RL + LLM | 1.92 | 1.92 | 1.92 | MPC + GNSS | 1.65 |
| L1 nominal | MPC + RL + LLM | 4.23 | 4.23 | 4.23 | MPC + GPS | 1.85 |
| L1 payload | MPC + RL + LLM | 1.84 | 1.84 | 1.84 | SGT + shield | 1.52 |
| L1 push | MPC + RL + LLM | 2.40 | 2.40 | 2.40 | MPC + GNSS | 2.22 |
| L4 delay | MPC + RL + LLM | 1.87 | 1.87 | 1.87 | MPC + GNSS | 1.58 |
| L4 friction | MPC + RL + LLM | 2.07 | 2.07 | 2.07 | SGT + shield | 1.63 |
| L4 gaussian | MPC + GNSS | 1.77 | 1.17 | 1.17 | MPC + GNSS | 1.77 |
| L4 nominal | MPC + RL + LLM | 2.62 | 2.62 | 2.62 | MPC + GPS + shield | 1.29 |
| L4 payload | MPC + RL + LLM | 2.24 | 2.24 | 2.24 | MPC + GNSS | 1.43 |
| L4 push | MPC + RL + LLM | 2.09 | 2.09 | 2.09 | MPC + GNSS | 2.07 |

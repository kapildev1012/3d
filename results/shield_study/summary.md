# Tension-certified safety filter study

10 conditions (L1-s1, L1-s2, L1-s3, L3-s1, L3-s2, push, impulse, payload+30%, delay 100ms, k-20%), 20 s cable-only runs, mean ± std over conditions; "better in" counts paired conditions where the filter improved the metric.

## SGT

| Metric | Filter off | Filter on | Better in |
|---|---|---|---|
| Progress (m) | 1.486 ± 0.391 | 2.202 ± 0.637 | 8/10 |
| Payload-cable slack time (%) | 35.129 ± 2.948 | 2.729 ± 0.631 | 10/10 |
| Mean payload-cable tension (N) | 6.173 ± 0.499 | 17.963 ± 1.036 | 10/10 |
| Min payload–rod clearance (m) | 0.102 ± 0.018 | 0.090 ± 0.035 | 5/10 |
| Payload rattle RMS (mm) | 18.571 ± 1.004 | 14.992 ± 2.519 | 9/10 |
| Payload accel. 99th pct (g) | 2.503 ± 1.079 | 3.177 ± 1.698 | 4/10 |
| Max outer-cable tension (N) | 137.045 ± 80.784 | 179.367 ± 79.189 | 3/10 |
| Aborted tip cycles | 1.100 ± 1.101 | 0.000 ± 0.000 | 6/10 |
| Controller time (ms/tick) | 0.636 ± 0.114 | 3.597 ± 0.213 | 0/10 |

## Learned policy

| Metric | Filter off | Filter on | Better in |
|---|---|---|---|
| Progress (m) | 1.315 ± 0.644 | 1.292 ± 0.539 | 4/10 |
| Payload-cable slack time (%) | 33.629 ± 2.402 | 1.622 ± 0.576 | 10/10 |
| Mean payload-cable tension (N) | 6.915 ± 0.577 | 17.512 ± 0.756 | 10/10 |
| Min payload–rod clearance (m) | 0.105 ± 0.014 | 0.101 ± 0.033 | 4/10 |
| Payload rattle RMS (mm) | 22.379 ± 0.978 | 27.454 ± 4.587 | 3/10 |
| Payload accel. 99th pct (g) | 2.036 ± 1.121 | 1.712 ± 0.664 | 5/10 |
| Max outer-cable tension (N) | 121.791 ± 64.564 | 105.069 ± 51.506 | 7/10 |
| Aborted tip cycles | 1.500 ± 1.354 | 1.700 ± 1.418 | 3/10 |
| Controller time (ms/tick) | 0.785 ± 0.119 | 3.640 ± 0.228 | 0/10 |

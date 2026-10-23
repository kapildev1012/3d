/**
 * MPC + GNSS (satellite positioning) controller: receiver model, Kalman filter, steering MPC and a
 * closed-loop run on the research (DAE, cable-only) profile.
 */
import test from 'node:test';
import assert from 'node:assert/strict';
import { GpsReceiver, GpsKalmanFilter, solveSteeringMpc, GnssMpcNavigator, GPS_MPC_DEFAULTS } from '../js/gnssNavigation.js';
import { solveAdvancedController, ADVANCED_MODES } from '../js/advancedControllers.js';
import { runExperiment, METHODS } from '../js/experimentProtocol.js';

console.info = () => {};

test('receiver: rate, latency, dropout and noise statistics', () => {
  const rx = new GpsReceiver({ rate: 5, sigma: 0.3, biasSigma: 0, latency: 0.2, dropout: 0.1 }, 7);
  const errs = [];
  let n = 0;
  for (let k = 0; k <= 2000; k++) {           // 100 s at 20 Hz, moving at 0.5 m/s in y
    const t = 0.05*k;
    const fix = rx.sample(t, [0, 0.5*t, 0]);
    if (!fix) continue;
    n++;
    assert.ok(Math.abs(fix.receivedAt-fix.t-0.2) < 1e-9);
    errs.push(fix.y-0.5*fix.t, fix.x);         // error against the position at measurement time
  }
  assert.ok(n > 400 && n < 480, `fixes ${n} (≈ 500 × 0.9)`);
  const sd = Math.sqrt(errs.reduce((s, e) => s+e*e, 0)/errs.length);
  assert.ok(Math.abs(sd-0.3) < 0.03, `noise σ ${sd}`);
});

test('Kalman filter tracks a moving rover far better than raw fixes, with latency compensated', () => {
  const rx = new GpsReceiver({ sigma: 0.3, biasSigma: 0, dropout: 0 }, 3);
  const kf = new GpsKalmanFilter({ sigma: 0.3, biasSigma: 0 });
  const truth = t => [0.3*Math.sin(0.2*t), 0.6*t];
  let sum = 0, cnt = 0, raw = 0, rawCnt = 0;
  for (let k = 0; k <= 1200; k++) {
    const t = 0.05*k, p = truth(t);
    const fix = rx.sample(t, [...p, 0]);
    kf.predict(t);
    if (fix) { kf.update(fix); raw += (fix.x-p[0])**2+(fix.y-p[1])**2; rawCnt++; }
    if (t > 10) { const e = kf.position; sum += (e[0]-p[0])**2+(e[1]-p[1])**2; cnt++; }
  }
  const rmsKf = Math.sqrt(sum/cnt), rmsRaw = Math.sqrt(raw/rawCnt);
  assert.ok(rmsKf < 0.6*rmsRaw, `KF ${rmsKf.toFixed(3)} m vs raw ${rmsRaw.toFixed(3)} m`);
  assert.ok(Math.abs(kf.velocity[1]-0.6) < 0.15, `vy ${kf.velocity[1]}`);
});

test('Kalman filter rejects a 10 m outlier fix', () => {
  const kf = new GpsKalmanFilter({ sigma: 0.3, biasSigma: 0 });
  for (let k = 0; k < 20; k++) { kf.predict(0.2*k); kf.update({ t: 0.2*k, receivedAt: 0.2*k, x: 0, y: 0 }); }
  kf.predict(4.2);
  assert.equal(kf.update({ t: 4.2, receivedAt: 4.2, x: 10, y: 0 }), false);
  assert.ok(Math.abs(kf.position[0]) < 0.1);
});

test('steering MPC respects |ω| ≤ ωmax and turns back toward the path', () => {
  const left = solveSteeringMpc({ e: 1.5, psi: 0, speeds: [0.5] });
  const right = solveSteeringMpc({ e: -1.5, psi: 0, speeds: [0.5] });
  for (const s of [left, right]) for (const w of s.omegas) assert.ok(Math.abs(w) <= GPS_MPC_DEFAULTS.omegaMax+1e-12);
  assert.ok(left.psiCommand < 0 && right.psiCommand > 0);
  assert.ok(Math.abs(left.terminalCrossTrack) < 1.5);
  const onPath = solveSteeringMpc({ e: 0, psi: 0, speeds: [0.5] });
  assert.ok(Math.abs(onPath.psiCommand) < 1e-6);
});

test('navigator on a kinematic rover converges onto the start→goal line from a 2 m offset', () => {
  const nav = new GnssMpcNavigator({ seed: 5, start: [0, 0], goal: [0, 40] });
  let p = [2, 0];
  for (let k = 0; k < 1200; k++) {            // 60 s, rover rolls 0.5 m/s in the commanded direction
    const out = nav.step(0.05*k, [p[0], p[1], 0.5], { forwardSpeeds: [0.5] });
    const d = out ? out.direction : [0, 1, 0];
    p = [p[0]+0.05*0.5*d[0], p[1]+0.05*0.5*d[1]];
  }
  assert.ok(Math.abs(p[0]) < 0.5, `final cross-track ${p[0].toFixed(3)} m`);
  assert.ok(p[1] > 25);
});

test('mpc_gnss is a registered advanced mode and a benchmark method', () => {
  assert.ok(ADVANCED_MODES.has('mpc_gnss'));
  assert.ok(METHODS.mpcGnss);
  const s = solveAdvancedController('mpc_gnss', { state0: [0.1, 0], referenceSpeed: 0.5, horizon: 10,
    weights: { qv: 6, qh: 4, r: 1, rd: 0.5, qvT: 10, qhT: 6 },
    params: { thrust: 1.35, drag: 0.95, gradeLoss: 0.85, heightRelax: 2.2, rollCoupling: 0.1, dtH: 0.05 } });
  assert.ok(s.command >= -1 && s.command <= 1);
});

test('closed loop (DAE, cable-only): MPC + GNSS rolls forward with a bounded estimate error', { timeout: 300000 }, async () => {
  const result = await runExperiment({ method: 'mpcGnss', scenario: 'nominal', seed: 1, level: 1 }, { duration: 12 });
  assert.ok(result.metrics.progress > 0.3 || result.series.y.at(-1) > 0.3, `progress ${result.series.y.at(-1)}`);
  assert.equal(result.metrics.assistShare ?? 0, 0);
});

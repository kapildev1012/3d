/**
 * PARALLELWORKER.JS - worker entry for js/parallelPool.js.
 *
 * Runs in a Node `worker_threads` Worker or a browser module Web Worker. After
 * receiving the SharedArrayBuffers (workerData / first message) it parks in
 * Atomics.wait and executes commands written by the coordinator; there is no
 * message traffic on the hot path.
 */
import { CMD, sharedViews, DIGEST_DIM, DIGEST_FIELDS, SPEC_BYTES, ERR_BYTES } from './parallelPool.js';
import { ResidualEnv } from './residualEnv.js';
import { OBS_DIM, ACT_DIM, TEL, TEL_DIM, PARAM_IDX } from './residualMppi.js';
import { encodeState, decodeState } from './simSnapshot.js';

const inNode = typeof process !== 'undefined' && Boolean(process.versions?.node) && typeof window === 'undefined';

async function main(boot) {
  console.info = () => {};
  const { id, shared } = boot;
  const V = sharedViews(shared);
  const env = new ResidualEnv(V.params, V.weights);
  let loadedVersion = -1, checkpoint = null;

  const writeObs = () => { env.observe(V.obs.subarray(id*OBS_DIM, (id+1)*OBS_DIM)); };
  const publishTick = () => {
    V.tel.set(env.tel, id*TEL_DIM);
    V.decFlag[id] = env.called ? 1 : 0;
    if (env.called) { V.decObs.set(env.lastObs, id*OBS_DIM); V.decAct.set(env.lastDu, id*ACT_DIM); }
    writeObs();
  };
  const loadCheckpoint = () => {
    const version = Atomics.load(V.ckpt, 0);
    if (version !== loadedVersion) {
      const length = Atomics.load(V.ckpt, 1);
      checkpoint = decodeState(V.state.slice(0, length));
      loadedVersion = version;
    }
    env.restore(checkpoint);
  };

  const run = code => {
    switch (code) {
      case CMD.RESET: {
        const len = Atomics.load(V.specLen, id);
        const spec = JSON.parse(new TextDecoder().decode(V.spec.slice(id*SPEC_BYTES, id*SPEC_BYTES+len)));
        env.reset(spec);
        V.tel.set(env.tel, id*TEL_DIM);
        writeObs();
        break;
      }
      case CMD.STEP:
        env.tick(V.act.subarray(id*ACT_DIM, (id+1)*ACT_DIM));
        publishTick();
        break;
      case CMD.SAVE: env.save(); break;
      case CMD.RESTORE: env.restore(); writeObs(); break;
      case CMD.PUBLISH: {
        const bytes = encodeState(env.exportState());
        if (bytes.length > V.state.length) throw new Error(`checkpoint ${bytes.length} B exceeds the shared slot`);
        V.state.set(bytes);
        Atomics.store(V.ckpt, 1, bytes.length);
        Atomics.add(V.ckpt, 0, 1);
        break;
      }
      case CMD.BRANCH: loadCheckpoint(); writeObs(); break;
      case CMD.ROLLOUT: {
        const cand = Atomics.load(V.arg, id*4), H = Atomics.load(V.arg, id*4+1);
        loadCheckpoint();
        const gamma = V.params[PARAM_IDX.gamma];
        let R = 0, discount = 1, over = 0, maxOuter = 0, minClear = Infinity, steps = 0;
        const sum = new Float64Array(TEL_DIM);
        for (let t = 0; t < H; t++) {
          const base = (cand*shared.maxHorizon+t)*ACT_DIM;                // noise[K][Hmax][24]
          env.tick(V.noise.subarray(base, base+ACT_DIM));
          R += discount*env.reward();
          discount *= gamma;
          for (let k = 0; k < TEL_DIM; k++) sum[k] += env.tel[k];
          over += env.tel[TEL.over];
          minClear = Math.min(minClear, env.tel[TEL.clearance]);
          steps++;
          if (env.tel[TEL.done]) break;
        }
        V.ret[cand] = R;
        env.observe(V.termObs.subarray(cand*OBS_DIM, (cand+1)*OBS_DIM));
        for (let k = 0; k < TEL_DIM; k++) V.candTel[cand*TEL_DIM+k] = sum[k]/Math.max(1, steps);
        V.candInfo[cand*4] = steps; V.candInfo[cand*4+1] = over; V.candInfo[cand*4+2] = env.maxOuterTension || 0; V.candInfo[cand*4+3] = minClear;
        break;
      }
      case CMD.EPISODE: {
        const duration = Atomics.load(V.arg, id*4+2)/1000;
        const d = env.runEpisode(duration);
        DIGEST_FIELDS.forEach((name, i) => { V.digest[id*DIGEST_DIM+i] = d[name]; });
        break;
      }
      default: throw new Error(`unknown command ${code}`);
    }
  };

  // Handshake: tell the coordinator the simulator module is loaded.
  Atomics.store(V.ctrl, id, -1);
  Atomics.notify(V.ctrl, id);
  for (;;) {
    Atomics.wait(V.ctrl, id, 0);                       // sleeps while ctrl[id] === 0
    const code = Atomics.load(V.ctrl, id);
    if (code <= 0) { if (code === -1) { Atomics.wait(V.ctrl, id, -1, 50); } continue; }
    if (code === CMD.EXIT) { Atomics.store(V.ctrl, id, 0); Atomics.notify(V.ctrl, id); return; }
    try { run(code); } catch (error) {
      const text = new TextEncoder().encode(String(error?.stack || error).slice(0, ERR_BYTES-1));
      V.errText.fill(0, id*ERR_BYTES, (id+1)*ERR_BYTES);
      V.errText.set(text, id*ERR_BYTES);
      Atomics.store(V.err, id, 1);
    }
    Atomics.store(V.ctrl, id, 0);
    Atomics.notify(V.ctrl, id);
  }
}

if (inNode) {
  const wt = await import('node:worker_threads');
  if (!wt.isMainThread) { await main(wt.workerData); wt.parentPort?.close(); }
} else if (typeof self !== 'undefined' && typeof WorkerGlobalScope !== 'undefined' && self instanceof WorkerGlobalScope) {
  self.onmessage = event => { self.onmessage = null; main(event.data).then(() => self.close()); };
}

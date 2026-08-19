import numpy as np, json
from PIL import Image
from scipy import ndimage

XLIM = (-50.0, 50.0)
YLIM = (-20.0, 140.0)
names = ['fig5a', 'fig5b', 'fig5c', 'fig5d', 'fig6a', 'fig6b', 'fig6c', 'fig6d']
NAG = 12


def runs(m):
    idx = np.where(m)[0]
    out = []
    if len(idx) == 0:
        return out
    s = p = idx[0]
    for i in idx[1:]:
        if i != p + 1:
            out.append((s, p)); s = i
        p = i
    out.append((s, p))
    return out


def disk(r):
    y, x = np.mgrid[-r:r + 1, -r:r + 1]
    return (x * x + y * y <= r * r)


out = {}
for nm in names:
    im = Image.open('figs_extracted/%s.png' % nm).convert('RGB')
    a = np.asarray(im).astype(int)
    H, W, _ = a.shape
    R, G, B = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    grayish = (abs(R - G) < 40) & (abs(G - B) < 40) & (R < 200)
    axrow = int(np.argmax(grayish.sum(axis=1)))
    axcol = int(np.argmax(grayish.sum(axis=0)))
    y_top, y_bot = max(runs(grayish[:, axcol]), key=lambda t: t[1] - t[0])
    x_left, x_right = max(runs(grayish[axrow, :]), key=lambda t: t[1] - t[0])
    sx = (XLIM[1] - XLIM[0]) / (x_right - x_left)
    sy = (YLIM[1] - YLIM[0]) / (y_bot - y_top)

    black = (R < 90) & (G < 90) & (B < 90)
    black[:, :x_left + 4] = False
    black[:, x_right - 3:] = False
    black[:y_top + 3, :] = False
    black[y_bot - 3:, :] = False
    closed = ndimage.binary_closing(black, structure=disk(5))
    # marker radius from typical full-marker area (~270 px)
    rm = 9.2
    K = disk(int(round(rm))).astype(float)
    K /= K.sum()
    score = ndimage.correlate(closed.astype(float), K, mode='constant')
    sc = score.copy()
    pts = []
    for _ in range(NAG):
        idx = np.unravel_index(np.argmax(sc), sc.shape)
        val = sc[idx]
        if val < 0.55:
            break
        cy, cx = idx
        # refine centroid within a small window of the closed mask
        r = int(round(rm))
        y0, y1 = max(0, cy - r), min(H, cy + r + 1)
        x0, x1 = max(0, cx - r), min(W, cx + r + 1)
        sub = closed[y0:y1, x0:x1]
        ys, xs = np.where(sub)
        if len(xs) > 0:
            cy = y0 + ys.mean()
            cx = x0 + xs.mean()
        pts.append((XLIM[0] + (cx - x_left) * sx, YLIM[1] - (cy - y_top) * sy, val))
        yy, xx = np.mgrid[0:H, 0:W]
        sc[(xx - idx[1]) ** 2 + (yy - idx[0]) ** 2 <= (1.35 * rm) ** 2] = -1
    pts.sort(key=lambda t: (-t[1], t[0]))
    print('=====', nm, len(pts), 'markers')
    for px, py, v in pts:
        print('   (%8.2f,%8.2f) score=%.2f' % (px, py, v))
    ys = [p[1] for p in pts]
    print('   centroid y = %.2f   (n=%d)' % (np.mean(ys), len(ys)))
    out[nm] = {'pts': [[round(p[0], 2), round(p[1], 2)] for p in pts],
               'centroid': [round(float(np.mean([p[0] for p in pts])), 2),
                            round(float(np.mean(ys)), 2)]}

with open('figs_extracted/paper_positions.json', 'w') as f:
    json.dump(out, f, indent=1)

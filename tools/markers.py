import numpy as np, json
from PIL import Image
from scipy import ndimage

XLIM = (-50.0, 50.0)
YLIM = (-20.0, 140.0)
names = ['fig5a', 'fig5b', 'fig5c', 'fig5d', 'fig6a', 'fig6b', 'fig6c', 'fig6d']


def runs(mask1d):
    idx = np.where(mask1d)[0]
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


allres = {}
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
    lab, n = ndimage.label(black)
    pts = []
    for k in range(1, n + 1):
        ys, xs = np.where(lab == k)
        area = len(xs)
        w = xs.max() - xs.min() + 1
        h = ys.max() - ys.min() + 1
        if area < 25 or w > 30 or h > 30:
            continue
        fill = area / float(w * h)
        if fill < 0.55:
            continue
        cx, cy = xs.mean(), ys.mean()
        pts.append((XLIM[0] + (cx - x_left) * sx, YLIM[1] - (cy - y_top) * sy, area))
    pts.sort(key=lambda t: (-t[1], t[0]))
    print('=====', nm, 'markers found:', len(pts))
    for px, py, ar in pts:
        print('    (%8.2f, %8.2f)  area=%d' % (px, py, ar))
    allres[nm] = [[round(px, 2), round(py, 2)] for px, py, ar in pts]

with open('figs_extracted/agent_positions.json', 'w') as f:
    json.dump(allres, f, indent=1)

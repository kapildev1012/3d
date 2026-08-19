import numpy as np
from PIL import Image
from scipy import ndimage

XLIM = (-50.0, 50.0)
YLIM = (-20.0, 140.0)


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


for nm in ['fig5b', 'fig5d', 'fig6b', 'fig6d']:
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
    # translucent pink: light red-ish (high R, medium G/B, G~B)
    pink = (R > 235) & (G > 140) & (G < 235) & (abs(G - B) < 25)
    lab, n = ndimage.label(pink)
    print('=====', nm, ' pink blobs:', n, ' (sx=%.4f)' % sx)
    sizes = ndimage.sum(pink, lab, range(1, n + 1))
    order = np.argsort(sizes)[::-1]
    for k in order[:8]:
        area = sizes[k]
        if area < 200:
            continue
        ys, xs = np.where(lab == (k + 1))
        cx, cy = xs.mean(), ys.mean()
        dx = XLIM[0] + (cx - x_left) * sx
        dy = YLIM[1] - (cy - y_top) * sy
        wpx = xs.max() - xs.min() + 1
        hpx = ys.max() - ys.min() + 1
        print('   c=(%7.2f,%7.2f) area=%6d r_area=%5.2f  half_w=%5.2f half_h=%5.2f'
              % (dx, dy, area, np.sqrt(area / np.pi) * (sx + sy) / 2, wpx * sx / 2, hpx * sy / 2))

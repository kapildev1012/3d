import numpy as np
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
    start = idx[0]
    prev = idx[0]
    for i in idx[1:]:
        if i != prev + 1:
            out.append((start, prev))
            start = i
        prev = i
    out.append((start, prev))
    return out


for nm in names:
    im = Image.open('figs_extracted/%s.png' % nm).convert('RGB')
    a = np.asarray(im).astype(int)
    H, W, _ = a.shape
    R, G, B = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    grayish = (abs(R - G) < 40) & (abs(G - B) < 40) & (R < 200)
    rowsum = grayish.sum(axis=1)
    colsum = grayish.sum(axis=0)
    axrow = int(np.argmax(rowsum))     # bottom axis line
    axcol = int(np.argmax(colsum))     # left axis line
    rr = max(runs(grayish[:, axcol]), key=lambda t: t[1] - t[0])
    cc = max(runs(grayish[axrow, :]), key=lambda t: t[1] - t[0])
    y_top, y_bot = rr
    x_left, x_right = cc
    print('=====', nm, 'W,H', W, H)
    print('   axis box px: x[%d,%d] y[%d,%d]  w=%d h=%d' %
          (x_left, x_right, y_top, y_bot, x_right - x_left, y_bot - y_top))
    sx = (XLIM[1] - XLIM[0]) / (x_right - x_left)
    sy = (YLIM[1] - YLIM[0]) / (y_bot - y_top)
    print('   units/px: sx=%.5f sy=%.5f  ratio=%.4f' % (sx, sy, sx / sy))

    def to_data(xp, yp):
        return (XLIM[0] + (xp - x_left) * sx, YLIM[1] - (yp - y_top) * sy)

    # opaque red obstacles
    red = (R > 200) & (G < 70) & (B < 70)
    lab, n = ndimage.label(red)
    print('   red blobs:', n)
    for k in range(1, n + 1):
        ys, xs = np.where(lab == k)
        area = len(xs)
        if area < 30:
            continue
        cx, cy = xs.mean(), ys.mean()
        dx, dy = to_data(cx, cy)
        rad_px = np.sqrt(area / np.pi)
        rad = rad_px * (sx + sy) / 2
        wpx = xs.max() - xs.min() + 1
        hpx = ys.max() - ys.min() + 1
        print('     center=(%7.2f,%7.2f) r_area=%5.2f  r_w=%5.2f r_h=%5.2f area=%d'
              % (dx, dy, rad, wpx * sx / 2, hpx * sy / 2, area))

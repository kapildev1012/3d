import numpy as np
from PIL import Image

names = ['fig5a', 'fig5b', 'fig5c', 'fig5d', 'fig6a', 'fig6b', 'fig6c', 'fig6d']
for nm in names:
    im = Image.open('figs_extracted/%s.png' % nm).convert('RGB')
    a = np.asarray(im).astype(int)
    H, W, _ = a.shape
    R, G, B = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    grayish = (abs(R - G) < 40) & (abs(G - B) < 40) & (R < 200)
    rowsum = grayish.sum(axis=1)
    colsum = grayish.sum(axis=0)
    print('=====', nm, W, H)
    print(' rows>30%%:', [(int(i), int(rowsum[i])) for i in np.where(rowsum > 0.3 * W)[0]])
    print(' cols>30%%:', [(int(i), int(colsum[i])) for i in np.where(colsum > 0.3 * H)[0]])

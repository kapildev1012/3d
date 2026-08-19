import pymupdf, os, numpy as np
from PIL import Image

PDF = 'Adaptive_Tensegrity-Based_Control_for_Multi-Agent_Obstacle_Avoidance.pdf'
d = pymupdf.open(PDF)
os.makedirs('figs_extracted', exist_ok=True)
p = d[4]
xrefs = [89, 90, 88, 94, 93, 96, 95, 97]
names = ['fig5a', 'fig5b', 'fig5c', 'fig5d', 'fig6a', 'fig6b', 'fig6c', 'fig6d']
for xref, nm in zip(xrefs, names):
    pix = pymupdf.Pixmap(d, xref)
    fn = 'figs_extracted/%s.png' % nm
    pix.save(fn)
    im = Image.open(fn).convert('RGB')
    a = np.asarray(im).astype(int)
    H, W, _ = a.shape
    R, G, B = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    # Pure red obstacles: high R, low G,B, fully opaque
    red = (R > 200) & (G < 80) & (B < 80)
    # Dark (black) axis lines / markers
    dark = (R < 100) & (G < 100) & (B < 100)
    print('=====', nm, 'size', W, H, 'red px', red.sum())
    # Find axis frame: the longest horizontal and vertical dark runs
    rowsum = dark.sum(axis=1)
    colsum = dark.sum(axis=0)
    cand_rows = np.where(rowsum > 0.5 * W)[0]
    cand_cols = np.where(colsum > 0.5 * H)[0]
    print('frame rows', cand_rows, 'frame cols', cand_cols)
    np.save('figs_extracted/%s_red.npy' % nm, red)
    np.save('figs_extracted/%s_dark.npy' % nm, dark)

import pymupdf, os, sys

PDF = 'Adaptive_Tensegrity-Based_Control_for_Multi-Agent_Obstacle_Avoidance.pdf'
d = pymupdf.open(PDF)
os.makedirs('pages', exist_ok=True)
print('pages', d.page_count)
for i, p in enumerate(d):
    print('page', i + 1, p.rect)
    pm = p.get_pixmap(dpi=170)
    pm.save('pages/page%d.png' % (i + 1))
    # also crop halves (left/right column) at higher dpi
    r = p.rect
    mid = (r.x0 + r.x1) / 2.0
    left = pymupdf.Rect(r.x0, r.y0, mid + 12, r.y1)
    right = pymupdf.Rect(mid - 12, r.y0, r.x1, r.y1)
    p.get_pixmap(dpi=200, clip=left).save('pages/page%d_L.png' % (i + 1))
    p.get_pixmap(dpi=200, clip=right).save('pages/page%d_R.png' % (i + 1))

with open('paper_blocks.txt', 'w', encoding='utf-8') as f:
    for i, p in enumerate(d):
        f.write('\n\n=============== PAGE %d ===============\n' % (i + 1))
        blocks = p.get_text('blocks', sort=False)
        # sort by column then y
        r = p.rect
        mid = (r.x0 + r.x1) / 2.0
        blocks.sort(key=lambda b: (0 if b[0] < mid - 30 else 1, round(b[1], 1), b[0]))
        for b in blocks:
            f.write('--- bbox=(%.0f,%.0f,%.0f,%.0f)\n' % (b[0], b[1], b[2], b[3]))
            f.write(b[4].rstrip() + '\n')
print('done')

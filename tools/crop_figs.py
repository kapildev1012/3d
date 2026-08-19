import pymupdf, os

PDF = 'Adaptive_Tensegrity-Based_Control_for_Multi-Agent_Obstacle_Avoidance.pdf'
d = pymupdf.open(PDF)
os.makedirs('pages', exist_ok=True)
p = d[4]  # page 5 (0-indexed 4)
# Right column spans roughly x in [300, 580] pt
crops = {
    'fig5': pymupdf.Rect(300, 450, 590, 560),
    'fig6': pymupdf.Rect(300, 600, 590, 710),
    'fig5a': pymupdf.Rect(300, 450, 380, 550),
    'fig5d': pymupdf.Rect(500, 450, 590, 550),
    'fig6d': pymupdf.Rect(500, 600, 590, 700),
    'fig6c': pymupdf.Rect(420, 600, 510, 700),
    'table1': pymupdf.Rect(300, 330, 590, 450),
}
for name, r in crops.items():
    p.get_pixmap(dpi=700, clip=r).save('pages/%s.png' % name)
    print(name, 'saved')

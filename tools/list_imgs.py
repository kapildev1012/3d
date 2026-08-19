import pymupdf, os

PDF = 'Adaptive_Tensegrity-Based_Control_for_Multi-Agent_Obstacle_Avoidance.pdf'
d = pymupdf.open(PDF)
os.makedirs('pages', exist_ok=True)
p = d[4]
# List embedded images with their placement rectangles
for info in p.get_image_info(xrefs=True):
    print(info['xref'], info['bbox'], info['width'], info['height'])

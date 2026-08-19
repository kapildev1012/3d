from PIL import Image
im = Image.open('figs_extracted/fig5b.png')
print(im.size)
im.crop((80, 300, 430, 700)).resize((700, 800), Image.LANCZOS).save('figs_extracted/zoom5b.png')
im2 = Image.open('figs_extracted/fig5a.png')
im2.crop((100, 600, 560, 880)).resize((920, 560), Image.LANCZOS).save('figs_extracted/zoom5a.png')

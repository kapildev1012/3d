from PIL import Image
for nm, box, out in [('fig5d', (100, 100, 660, 600), 'zoom5d.png'),
                     ('fig6d', (100, 40, 620, 560), 'zoom6d.png'),
                     ('fig5c', (100, 200, 660, 700), 'zoom5c.png'),
                     ('fig6c', (100, 200, 620, 700), 'zoom6c.png')]:
    im = Image.open('figs_extracted/%s.png' % nm)
    im.crop(box).resize((int((box[2] - box[0]) * 1.6), int((box[3] - box[1]) * 1.6)),
                        Image.LANCZOS).save('figs_extracted/' + out)
print('ok')

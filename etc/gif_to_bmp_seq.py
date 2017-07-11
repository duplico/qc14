from PIL import Image
import sys

def iter_frames(im):
    try:
        i= 0
        while 1:
            im.seek(i)
            imframe = im.copy()
            if i == 0: 
                palette = imframe.getpalette()
            else:
                imframe.putpalette(palette)
            yield imframe
            i += 1
    except EOFError:
        pass

def main():
    path = sys.argv[1]
    if '.' in sys.argv[1]:
        nameroot = ''.join(sys.argv[1].split('.')[:-1])
    else:
        nameroot = sys.argv
    im = Image.open(path)
    for i, frame in enumerate(iter_frames(im)):
        frame.thumbnail((7,7))
        frame.save('%s.%03d.bmp' % (nameroot,i),**frame.info)
        
if __name__ == '__main__':
    main()
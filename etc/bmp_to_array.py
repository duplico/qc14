import math
import argparse

from PIL import Image

# CONSIDERATION: Maybe let's normalize everything - try to keep everything that's not OFF, the same intensity????

# /   printf("%e*V+.5, ", (pow((double)x / 255.0, 2.5)));

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

def bmp_seq(imgpath):
    path = imgpath
    im = Image.open(path)
    seq = []
    for i, frame in enumerate(iter_frames(im)):
        frame.thumbnail((7,7))
        seq.append(frame)
        # frame.save('%s.%03d.bmp' % (nameroot,i),**frame.info)
    return seq

def print_array(i):
    
    bytes = list(map(ord, i.tobytes()))
    bytes = [int(math.pow(a/255.0,2.5)*255 + 0.5) for a in bytes]

    pixels = []

    pixels = [bytes[x:x+3] for x in range(0, len(bytes), 3)]

    lines = [pixels[x:x+7] for x in range(0, len(pixels), 7)]

    # for i in range(len(bytes))[::3]:
        # pixels.append('{%d, %d, %d}' % (bytes[i], bytes[i+1], bytes[i+2]))
        
    print str(lines).replace('[', '{').replace(']', '}') + ';'
    
def main():
    parser = argparse.ArgumentParser("Create an integer array from a bmp or gif file.")
    parser.add_argument('imgpath', action='store', type=str, help='Path to bitmap or gif file')
    
    args = parser.parse_args()
    
    if args.imgpath.endswith('.gif'):
        seq = bmp_seq(args.imgpath)
    else:
        i = Image.open(args.imgpath)
        i.thumbnail
        seq = [i]
        
    for img in seq:
        print_array(img)
    
if __name__ == "__main__":
    main()
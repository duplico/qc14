import math
import argparse
from intelhex import IntelHex

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

def bmp_bytes(i):
    i.thumbnail([7,7])
        
    bytes = list(map(ord, i.tobytes()))
    bytes = [int(math.pow(a/255.0,2.5)*255 + 0.5) for a in bytes]
    return bytes + [0]
    
def main():
    parser = argparse.ArgumentParser("Create an integer array from a bmp file.")
    parser.add_argument('imgpath', action='store', type=str, help='Path to bitmap or gif file')
    parser.add_argument('hexpath', action='store', type=str, help='Output file path')
    parser.add_argument('-s', '--start-addr', type=int, default=0x010000)
    
    args = parser.parse_args()
    
    bytes = []
    
    imgsrcs = []
    
    if args.imgpath.endswith('.gif'):
        im = Image.open(args.imgpath)
        for i, frame in enumerate(iter_frames(im)):
            frame = frame.convert('RGB')
            frame.thumbnail((7,7))
            frame.save('.qctmp.bmp',**frame.info)
            imgsrcs.append(Image.open('.qctmp.bmp'))
    else:
        imgsrcs = [Image.open(args.imgpath)]

    bts = []
    for src in imgsrcs:
        bts += bmp_bytes(src)
        
    ih = IntelHex()
    for i in range(len(bts)):
        ih[args.start_addr + i] = bts[i]
        
    if args.hexpath.endswith('.hex'):
        ih.write_hex_file(args.hexpath)
    else:
        ih.tobinfile(args.hexpath)
    print len(bts)
    
if __name__ == "__main__":
    main()
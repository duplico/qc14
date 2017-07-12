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
            imgsrcs.append(frame)
    else:
        imgsrcs = [Image.open(args.imgpath)]

    start_addr = args.start_addr
    bts = []
    ih = IntelHex()
    total_len = 0
    
    for src in imgsrcs:
        bts = bmp_bytes(src)
        for i in range(len(bts)):
            ih[start_addr + i] = bts[i]
        start_addr += 0x0200
        total_len += len(bts)
        
    if args.hexpath.endswith('.hex'):
        ih.write_hex_file(args.hexpath)
    else:
        ih.tobinfile(args.hexpath)
        
    print total_len
    
if __name__ == "__main__":
    main()
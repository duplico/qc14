import math
import argparse

import struct
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
    # Let's start by making i square.
    # We'll do this by cropping.
    
    width, height = i.size
    
    sq_dim = min(width, height)
    
    left = (width - sq_dim)/2
    top = (height - sq_dim)/2
    right = (width + sq_dim)/2
    bottom = (height + sq_dim) / 2
    i = i.crop((left,top,right,bottom))

    size = (7, 7)
    
    i.thumbnail(size) #, Image.ANTIALIAS)
    background = Image.new('RGB', size, (0, 0, 0))
    background.paste(
        i, (int((size[0] - i.size[0]) / 2), int((size[1] - i.size[1]) / 2))
    )
        
    bytes = list(map(ord, i.tobytes()))
    bytes = [int(math.pow(a/255.0,2.5)*255 + 0.5) for a in bytes]
    return bytes + [0]
    
def main():
    parser = argparse.ArgumentParser("Create an integer array from a bmp file.")
    parser.add_argument('imgpath', action='store', type=str, help='Path to bitmap or gif file')
    parser.add_argument('hexpath', action='store', type=str, help='Output file path')
    parser.add_argument('-s', '--start-addr', type=int, default=0x010000)
    parser.add_argument('-e', '--even-pages-only', action='store_true', help='Do the workaround when we cannot use odd numbered pages.')
    parser.add_argument('-d', '--frame-delay', type=int, default=30)
    
    args = parser.parse_args()
    
    bytes = []
    
    imgsrcs = []
    
    if args.imgpath.endswith('.gif'):
        im = Image.open(args.imgpath)
        for i, frame in enumerate(iter_frames(im)):
            frame = frame.convert('RGB')
            imgsrcs.append(frame)
    else:
        imgsrcs = [Image.open(args.imgpath)]

    start_addr = args.start_addr
    if args.even_pages_only:
        start_addr = start_addr * 2
    
    bts = []
    ih = IntelHex()
    total_len = 0
    
    curr_addr = start_addr
    
    # Let's start by writing the animation struct. It looks like this:
    #   typedef struct {
    #       uint32_t anim_start_frame;
    #       uint16_t anim_len;
    #       uint16_t anim_frame_delay_ms;
    #   } screen_anim_t;
    # Little-Endian; unsigned int; unsigned short; unsigned short:

    anim_pack_string = '<IHH'
    
    bts = map(ord, list(struct.pack(
        anim_pack_string,
        0,
        len(imgsrcs),
        args.frame_delay
    )))
        
    for i in range(len(bts)):
        ih[curr_addr] = bts[i]
        curr_addr += 1
        if args.even_pages_only and curr_addr % 0x0100 == 0:
            curr_addr += 0x0100 # Skip an odd one.
    total_len += len(bts)
    
    for src in imgsrcs:
        bts = bmp_bytes(src)
        for i in range(len(bts)):
            ih[curr_addr] = bts[i]
            curr_addr += 1
            if args.even_pages_only and curr_addr % 0x0100 == 0:
                curr_addr += 0x0100 # Skip an odd one.
        total_len += len(bts)
        
    if args.hexpath.endswith('.hex'):
        ih.write_hex_file(args.hexpath)
    else:
        ih.tobinfile(args.hexpath)
        
    print curr_addr - start_addr
    
if __name__ == "__main__":
    main()
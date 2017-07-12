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
    length = 0
    print 'screen_frame_t master_anim_buf[] = {'
    for i, frame in enumerate(iter_frames(im)):
        print "// frame %d" % length
        length+=1
        frame = frame.convert('RGB')
        frame.thumbnail((7,7))
        frame.save('.qctmp.bmp',**frame.info)
        print_array('.qctmp.bmp', True)
    print '}; // len: %d' % length

def print_array(img_path, collection=False):
    i = Image.open(img_path)
    i.thumbnail([7,7])
        
    bytes = list(map(ord, i.tobytes()))
    bytes = [int(math.pow(a/255.0,2.5)*255 + 0.5) for a in bytes]

    pixels = []

    pixels = [bytes[x:x+3] for x in range(0, len(bytes), 3)]

    lines = [pixels[x:x+7] for x in range(0, len(pixels), 7)]
    
    # for i in range(len(bytes))[::3]:
        # pixels.append('{%d, %d, %d}' % (bytes[i], bytes[i+1], bytes[i+2]))
        
    print str([lines, 0]).replace('[', '{').replace(']', '}') + (',' if collection else ';')
    
def main():
    parser = argparse.ArgumentParser("Create an integer array from a bmp file.")
    parser.add_argument('imgpath', action='store', type=str, help='Path to bitmap file')
    
    args = parser.parse_args()
        
    if args.imgpath.endswith('.gif'):
        bmp_seq(args.imgpath)
    else:
        print_array(args.imgpath)
    
if __name__ == "__main__":
    main()
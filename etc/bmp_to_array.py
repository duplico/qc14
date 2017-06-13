import math
import argparse

from PIL import Image

# CONSIDERATION: Maybe let's normalize everything - try to keep everything that's not OFF, the same intensity????

# /   printf("%e*V+.5, ", (pow((double)x / 255.0, 2.5)));

def print_array(img_path):
    i = Image.open(img_path)
    i.thumbnail([7,7])
    
    bytes = list(map(ord, i.tobytes()))
    bytes = [int(math.pow(a/255.0,2.5)*255 + 0.5) for a in bytes]

    pixels = []

    pixels = [bytes[x:x+3] for x in range(0, len(bytes), 3)]

    lines = [pixels[x:x+7] for x in range(0, len(pixels), 7)]

    # for i in range(len(bytes))[::3]:
        # pixels.append('{%d, %d, %d}' % (bytes[i], bytes[i+1], bytes[i+2]))
        
    print str(lines).replace('[', '{').replace(']', '}') + ';'
    
def main():
    parser = argparse.ArgumentParser("Create an integer array from a bmp file.")
    parser.add_argument('imgpath', action='store', type=str, help='Path to bitmap file')
    
    args = parser.parse_args()
    print_array(args.imgpath)
    
if __name__ == "__main__":
    main()
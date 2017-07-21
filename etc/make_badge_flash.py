import math
import argparse
import os, os.path

import struct
from intelhex import IntelHex
from PIL import Image

import make_game

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
    
def image_list_from_directory(dirpath):
    assert os.path.isdir(dirpath)
    images = []
    for filename in (os.path.join(dirpath, d) for d in os.listdir(dirpath)):
        if filename.endswith('.bmp') or filename.endswith('.png'):
            images.append(Image.open(filename).convert('RGB'))
        elif filename.endswith('.gif'):
            im = Image.open(filename)
            for i, frame in enumerate(iter_frames(im)):
                frame = frame.convert('RGB')
                images.append(frame)
    return images
            
def get_frame_delay(dirpath, default):
    assert os.path.isdir(dirpath)
    dirpath = os.path.join(dirpath, 'delay')
    if os.path.exists(dirpath) and os.path.isfile(dirpath):
        with open(dirpath) as f:
            return int(f.readline())
    return default
            
# Returns next position
def put_bytes_at(ih, position, bts, workaround):
    curr_page = position / 0x0100
    curr_byte = position % 0x0100
    
    multiplier = 0x0200 if workaround else 0x0100
    
    for i in range(len(bts)):
        ih[curr_page*multiplier + curr_byte] = bts[i]
        curr_byte += 1
        if curr_byte == 0x0100:
            curr_page += 1
            curr_byte = 0
    return curr_page*multiplier + curr_byte
    
def anim_struct_bytes(anim_start_frame, anim_len, anim_frame_delay_ms):
    # Let's start by writing the animation struct. It looks like this:
    #   typedef struct {
    #       uint32_t anim_start_frame;
    #       uint16_t anim_len;
    #       uint16_t anim_frame_delay_ms;
    #   } screen_anim_t;
    # Little-Endian; unsigned int; unsigned short; unsigned short:

    anim_pack_string = '<IHH'
        
    return map(ord, list(struct.pack(
        anim_pack_string,
        anim_start_frame,
        anim_len,
        anim_frame_delay_ms
    )))
    
def icon_struct_bytes(id, animation_bytes, connections):        
    # We're returning a packed game_icon_t from this.
    
    # typedef struct {
        # uint16_t id;
        # screen_anim_t animation;
        # mate_spec_t arms[4];
    # } game_icon_t;
    icon_pack_string = '<Hxx'
    # then animation_bytes
    # then four of these:
    
    # typedef struct {
        # uint16_t mate_icon_id;
        # uint16_t result_icon_id;
        # uint16_t arm_anim_id;
        # uint8_t sufficient_flag;
        # uint8_t other_arm_id;
        # rgbcolor_t arm_color;
    # } mate_spec_t;
        
    # Then one of these:
    
    # typedef struct {
        # uint8_t red;
        # uint8_t green;
        # uint8_t blue;
    # } rgbcolor_t;
    
    connection_pack_string = '<HHHBBBBBx'
    
    bytelist = map(ord, list(struct.pack(
        icon_pack_string,
        id
    )))
    
    bytelist += animation_bytes
    
    for connection in connections:
        bytelist += map(ord, list(struct.pack(
            connection_pack_string,
            connection.mate_icon if connection else 0,
            connection.result_icon if connection else 0,
            connection.arm_anim_id if connection else 0,
            connection.sufficiency if connection else 0,
            connection.other_arm_id if connection else 0,
            connection.rgb[0] if connection else 0,
            connection.rgb[1] if connection else 0,
            connection.rgb[2] if connection else 0,
        )))
    return bytelist
    
            
def main():
    parser = argparse.ArgumentParser("Create the flash data for a queercon 14 badge.")
        
    # 0x000000 - reserved
    # 0x001000 - first ID in here
    # 0x002000 - main conf in here
    # 0x003000 - boot anim here
    # 0x004000 - tiles live in here
    # 0x008000 - backup conf here
    # 0x009000 - second ID copy here
    # 0x00a000 - icons live in here
    # 0x010000 - frames live here
    # 0x200000 - end of memory (real)
        
    parser.add_argument('--workaround', action='store_true', help="Work around the issue where we cannot use odd numbered pages. Note that this must be paired with using the `skipodd` versions of the flash functions on the badge.")
    parser.add_argument('--boot-addr', type=int, default=0x003000, help="Address of the boot animation struct")
    parser.add_argument('--tile-addr', type=int, default=0x004000, help="Address of the tile animation buffer")
    parser.add_argument('--game-addr', type=int, default=0x00a000, help="Address of the game animation struct")
    parser.add_argument('--frame-addr', type=int, default=0x010000, help='Starting offset for animation frames')
    parser.add_argument('--id-addr', type=int, default=0x001000, help="Address of the badge ID.")
    parser.add_argument('--id-addr2', type=int, default=0x009000, help="Address of the badge ID.")
    
    parser.add_argument('-o', '--hexpath', action='store', type=str, default='a.bin', help='Output file path')
    parser.add_argument('-d', '--frame-delay', type=int, default=30, help="Default frame delay for animations")
    parser.add_argument('-n', '--handle', action='store', type=str, default='Human')
    parser.add_argument('id', type=int, action='store', default=1)
        
    args = parser.parse_args()
        
    flash = IntelHex()
    all_frames = []
    curr_frame_index = 0
    tile_animations = [] # This is a bit sequence
    game_animations = [] # This is a bit sequence (not nested)
    
    put_bytes_at(flash, 0, [0xaa], args.workaround)
    
    # OK. The badge will handle the main and backup confs.
    # All we need along those lines is to give it the ID.
    put_bytes_at(flash, args.id_addr, map(ord, struct.pack('<H', args.id)), args.workaround)
    put_bytes_at(flash, args.id_addr2, map(ord, struct.pack('<H', args.id)), args.workaround)
    
    # Load up the boot animation
    imgs = image_list_from_directory('_badge_graphics/bootup')
    boot_anim = anim_struct_bytes(
        0, 
        len(imgs), 
        get_frame_delay('_badge_graphics/bootup', args.frame_delay)
    )
    put_bytes_at(flash, args.boot_addr, boot_anim, args.workaround)
    
    all_frames += imgs
    
    # Load up the tile animations
    
    assert os.path.isdir('_badge_graphics/tiles')# TODO: Demeter
    for dirname in os.listdir('_badge_graphics/tiles'):
        dpath = os.path.join('_badge_graphics/tiles', dirname)
        if not os.path.isdir(dpath): continue
        # Each of these corresponds to a tile.
        imgs = image_list_from_directory(dpath)
        tile_animations += anim_struct_bytes(
            len(all_frames), # starting index
            len(imgs), # length
            get_frame_delay(dpath, args.frame_delay) # delay
        )
        all_frames += imgs
        
    put_bytes_at(flash, args.tile_addr, tile_animations, args.workaround)
    
    # Load up the game icons
    # These are a little different. They look, for now, like:
        # typedef struct {
            # uint16_t mate_icon_id;
            # uint16_t result_icon_id;
            # uint16_t arm_anim_id;
            # uint8_t sufficient_flag;
            # uint8_t other_arm_id;
        # } mate_spec_t;

        # typedef struct {
            # uint16_t id;
            # screen_anim_t animation;
            # mate_spec_t arms[4];
        # } game_icon_t;
    
    icon_connections = make_game.get_icons()
    
    icon_id = 0
    
    assert os.path.isdir('_badge_graphics/game') # TODO: Demeter
    for dirname in os.listdir('_badge_graphics/game'):
        dpath = os.path.join('_badge_graphics/game', dirname)
        if not os.path.isdir(dpath): continue
        # Each of these corresponds to an icon.
        imgs = image_list_from_directory(dpath)
        game_animations += icon_struct_bytes(
            icon_id, 
            anim_struct_bytes(
                len(all_frames), # starting index
                len(imgs), # length
                get_frame_delay(dpath, args.frame_delay) # delay # TODO: Demeter
            ), 
            icon_connections[icon_id]
        )
        all_frames += imgs
        icon_id += 1
    
    put_bytes_at(flash, args.game_addr, game_animations, args.workaround)
        
    # OK, so now we've got the animation structs for the boot-up,
    #  game, and tiles all loaded up. Now we just need to add
    #  all the actual frames.
    
    # This part's pretty easy, actually.
    flattened_frames = reduce(lambda a,b: a+b, map(bmp_bytes, all_frames))
    last_byte = put_bytes_at(flash, args.frame_addr, flattened_frames, args.workaround)
    
    if args.hexpath.endswith('.hex'):
        flash.write_hex_file(args.hexpath)
    else:
        flash.tobinfile(args.hexpath)
        
    print last_byte
    
if __name__ == "__main__":
    main()
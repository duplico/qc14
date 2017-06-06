import pprint

arms = [
    [71, 51, 52, 53, 54, 55],
    [74, 69, 68, 66, 67, 70],
    [73, 63, 62, 65, 64, 61],
    [59, 58, 57, 60, 72, 56]
]

screen = [
    [1, 6, 11, 16, 21, 26, 31],
    [2, 7, 12, 17, 22, 27, 32],
    [3, 8, 13, 18, 23, 28, 33],
    [4, 9, 14, 19, 24, 29, 34],
    [5, 10, 15, 20, 25, 30, 35],
    [46, 36, 37, 38, 39, 40, 48],
    [47, 41, 42, 43, 44, 45, 49],
]

screen_transforms = []

for i in range(15):
    screen_transforms.append(['']*15)
        
for arm_index in range(4):
    for led_index in range(6):
        led_id = arms[arm_index][led_index]
        channel = ((led_id-1) % 5) * 3
        scanline = int((led_id-1)/5)
        print scanline, channel
        for color in range(3):
            screen_transforms[scanline][channel+color] = '&arms[%d][%d][%d]' % (arm_index, led_index, color)

for screen_row in range(7):
    for screen_column in range(7):
        led_id = screen[screen_row][screen_column]
        channel = ((led_id-1) % 5) * 3
        scanline = int((led_id-1)/5)
        print scanline, channel
        for color in range(3):
            screen_transforms[scanline][channel+color] = '&screen[%d][%d][%d]' % (screen_column, screen_row, color)
            
for scanline in range(15):
    for channel in range(15):
        if not screen_transforms[scanline][channel]:
            screen_transforms[scanline][channel] = '&dontcare'
            
pprint.pprint(screen_transforms)

print '{'
for i in range(15):
    print '    {',
    print ', '.join(screen_transforms[i]) + '},'
print '};'
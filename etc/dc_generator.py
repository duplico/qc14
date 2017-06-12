# r = .67
# g = .25
# b = 1

# R = 1
# G = .3
# B = .9

r = 1
g = .2
b = .85

r_val = int(r*127)
g_val = int(g*127)
b_val = int(b*127)

r_str = format(r_val, '07b')
g_str = format(g_val, '07b')
b_str = format(b_val, '07b')

vals = '0000000'

for i in range(5):
    vals += b_str + g_str + r_str
    
strs = ['0b%s' % vals[x:x+8] for x in range(0, len(vals), 8)]

print ',\n'.join(strs)
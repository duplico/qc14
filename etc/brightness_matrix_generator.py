import sys

low = int(sys.argv[1])
high = int(sys.argv[2])
step = (high-low) / 19

for i in range(20):
    cutoff = low + step * i
    print "{%d, %d}," % (cutoff, int((127/19.0) * i))
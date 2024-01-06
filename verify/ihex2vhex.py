import sys
import os.path

for i, line in enumerate(open(sys.argv[1])):
    l = line[1:3]
    a = line[3:7]
    t = line[7:9]
    #print(line)
    #print(l, a, t)
    if t == "00" :
        addr = int(int(a, 16)/2)
        #print(addr)
        print(f"@{addr:x}")
        for ii in range(int(l,16)*2) :
            if ii%4 == 0 :
                print(f"{line[11+ii:13+ii]}{line[9+ii:11+ii]}")
            



    
#!/usr/python3

import sys

def print_array(bidule, filename):
    with open(filename, 'w') as f:
        for d in bidule:
            print(d[0], d[1], file=f)

def get_cumulative(arr, elt):
    cur = sorted(arr, key=lambda truc: truc[elt])
    ret = []
    prev = 0
    count = 0
    for d in cur:
        if d["res"] == "O":
            if d[elt] != prev and count > 0:
                ret.append((prev, count/len(cur)))
                prev = d[elt]
            count += 1
        else:
            break
    ret.append((prev, count/len(cur)))
    return ret

def get_average(arr, elt):
    tot=0.0
    for d in arr:
        tot += d[elt]
    return tot/len(arr)

data=[]
for line in sys.stdin:
    tokens = line.split("\t")
    if len(tokens) > 0:
        data.append({
            "cells" : int(tokens[0]),
            "nets"  : int(tokens[1]),
            "res"   : tokens[2],
            "time"  : int(tokens[3]),
            "nodes" : int(tokens[4]),
            #"bprun" : int(tokens[5]),
            #"fprun" : int(tokens[6]),
        })

if len(data) == 0:
    quit()

#cell_counts = {cur[cells] for cur in data}

print("Number of problems: ", len(data))
print("Average time is: ", get_average(data, "time"))
print("Average visited node count is: ", get_average(data, "nodes"))
print("Average time for succesful runs is: ", get_average([d for d in data if d["res"] == "O"], "time"))
print("Average node count for succesful runs is: ", get_average([d for d in data if d["res"] == "O"], "nodes"))

print_array(get_cumulative(data, "time"),  sys.argv[1]+"/times_"+sys.argv[2])
print_array(get_cumulative(data, "nodes"), sys.argv[1]+"/nodes_"+sys.argv[2])


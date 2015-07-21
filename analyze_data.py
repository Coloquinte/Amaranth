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
            "fixeds": int(tokens[1]),
            "res"   : tokens[3],
            "time"  : int(tokens[4]),
            "nodes" : int(tokens[5]),
            "cost" : int(tokens[6]),
        })

if len(data) == 0:
    quit()

#cell_counts = {cur[cells] for cur in data}

improved_cnt = len([d for d in data if d["res"]=="O" or d["res"] == "U"]) # improved
optimal_cnt = len([d for d in data if d["res"]=="O" or d["res"] == "I"]) # optimal
any_cnt = len([d for d in data if d["res"]=="O" or d["res"] == "I" or d["res"] == "U"]) # one of them
both_cnt = len([d for d in data if d["res"]=="O"]) # all of them
average_explo = get_average(data, "nodes")
average_time = get_average(data, "time")
average_s_explo = get_average([d for d in data if d["res"] == "O" or d["res"] == "I"], "nodes")
average_s_time = get_average([d for d in data if d["res"] == "O" or d["res"] == "I"], "time")

print(
 len(data)
,improved_cnt # improved
,optimal_cnt # optimal
,any_cnt # one of them
,both_cnt # all of them
,average_time # global average time
,average_s_time # successful average time
,average_explo # global average nodes
,average_s_explo # successful average nodes
)


#print("Number of problems: ", len(data))
#print("Improved or solved to optimality: ", len([d for d in data if d["res"]=="O" or d["res"] == "I" or d["res"] == "U"]))
#print("Improved: ", len([d for d in data if d["res"]=="O" or d["res"] == "U"]))
#print("Solved to optimality: ", len([d for d in data if d["res"]=="O" or d["res"] == "I"]))
#print("Optimal and improved: ", len([d for d in data if d["res"]=="O"]))
#print("Average time is: ", get_average(data, "time"))
#print("Average time for successful runs is: ", average_time)
#print("Explored " + str(get_average(data, "nodes")) + " nodes on average")
#print("For successful runs, explored " + str(average_explo) + " nodes on average")
#print("Average cost " + str(get_average([ d for d in data if d["res"] != "F"], "cost")) + " on average")

print_array(get_cumulative(data, "time"),  sys.argv[1]+"/times_"+sys.argv[2])
print_array(get_cumulative(data, "nodes"), sys.argv[1]+"/nodes_"+sys.argv[2])


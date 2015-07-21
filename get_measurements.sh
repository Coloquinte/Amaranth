#!/bin/bash

dataset=$1
resultdir=$2

for cells in {6..16}
do
end=$((cells/2))
for rows in $(seq 1 $end)
do
suffix=${cells}c${rows}r
dataname=${dataset}/${resultdir}/results_${suffix}
[ -f $dataname ] && cat $dataname
done | grep "^${cells}" | python3 analyze_data.py ${dataset}/${resultdir} ${cells}
done


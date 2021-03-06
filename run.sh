#!/bin/bash

dataset=$1
resultdir=$2

[ -d ${dataset}/${resultdir} ] || mkdir ${dataset}/${resultdir}

for cells in {6..16}
do
end=$((cells/2))
for rows in $(seq 1 $end)
do
suffix=${cells}c${rows}r
dirname=${dataset}/data_${suffix}
outname=${dataset}/${resultdir}/results_${suffix}
rm $outname
[ -d $dirname ] && 
for i in {0..100}
do
    ls $dirname | grep window_${i}_ | parallel -j4 -k --linebuffer "./truc < ${dirname}/{}" >> $outname 
done
done
done


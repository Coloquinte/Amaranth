
dataset=$1
resultdir=$2

for cells in {8,12,16,20,24}
do
for rows in {2..8}
do
suffix=${cells}c${rows}r
dataname=${dataset}/${resultdir}/results_${suffix}
[ -f $dataname ] && cat $dataname
done | grep "^${cells}" | python3 analyze_data.py ${dataset}/${resultdir} ${cells}
done


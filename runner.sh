#!/bin/bash
#PBS -N onetoallquery
#PBS -l select=1:ncpus=24:mem=500gb,walltime=72:00:00
#PBS -q bigmem
#PBS -o /home/jsybran/job.out
#PBS -e /home/jsybran/job.err
#PBS -M jsybran@clemson.edu
#PBS -m ea

module load gcc openmpi

ID_A=C0038250
TID_FILE=~/projects/queries/cpp/TIDS
GRAPH_FILE="/scratch2/jsybran/FINAL_NETWORK/subgraph.edges"
OUT_FILE="/scratch2/jsybran/oneToAllResults"

/home/jsybran/projects/queries/cpp/runDijkstra $GRAPH_FILE $TID_FILE $ID_A $OUT_FILE

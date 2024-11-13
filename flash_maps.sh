#!/usr/bin/env bash
#
# Demonstrates command-line interface of Partition Tool, parttool.py
#
#
# $1 - serial port where target device to operate on is connnected to, by default the first found valid serial port
# $2 - path to this example's built binary file (parttool.bin), by default $PWD/build/parttool.bin
export IDF_PATH='/home/ok-home/esp/v5.2.1/esp-idf'
PORT=$1
PARTTOOL_PY="/home/ok-home/.espressif/python_env/idf5.2_py3.10_env/bin/python $IDF_PATH/components/partition_table/parttool.py -q"

if [[ "$PORT" != "" ]]; then
    PARTTOOL_PY="$PARTTOOL_PY --port $PORT"
fi

# Write the contents of the created file to storage partition
echo "Writing to mapx partition"
$PARTTOOL_PY write_partition --partition-name=mapx --input mapx.bin
echo "Writing to mapy partition"
$PARTTOOL_PY write_partition --partition-name=mapy --input mapy.bin
echo "Writing to roi partition"
$PARTTOOL_PY write_partition --partition-name=roi --input roi.bin

# Example end and cleanup
printf "\nPartition tool operations performed successfully\n"

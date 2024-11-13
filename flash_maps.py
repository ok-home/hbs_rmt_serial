#!/usr/bin/env python
#
# Demonstrates the use of parttool.py, a tool for performing partition level
# operations.
#
# Copyright 2018 Espressif Systems (Shanghai) PTE LTD
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import argparse
import os
import sys

PARTITION_TABLE_DIR = os.path.join('components', 'partition_table', '')


def main():
    COMPONENTS_PATH = os.path.expandvars(os.path.join('$IDF_PATH', 'components'))
    PARTTOOL_DIR = os.path.join(COMPONENTS_PATH, 'partition_table')

    sys.path.append(PARTTOOL_DIR)
    from parttool import PartitionName, PartitionType, ParttoolTarget

    parser = argparse.ArgumentParser('ESP-IDF Partitions Tool Example')

    parser.add_argument('--port', '-p', help='port where the device to perform operations on is connected')
    #parser.add_argument('--binary', '-b', help='path to built example binary', default=os.path.join('build', 'parttool.bin'))

    args = parser.parse_args()

    target = ParttoolTarget(args.port)
  
    # Retrieve info on data storage partition, this time identifying it by name.
    storage_mapx = PartitionName('mapx')
    storage_info = target.get_partition_info(storage_mapx)
    print('Found data partition at offset 0x{:x} with size 0x{:x}'.format(storage_info.offset, storage_info.size))

    storage_mapy = PartitionName('mapy')
    storage_info = target.get_partition_info(storage_mapy)
    print('Found data partition at offset 0x{:x} with size 0x{:x}'.format(storage_info.offset, storage_info.size))

    storage_roi = PartitionName('roi')
    storage_info = target.get_partition_info(storage_roi)
    print('Found data partition at offset 0x{:x} with size 0x{:x}'.format(storage_info.offset, storage_info.size))


    # Write the contents of the created file to storage partition
    print('Writing to mapx partition')
    target.write_partition(storage_mapx, 'mapx.bin')

    print('Writing to mapx partition')
    target.write_partition(storage_mapy, 'mapy.bin')

    print('Writing to mapx partition')
    target.write_partition(storage_roi, 'roi.bin')

    # Example end and cleanup
    print('\nPartition tool operations performed successfully!')

if __name__ == '__main__':
    main()

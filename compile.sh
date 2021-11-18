#!/bin/bash
rm build -rf
mkdir build
cd build
cmake -DMCU_TYPE=MM32SPIN0x ..
make
#rm -rf ~/shardir/mm32_firmware/mm32_project_demo.hex
#cp mm32_project_demo.hex ~/shardir/mm32_firmware/
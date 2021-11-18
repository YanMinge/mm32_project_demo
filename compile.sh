#!/bin/bash
rm build -rf
mkdir build
cd build
cmake -DMCU_TYPE=MM32SPIN0x ..
make
rm -rf ~/shardir/mm32_firmware/ai_set_motor.hex
cp ai_set_motor.hex ~/shardir/mm32_firmware/
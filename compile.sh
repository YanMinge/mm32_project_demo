#!/bin/bash
rm build -rf
mkdir build
cd build
cmake -DMCU_TYPE=MM32F013x ..
make
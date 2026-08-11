#!/usr/bin/env bash

rm -rf build/*
cd build
cmake -DCMAKE_INSTALL_PREFIX=$ARGOS_INSTALL_PATH/argos3-dist ../src
make

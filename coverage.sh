#!/usr/bin/env sh

cd build/march_rqt_gait_generator || return
coverage xml --rcfile ../../src/gait-generator/.coveragerc
cd ../.. || return

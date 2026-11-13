#!/bin/sh
# Compiles the real firmware against PC stand-ins for the Arduino libraries and runs the checks.
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -Wall -Wno-unused-variable -I. test_scheduler.cpp -o /tmp/eva_firmware_sim
/tmp/eva_firmware_sim

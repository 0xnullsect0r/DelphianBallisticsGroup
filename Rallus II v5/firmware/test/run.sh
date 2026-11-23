#!/bin/sh
# Host tests for the flight-critical firmware logic. No Pico needed:
#   sh firmware/test/run.sh
set -e
cd "$(dirname "$0")"
CFLAGS="-O1 -w -Istubs -I../include"
gcc $CFLAGS test_flashlog.c ../src/flashlog.c -o /tmp/rallus_test_flashlog
gcc $CFLAGS test_state.c ../src/state.c ../src/control.c -lm -o /tmp/rallus_test_state
/tmp/rallus_test_flashlog
/tmp/rallus_test_state

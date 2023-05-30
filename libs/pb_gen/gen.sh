#!/usr/bin/env bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
NANOPB_DIR=$SCRIPT_DIR/../nanopb
$NANOPB_DIR/generator/nanopb_generator.py simple.proto

#!/usr/bin/env bash

# arg 1 - working directory (optional)
if [ ! -z $1 ]; then
    cd $1
fi
python -m SimpleHTTPServer

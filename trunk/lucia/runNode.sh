#!/bin/sh
if [ $# -ne 1 ]; then
   echo "Usage: ./runNode <simple class name>"
   exit 127
fi
echo "Running node "$1
./contextrecognition/build/install/contextrecognition/bin/contextrecognition se.oru.lucia.contextrecognition.$1

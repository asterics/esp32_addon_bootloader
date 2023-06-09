#!/bin/bash

for dir in ./main/ ; do
    find $dir -type f \( -name "*.c" -o -name "*.h" -o -name "*.cpp" \) -a  \! -path '*api*' -exec astyle --suffix=none --options=./tools/astyle_core.conf \{\} \;
done


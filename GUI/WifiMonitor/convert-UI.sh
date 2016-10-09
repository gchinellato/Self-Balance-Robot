#!/bin/bash

echo "Converting .UI files..."

ext=*.ui

for f in $ext; do
    filename="$f"
    name=${filename%.*}
    echo "Converting $filename to $name.py"
    pyuic5 "$filename" -o "$name".py
done


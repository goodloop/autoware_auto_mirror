#! /bin/bash

# Convert CSV files to the graphviz language and to an image from there

set -e

# output file type
file_type=pdf
# file_type=svg
# file_type=png

for f in *.csv
do
    echo "Converting $f..."
    python3 create_dot.py "$f"
    dot -T${file_type} "${f%.csv}.gv" -o "${f%.csv}.${file_type}"
done

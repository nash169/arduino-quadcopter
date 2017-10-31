#!/bin/sh

cd ../build &&
make &&
cp src/main ../run &&
cd ../run &&
echo "Program compiled (1. Run, 2. Do nothing) > "
read num
if [[ num -eq 1 ]]; then
  ./main
elif [[ num -eq 2 ]]; then
  echo "Good Bye"
fi

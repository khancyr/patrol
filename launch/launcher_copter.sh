#!/bin/bash
if [ "$#" -eq 0 ]
  then
    echo "No arguments supplied"
    exit
fi

for ((i=1 ; $i <= "$1"; i++))
do
        temp="--tab -t SIM${i} -e 'sim_vehicle.py -v ArduCopter -f quad -w --no-mavproxy -S 10 -I $(($i-1))  --sysid ${i} --use-dir quad${i} -N' "
        eval "gnome-terminal " "$temp"
        #echo "$temp"
        echo "Launching vehicle "${i}
        sleep 2
done

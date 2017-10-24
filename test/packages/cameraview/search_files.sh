#!/bin/bash


FILE_NOT_FONUD=64

while IFS='' read -r line || [[ -n "$line" ]]; do
    if [ ! -f "$line" ]; then
    	echo "File Not Found!" 1>&2
		exit $FILE_NOT_FONUD
	fi
done < "$1"



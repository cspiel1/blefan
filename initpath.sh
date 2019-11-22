#!/bin/bash

orgdir=$(pwd)

while [ ! -d esp-idf ]; do
	cd ..

	if [ `pwd` == "/" ]; then
	echo "esp-idf not found in upper directories"
	fi
done

spath=`pwd`

cd ${orgdir}

export IDF_PATH=$spath/esp-idf ; export PATH=$PATH:$spath/crosstool-NG/builds/xtensa-esp32-elf:$spath/xtensa-esp32-elf/bin:$spath/crosstool-NG/builds/xtensa-esp32-elf:$spath/xtensa-esp32-elf/bin

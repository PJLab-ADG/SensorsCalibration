#!/bin/bash
# This file is part of the SPLINTER library.
# Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

SPLINTER_DIR=$(cd `dirname "${BASH_SOURCE[0]}"` && pwd)/..
# Verify that CMakeLists.txt exists in $SPLINTER_DIR
if [ ! -f $SPLINTER_DIR/CMakeLists.txt ]; then
	echo "Error: Unable to locate CMakeLists.txt!"
	exit 1
fi

RED_COLOR="\033[0;31m"
GREEN_COLOR="\033[0;32m"
NO_COLOR="\033[0m"

NUM_MISSING=0
NUM_OK=0
TO_CHECK="$SPLINTER_DIR/CMakeLists.txt $SPLINTER_DIR/src $SPLINTER_DIR/include $SPLINTER_DIR/matlab $SPLINTER_DIR/test $SPLINTER_DIR/scripts $SPLINTER_DIR/python"
IGNORE="--hide=*.pyc --hide=__pycache__"

function print_red {
	echo -e "$RED_COLOR$1$NO_COLOR"
}

function print_green {
	echo -e "$GREEN_COLOR$1$NO_COLOR"
}


function check_license {
	while read p; do
		if ! grep -q "$p" $1
		then
			MISSING_LICENSE="true"
			return
		fi
	done < $SPLINTER_DIR/scripts/LICENSE_HEADER
}

function check_entry {
	if [ -f $1 ]; then
		check_file $1
	else
		check_directory $1
	fi
}

function check_file {
#	echo "Checking file: $1"

	MISSING_LICENSE="false"
	check_license $1
	if [ $MISSING_LICENSE == "true" ]
	then
		print_red "$(readlink -m $1)"
		NUM_MISSING=$(($NUM_MISSING + 1))
	else
		print_green "$(readlink -m $1)"
		NUM_OK=$(($NUM_OK + 1))
	fi
}

function check_directory {
#	echo "Checking directory: $1"

	for _ENTRY in $(ls $1 $IGNORE)
	do
		ENTRY=$1/$_ENTRY

		check_entry $ENTRY
	done
}

for __ENTRY in $TO_CHECK
do
	check_entry $__ENTRY
done

echo "$NUM_OK/$(($NUM_OK + $NUM_MISSING)) files has the license header."

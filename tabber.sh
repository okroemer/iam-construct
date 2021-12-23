#!/usr/bin/env bash

BASE_FOLDER=~/Prog
TABNAME="noname"
while getopts ":t:c:n:" opt; do
	case $opt in
		t) echo "Opening tab to exe: $OPTARG" >&2; 
		gnome-terminal --tab --title "$TABNAME" -- bash -c "cd $BASE_FOLDER; $OPTARG";
		;;
		c) echo "Using current tab to exe: $OPTARG" >&2; 
                bash -c "cd $BASE_FOLDER; $OPTARG";
                ;;
                n) echo "Use next tab name: $OPTARG" >&2; 
                TABNAME=$OPTARG;
                ;;
		\?) echo "Invalid option: $OPTARG" >&2; ;;

esac
done

#!/bin/sh
nice -n-19 ./main &
sleep 1
nice -n-19 ./cpc

#!/bin/bash

while true
do
 mraa-gpio set 38 0
 sleep 1
 mraa-gpio set 38 1
 sleep .01
done

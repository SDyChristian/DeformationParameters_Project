#!/bin/bash

clear
printf "Compiling\n\n" 
printf "lgsl lgslcblas has been removed\n\n"
g++ main.cpp keyboard_hit.c sleep_time.c -O2 -Wall -lpthread -lm -lphidget21

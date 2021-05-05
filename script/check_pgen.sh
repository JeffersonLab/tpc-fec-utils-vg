#!/bin/bash

if [ "$1" != "" ]; then
  LOGFILE=$1
else
  LOGFILE="log_pgen.txt"
fi

function exitTrap {
  STOP=`date +%s`
  DELTA=$(($STOP-$START))
  echo "Stopped: `date`" >> $LOGFILE
  echo "deltaT: $DELTA seconds" >> $LOGFILE
  echo "Results:" >> $LOGFILE
  echo "------------------------------------------------" >> $LOGFILE
  for i in `seq 0 11`;
  do
    trorc_gbtctrl -c $i -s >> $LOGFILE
  done
  echo "================================================" >> $LOGFILE
  echo -ne "\n" >> $LOGFILE
  exit 0
}

#trap "echo \"Stopped: `date`\" >> log_pgen.txt; exit 0;" SIGINT SIGTERM
trap "exitTrap" SIGINT SIGTERM
echo "================================================" >> $LOGFILE
echo "Started: `date`" >> $LOGFILE
START=`date +%s`
while [ true ]; do
clear
  for i in `seq 0 11`;
  do
    trorc_gbtctrl -c $i -s
  done
sleep 5;
done

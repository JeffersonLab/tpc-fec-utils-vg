#!/bin/bash

DEV=0
SCRIPTPATH=$(cd `dirname "${BASH_SOURCE[0]}"` && pwd)
LOGPATH=$SCRIPTPATH/log
BINPATH=$(which trorc_fec_readout)

for CH in {0..11}
do
  PID=${LOGPATH}/trorc_fec_readout_${DEV}_${CH}.pid
  if [ -f $PID ]; then
    echo "Terminating trorc_fec_readout on device ${DEV} channel ${CH}"
    kill -s 2 `cat $PID`
  else
    echo "No PID file found for device ${DEV} channel ${CH}"
  fi
done

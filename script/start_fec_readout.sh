#!/bin/bash
#
# Start 12 instances of trorc_fec_readout in background

DEV=0
SCRIPTPATH=$(cd `dirname "${BASH_SOURCE[0]}"` && pwd)
LOGPATH=$SCRIPTPATH/log
BINPATH=$(which trorc_fec_readout)

mkdir -p $LOGPATH

for CH in {0..11}
do
  PID=${LOGPATH}/trorc_fec_readout_${DEV}_${CH}.pid
  LOG=${LOGPATH}/trorc_fec_readout_${DEV}_${CH}
  DUMP=${LOGPATH}/data_${DEV}
  echo "Starting trorc_fec_readout on device ${DEV} Channel ${CH}"
  /usr/sbin/daemonize -o $LOG.log -e $LOG.err -p $PID -l $PID ${BINPATH} \
	--device $DEV --channel $CH --readoutframecount 200 \
	--continuous --quiet --dump ${DUMP}
done
sleep 1
cat ${LOGPATH}/*.err

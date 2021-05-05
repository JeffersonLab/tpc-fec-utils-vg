#!/bin/sh

function printUsage {
  echo "0: Data forwarding"
  echo "1: Static pattern"
  echo "2: Counter"
  echo "3: PBRS"
  echo "reset: Set everything to normal data taking mode and reset counters"
}

# If there is no arguments we print usage and exit
if [ "$1" = "" ]; then
  printUsage
  exit 1
fi

# Only do reset of system and exit
if [ "$1" = "reset" ]; then
  echo "Resetting pattern generator..."
  for i in `seq 0 5`; do
    bpgbtx.py --dev "/dev/ttyUSB${i}" --write 28 0
  done
  tfec -m 0x2 --gbtx0-pgen 0
  tfec -m 0x3f --gbtx1-pgen 0
  # Reset ctrl core
  for i in `seq 0 11`; do
    trorc_gbtctrl -c $i --pcerrorcount=0 --pcwidebusmode=0 --pcpatternmode=0 --rxreadylostcount=0
  done
  echo "Reset"
  exit 0
# Set pattern mode for setup in LaTotale with 1 rev1 FEC and 5 rev0a FECs:
elif [ "$1" -ge 0 -a "$1" -le 3 ]; then
  PATTERNMODE=$1
  echo "Configuring GBTx pattern generator with mode $PATTERNMODE..."
else
  echo "Pattern mode is not recognized..."
  printUsage
  exit 1
fi


# Set pattern generator mode for GBTx0
# On rev0a the SDA line is pulled high, so it needs to be done with bus pirate
# also do it for bus pirate connected to rev1, since there is no logical mapping of the tty's
for i in `seq 0 5`;
do
  bpgbtx.py --dev /dev/ttyUSB${i} --write 28 $PATTERNMODE
done;
# Only works for rev1, since rev0a has SDA pulled high on GBTx0
tfec -m 0x2 --gbtx0-pgen $PATTERNMODE 
tfec -m 0x3f --gbtx1-pgen $PATTERNMODE

# Set up the firmware pattern checker core 
for i in `seq 0 11`;
do
  trorc_gbtctrl -c $i --pcerrorcount=0 --pcwidebusmode=1 --pcpatternmode=$PATTERNMODE --rxreadylostcount=0
done;
echo "Configuration done"

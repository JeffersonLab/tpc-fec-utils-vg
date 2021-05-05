#!/bin/bash

# Figure out what devices are installed in the system (CRU or T-RORC)
#   Goal is to decide based on that which software to build
#
# Script will return a bit field with the following bits set to true, if
#   a card of the given type is detected in the system:
CRU_BITPOS=0
TRORC_BITPOS=1
#

CRU_VENDOR_CARD='1172:e001'
TRORC_VENDOR_CARD='10dc:01a0'

DEVICE_LIST=$(/sbin/lspci -nn | grep -i -E "${CRU_VENDOR_CARD}|${TRORC_VENDOR_CARD}")

# # Some default strings for testing
# DEVICE_LIST='04:00.0 Signal processing controller [1108]: CERN/ECP/EDU Device [10dc:01a0]
# 05:00.0 Signal processing controller [1108]: CERN/ECP/EDU Device [10dc:01a0]'
# DEVICE_LIST_CRU='06:00.0 Non-VGA unclassified device [0000]: Altera Corporation Device [1172:e001]
# 07:00.0 Non-VGA unclassified device [0000]: Altera Corporation Device [1172:e001]'
# DEVICE_LIST="$DEVICE_LIST
# $DEVICE_LIST_CRU"

#
# Generate bitfield to return from detected devices
CARDS_PRESENT=0

while IFS= read -r line
do
  MATCH=$(expr match "$line" '.*\[\([a-fA-F0-9]*:[a-fA-F0-9]*\)\]')
  if [ x"$MATCH" == x"$CRU_VENDOR_CARD" ]; then
    CARDS_PRESENT=$((CARDS_PRESENT|(1<<CRU_BITPOS)))
  elif [ x"$MATCH" == x"$TRORC_VENDOR_CARD" ]; then
    CARDS_PRESENT=$((CARDS_PRESENT|(1<<$TRORC_BITPOS)))
  fi
done<<EOF
$DEVICE_LIST
EOF

#
# Generate some readable output
case $CARDS_PRESENT in
  $((1 << CRU_BITPOS)) )
    echo -ne "CRU"
    ;;
  $((1 << TRORC_BITPOS)) )
    echo -ne "TRORC"
    ;;
  $(((1 << CRU_BITPOS) | (1 << TRORC_BITPOS))) )
    echo -ne "CRU_TRORC"
    ;;
esac

#
# Return code
exit $CARDS_PRESENT

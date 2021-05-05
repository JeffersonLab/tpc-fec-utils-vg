#!/bin/bash

#
# Script takes (alternative) FEC mask as first argument
#

FEC_MASK=0x3
if [ ! -z $1 ]; then
  FEC_MASK=$1
fi

BINDIR=/usr/local/bin
VERBOSE=

echo "=== HDLC STATUS ==="
${BINDIR}/tsca ${VERBOSE} --mask ${FEC_MASK} \
               --status

echo
echo "=== FEC STATUS ==="
${BINDIR}/tfec ${VERBOSE} --mask ${FEC_MASK} \
               --status
${BINDIR}/tfec ${VERBOSE} --mask ${FEC_MASK} \
               --gbtx-scan

echo
echo "=== SAMPA STATUS ==="
${BINDIR}/tsampa --mask ${FEC_MASK} \
                 --scan

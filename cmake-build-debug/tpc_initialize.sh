#!/bin/bash
#
# Script takes (alternative) FEC mask as first argument
#
function eval_rc {
  if [ $1 -ne 0 ]; then
    echo "ERROR. Exiting."
    exit $1
  else
    echo "done"
  fi
}

function show_help {
  cat << EOF
TPC FEC initialization script
Usage:
  $(basename $1) <options>
Options:
  -h | --help   Show this help
  --version     Show version information
  -m | --mask   Specify FEC mask (default: $FEC_MASK)
  --rev0        Adjust configuration for rev0 boards,
                  i.e. SAMPA_MASK=0x1e
EOF
  exit 0
}

function show_version {
  cat << EOF
Version information
  branch : master
  hash   : 6ddac21
  clean  : no
  build  : gurjyan@vapapi.jlab.org

EOF
  exit 0
}

FEC_MASK=0x3
SAMPA_MASK=0x1f

GBTX1_FILE=/usr/local/config/fec-gbtx1-cfg-wb-wd-to.txt
BINDIR=/usr/local/bin
VERBOSE=-v

#-------------------------------------------------------------------------------
# Command line parsing
CMDTEMP=`getopt -o m:h --long help,version,mask:,sampa-mask:,rev0 -n "$FUNCNAME" -- "$@"`
if [ $? != 0 ]; then echo "ERROR: Option parsing. Exiting" >&2; exit 1; fi
eval set -- "$CMDTEMP";

while true; do
  if [ -z $1 ]; then break; fi
  case ${1} in
    -h|--help)
        show_help $0; shift;
        ;;
    -m | --mask)
        FEC_MASK="$2"; shift 2;
        ;;
    --sampa-mask)
        SAMPA_MASK="$2"; shift 2;
        ;;
    --rev0)
        SAMPA_MASK=0x15; shift;
        ;;
    --version)
        show_version; shift;
        ;;
    --)
        shift; break;
        ;;
  esac
done

if [ $# -ne 0 ]; then
  echo "ERROR: Unrecognized option(s): $@"
  exit 100
fi

is_number='^[0-9xXa-fA-F]+$'
if ! [[ $FEC_MASK =~ $is_number ]]; then
  echo "ERROR: Mask argument is invalid: $FEC_MASK"
  exit 1
fi

if ! [[ $SAMPA_MASK =~ $is_number ]]; then
  echo "ERROR: Sampa mask argument is invalid: $SAMPA_MASK"
  exit 1
fi


#------------------------------------------------------------------------------
echo "Configuring SCA cores..."
${BINDIR}/tsca ${VERBOSE} --mask ${FEC_MASK} \
               --core-init
eval_rc $?

echo "Running FEC init"
${BINDIR}/tfec ${VERBOSE} --mask ${FEC_MASK} \
               --init
eval_rc $?

echo "Configuring GBTx1 with ${GBTX1_FILE}"
${BINDIR}/tfec ${VERBOSE} --mask ${FEC_MASK} \
               --gbtx1-cfg ${GBTX1_FILE}
eval_rc $?

echo "Turning on SAMPA power"
${BINDIR}/tsampa ${VERBOSE} --mask ${FEC_MASK} \
                 --sampa-mask ${SAMPA_MASK} \
                 --pwr-on --pwr-wait 10
eval_rc $?

sleep 1

echo "Turning on 11th SAMPA elink"
${BINDIR}/tsampa ${VERBOSE} --mask ${FEC_MASK} \
                 --sampa-mask ${SAMPA_MASK} \
                 --elinks 11
eval_rc $?

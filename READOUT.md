# T-RORC Utils / TPC FEC Readout Usage Examples

### Overview
Some of the tools, and what they're supposed to do:
- `trorc_reset`: Perform hard reset of T-RORC
- `trorc_init`: Initialize T-RORC, especially make GBTx operational
- `tsca`: Control and configure slow control communication core on the T-RORC
- `tfec`: FEC utility, e.g. for initalization of FEC, GBTx(1) programming, ...
- `tsampa`: SAMPA utility, e.g. SAMPA power control, I2C scan, ...
- `treadout`: Readout utility capable of handling multiple FECs/DMA channels
- `trorc_dma_monitor`: T-RORC DMA performance monitor
- `trorc_fec_readout_control`: Low-level readout control
- `trorc_fec_readout`: More low-level readout utility targeting a single DMA channel (predecessor of `treadout`, with more freedom wrt debugging)
- `tpc_initialize.sh`: Script to quickly initialize the setup (wraps some of the tools above)

### Setting it up
- Full T-RORC reset
```bash
trorc_reset
```
<p/>

- Initialize the GBT links on the T-RORC
```bash
trorc_init
```
Make sure that the GBTx0s on the associated FECs, or GBTx on the VLDBs in case of the SAMPA test stand, are configured/fused
with a configuration that has watchdog and timeout enabled prior to running `trorc_init`. Configuration files for the GBTx
in the system are available in the [config](../config) folder.
<p/>
Running `trorc_init` on a system with 2 FECs connected on links 0..3 should produce an output like
```bash
tpc@t-rorclab ~ $ trorc_init
*** Initializing GBTs on FEC ***
Resetting QSFPs...
Resetting Transceivers...
Reconfiguring Reference Clock to 240 MHz...
Releasing Transceiver Resets...
Releasing QSFP Resets...
Waiting 10s for GBT RX READY...
RX READY received on GBT0
RX READY received on GBT1
RX READY received on GBT2
RX READY received on GBT3
NO RX READY on GBT4!
NO RX READY on GBT5!
[...]
```
and confirm that the transceivers indicate an `RX READY` for the GBTxs on the connected FECs.
<p/>

- To establish the communcation with the GBT SCA on the FEC/VLDB, use the `tsca` utility. It allows to reset/initialize the SCA communication core on the T-RORC and send a supervisory-level CONNECT frame to the SCA by doing
```
tsca --mask 0x3 --core-rst --core-init --svl-connect
```
where `--mask` specifies the FEC/VLDB mask, i.e. these commands would be executed for the SCA communcation cores and GBT SCAs on CH0 and CH2.
Note that on the VLDB a cable is required to connect the GBT SCA output port to the SCA primary or secondary input port.

- Depending on which communication interfaces of the GBT SCA are to be used, the GBT SCA must be configured to enabled the corresponding
channels. The `tsca` tool (see `-h` for a full list of options) may be used to do that, e.g. to set the configuration registers B and D
on the SCA connected via CH2
```
tsca --mask 0x2 --sca-cfg-b 0xa --sca-cfg-d 0xf
```

### Wrapper Script for Initalization of the FEC
- Initialize the SCA communication core, configure the FECs and SAMPAs for FECs 0 and 2 (mask 0x5).
The script wraps a number of the aforementioned tools, and requires GBTx0 on the FECs to be configured
and operating correctly
```bash
tpc_initialize.sh --mask 0x5
```
<p/>

- Same thing as before, but we have only one rev0 FEC (i.e. SAMPA mask 0x15 instead of 0x1f), connected to DMA channels 2 & 3
```bash
tpc_initialize.sh --mask 0x2 --rev0
```


### Readout and Triggering with T-RORC/FEC
- Use `treadout` and 2 software-initiated triggers to read out a SYNC plus 2 normal events from FECs 1 and 2
```bash
# Start readout and request a SYNC as first event
treadout --nr 4 --output-dir /tmp/testdata --mask 0x6
# On another console, send two triggers in software
trorc_fec_readout_control -t
trorc_fec_readout_control -t
```
At the start of the readout, `treadout` will potentially try to create the output directory and/or
check the write permissions. The output files produced are placed in `<output-dir>/run<nr>/`, in
this case `/tmp/testdata/run000004/`, and their names will be of the form `run<%06d>_trorc<%02d>_link<%02d>.bin`.
<p/>
The program typically requests a SYNC event at startup and records it as first event in each file.
<p/>
The enable for the external T-RORC trigger input (or its internal hardware trigger generator)
is activated at the start of the program, and gets disabled when it exits.
<p/>
For correct operation, instances of `trorc_fec_readout` must not be running when using `treadout`.
<p/>

- Readout events with `treadout` for 6 FECs with no SYNC event as first event
```bash
treadout --nr 5 --output-dir /tmp --mask 0x3f --no-sync
```
<p/>

- Readout a fixed number of events, for example 42, with `treadout` and software trigger
```bash
treadout --nr 6 --output-dir /tmp --mask 0x3f --events 42
# On another console, send the triggers
for i in `seq 0 40`; do trorc_fec_readout_control -t; done
```
<p/>

- Run `treadout` for FECs 0 and 1 until interrupted by the user and record 480 GBT frames per T-RORC readout gate opening
```bash
treadout --nr 7 --output-dir /mydata/endless --mask 0x3 --frames 480 --events 0
# Send a trigger, i.e. open the readout gate
trorc_fec_readout_control -t
[...]
# Then use ctrl-C to gracefully shut down the readout
```

### Readout for the SAMPA Tester (T-RORC/VLDB)
- The utility `treadout` has been adapted for use with the SAMPA tester setup. In this case the option `--sampa-tester` is required.
It will disable readout of the odd-numbered links and adjust the IDLE and CTRL downlink patterns for the readout with the SAMPA test stand.

- The directory structure created/used by `treadout` is as follows: Files written by `treadout` are placed in
`<output-dir>/<nr-prefix><nr>/`, the files names will be `<nr-prefix><nr %06d>_trorc<%02d>_link<%02d>_<fn_postfix>.bin`.
The options `<nr>` (SAMPA number), `<nr-prefix>`, `<fn-postfix>` (some arbitrary identifier) can be passed on the command line.

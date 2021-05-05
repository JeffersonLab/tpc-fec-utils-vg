## T-RORC Notes and Snippets


#### Programming
Make sure Xilinx lab tools are installed.

Source settings script or do `module add xil/141` to load tools and program FPGA with
`usbupload [.bit or url]`<br/>

Initiate PCIe rescan:<br/>
`crorc_rescan_pci` (as root)<br/>

Then do `trorc_init` to reset QSFP, GTX, set refclk, etc.



#### Flashing
`crorc-flash -n0 -c[0|1] -w [.bin] -v`<br/>
Once done, powercycle the machine.

Alternatively, make the FPGA refetch the design from the flash (in software - might not work on all machines):<br/>
`/opt/packages/crorc-smbus-supermicro/crorc_smbus_reconfigure_fpga.sh 0 0x40 [0|1]`<br/>
and initiate PCIe rescan:<br/>
`crorc_rescan_pci` (as root)

Once the machine is up again, do a
`trorc_init` to reset QSFP, GTX, set refclk, etc.



#### Snippets and Notes
* Get T-RORC revision and build date<br/>
`crorc_fpga_ctrl -l`

* Control GTX loopback<br/>
Display value: `crorc_fpga_ctrl --gtxloopback`<br/>
Set value: `crorc_fpga_ctrl --gtxloopback=value`<br/>
Show GTX status in WebGUI<br/>
`/cgi-bin/pystatus/status.py?verbose=1`<br/>
Values are:
  - 0  Normal operation
  - 1  Near-end PCS loopback
  - 2  Near-end PMA loopback
  - 3  Reserved
  - 4  Far-end PMA loopback
  - 5  Reserved
  - 6  Far-end PCS loopback

* VLDB with SPF needs TX polarity = 1, in that case run ```trorc_init --vldb```

* Dump GBT data to file and display it<br/>
`trorc_fec_readout -C 0x10 -R 0x1000 -d /tmp/`<br/>
`od -tx4 /tmp/dev0_ch0_0.ddl | less`

* Enable GBT pattern checking mode<br/>
`trorc_gbtctrl --pcpatternmode=1` 
  * `--pcpatternmode [value]`: 1: static pattern, 2: counter patter, 3: prbs pattern)
  * `--pcwidebusmode [value]` : 0: pattern check on GBT frame, 1: pattern check on GBT frame + Widebus word


#### Directories, Files and Scripts
> type checkout_librorc<br/>
> type build_librorc_local<br/>
> type checkout_crorc_utils<br/>
> type build_crorc_utils_local<br/>
> type build_trorc_utils_local<br/>

> $LIBRORC_SRC<br/>
> $LIBRORC_INSTALL<br/>
> $CRORC_UTILS_SRC<br/>
> $CRORC_UTILS_INSTALL<br/>
> $TRORC_UTILS_SRC<br/>
> $TRORC_UTILS_INSTALL<br/>

> /opt/HLT/operator<br/>
> /opt/HLT/operator/setenv.sh<br/>

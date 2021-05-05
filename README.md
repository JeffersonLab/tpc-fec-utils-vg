# TPC Run3 Readout Utilities
Tools and Utilities to configure and readout out the TPC FEC, either with the T-RORC
or the CRU.

### Usage Examples
Some examples for using the tools can be found in [READOUT](READOUT.md)

### Notes / Snippets
For some (maybe useful) notes and snippets, see [NOTES](NOTES.md)

### Tools
Common tool options are:
  * `-h | --help`<br/>Displays the tool help
  * `--version`<br/>Display version information extracted from the system and git
  * `-m | --mask <mask>`<br/>Specify bit mask of FECs/SCAs to apply the commands to

With the possibility to have multiple CRUs in a server, additional tool options are
required for the CRU:
  * `--id <bdf or serial>`<br/>    Specify card to address, either by serial number of PCI bdf notation
  * `-b | --bar <bar_number>`<br/> Specify bar (typically 2 in case of CRU)

### Setup
- CRU<br/>

- T-RORC<br/>
  The software expects the following T-RORC wiring schemes<br>
  For the FEC:
  ```
  FEC0, GBTx0 --> DMA CH0
  FEC0, GBTx1 --> DMA CH1
  FEC1, GBTx0 --> DMA CH2
  FEC1, GBTx1 --> DMA CH3
  ...
  ```
  For the SAMPA test stand:
  ```
  VLDB0, GBTx --> DMA CH0
    (open)    --> DMA CH1
  VLDB1, GBTx --> DMA CH2
    (open)    --> DMA CH3
  ...
  ```
  The QSFP connector for the first four channels is the one closest to the LED.
  <p/>
  SCA cores are instantiated for every other GBT link, i.e. all the even link/DMA indices.
  For example with the parameter `-m 0x3`, the tools will communicate with the SCA
  cores/SCA related to FEC0-GBTx0 and FEC1-GBTx0, or VLDB0 and VLDB1 in case of the SAMPA test stand.

### Build System
- The `cmake` build flow has been adapted to enable builds for CRU and T-RORC<br/>
  By default, the build system tries to autodetect the first readout card present in the system and
  builds the corresponding libraries/tools. Autodetect is equivalent to
  ```
  cmake -DBUILD_FOR_READOUT_CARD=AUTODETECT <cmake-top-dir>
  ```
  However, building of the tools for a specific readout card variant can be forced with (required
  libraries must of course be available/findable on the system)
  ```
  cmake -DBUILD_FOR_READOUT_CARD=TRORC <cmake-top-dir>
  cmake -DBUILD_FOR_READOUT_CARD=CRU <cmake-top-dir>
  ```
  
- Utilities specific to SAMPA test stand<br/>
  Some binaries/features, which are specific to the SAMPA test stand, are not built by default.
  Building them can be enabled by setting the `cmake` variable `BUILD_SAMPA_TESTER` to `ON`
  (use `ccmake .` in the `cmake` build directory, change the variable, press `c` `g` to configure and regenerate)

- T-RORC<br/>
  - Tools on the T-RORC are usually built against the boost libraries provided by the system
  - Build requires a somewhat modern compiler, install e.g. gcc5 via software collections (gcc6 doesn't work with boost-1.53.0 because of bug in boost::multiprecision)
   
    ```
    sudo yum install centos-release-scl-rh
    sudo yum install devtoolset-4-gcc-c++
    . /opt/rh/devtoolset-4/enable
    ```


- CRU
  - Build requires the ReadoutCard library installed and in the path. That typically means
    that also the gcc shipped with O2 and boost libraries shipped with O2 are in the path.

- In case of trouble with building the software, have a look at the [CI yaml file](./.gitlab-ci.yml).
Most clues are probably in there.

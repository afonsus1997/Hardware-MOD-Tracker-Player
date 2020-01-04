# Hardware MOD Tracker Player

## How to setup the ST ARM toolchain and IDE

* [Download the GNU toolchain from arm](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)

* Uninstall old packages 

```sudo apt remove binutils-arm-none-eabi gcc-arm-none-eabi libnewlib-arm-none-eabi```

* Untar package to desired directory (ex: ~/arm/)

```tar -xjvf gcc-arm-none-eabi-xxxxxxxxx```

* Add toolchain to path

```export PATH=$PATH:"PATH TO TOOLCHAIN"/bin/```

### VSCODE Setup

* Install ```stm32-for-vscode``` plugin

* Install ```Cortex-Debug extension``` plugin

### Install STLINK drivers

* Install Libusb

```sudo apt-get install libusb-1.0-0-dev```

* Install stlink ultility

```git clone https://github.com/texane/stlink stlink.git
cd stlink
make
#install binaries:
sudo cp build/Release/st-* /usr/local/bin
#install udev rules
sudo cp etc/udev/rules.d/49-stlinkv* /etc/udev/rules.d/
#and restart udev
sudo udevadm control --reload```

### Install openOCD

```sudo apt-get install openocd```

### Install STM32CUBEMX (Optional but recommended)

For pinmaps, code generation, etc...

* [Download STM32CUBEMX](https://my.st.com/content/my_st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-configurators-and-code-generators/)

* unzip and cd to extracted

* Mark binary as executable

```chmod +x SetupSTM32CubeMX-x.x.x.linux```

* Run binary

```./SetupSTM32CubeMX-x.x.x.linux```

* Go through installation process on GUI

## How to flash STM project

### Using VScode

* Open project folder

* Open command list (Ctrl+Shift+p)

* Type ```stm32``` and choose an option

### Using make

* Build

```make -f STM32Make.make```

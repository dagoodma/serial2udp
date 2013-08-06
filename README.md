# serial2udp

This program converts a serial port into a UDP port via software. It is a multi-threaded app that reads/writes from both a serial port and a UDP port merely passing the data through. It is used for various autonomous systems projects that require interfacing with Simulink to avoid issues that the serial port has on some platforms.

## License/distribution

This project follows the terms of the Mozilla Public License v2.0. Any distribution of the source code of this project must follow those licensing terms.

## Usage

This program is GUI-less and runs directly from the command line. Details on available options is available when running with '-h' or '--help' as arguments.

NOTE: You should specify a packet size for both the serial and UDP lines to get optimal performance. This will specify how many bytes the program waits for before triggering an interrupt on the serial side. On the UDP
side it must be larger than the largest expected UDP datagram size otherwise the program may not work properly.

### SLUGS Parser

This version of serial2udp has been modified for use as a communication bridge for the SLUGS autopilot while in hardware in the loop (HIL) simulation. This invovles converting UDP packets from the HIL simulation model in Simulink to MAVLink messages to be sent to the autopilot. The parser will also parse MAVLink messages from the autopilot and pack them into UDP packets to be sent to Simulink. 

To accomplish this, many arguments were added to serial2udp to allow for separate UDP receive and send sockets. This was tested and works on Windows 7 with a Visual Studio 2010 project. Additional folders were added to the include path to make this work. To obtain the Visual 2010 project as is, see the 'slugs_parser_vs10' branch. If using the existing project, then be sure to change the include directories to point to the proper locations (under Properties->C/C++->General->Additional Include Directories).

## Building

### Windows 64-bit or 32-bit

Prerequisites:
 - Compiler: Only tested with Microsoft Visual Studio 2010
 - Boost library 
 - MAVLink library with SLUGS messages
 - inttypes.h for MAVLink

#### Install Boost (http://www.boost.org/)
1. Download the Boost library from boost.org
2. Extract the library to C:\boost_1_51_0 (changing 1_51 to whatever your version is)
3. (Following the build instructions for x86-64 1.51) Open a command prompt and enter the root directory of the Boost files.
4. Run `bootstrap.bat`
5. Compile for x86-64 by running `.\b2 --toolset=msvc-10.0 variant=release link=static threading=multi runtime-link=static architecture=x86 address-mode=64`
  - Remove 'address-mode=64' if compiling for x86.
  - This will build only the multi-threaded statically-linked release build that is necessary for this project. If you'd like to use this Boost installation for debugging or other projects you'll probably want to just build everything by switching out the 'variant=release link=static threading=multi runtime-link=static' options for '--build-type=complete' instead.

#### Building the program

I have only tested this program on Windows 7 and as such have no instructions for compiling on other platforms. This program only relies on standard library functions as well as the cross-platform C++ library Boost and as such should be retargetable to many platforms.

I only include instructions for building this project for x64 and x86 as that is my primary target. 

1. Create a new MSVC++ 2010 Win32 Console Application project
  - Specify Empty Project within the wizard
2. Add 'serial2udp.cpp' and 'slugs_parser\SlugsMavlink.cpp' to the project by right-clicking on the Sources folder in the Solution Explorer.
3. Add 'slugs_parser\Packing.h' and 'slugs_parser\SlugsMavlink.h' to the project by right-clicking on the Header Files folder in the Solution Explorer.
4. Visual Studio does not support C types, so inttypes.h must be obtained and copied into the root of your project's source folder (ie. 'MyProject/MyProject' if your VS2010 project were called 'MyProject'). This header can be found under the 'slugs_parser_vs10' branch at 'HILBridgeClient\HILBridgeClient\inttypes.h'.
5. Specify building for an x64 release target. Skip this step if you are using x86, and just change 'Debug' in the Solutions Configurations dropdown menu to 'Release'.
  - Build->Configuration Manager->Active Solution Platform->New...->OK
  - Under 'Platform'  select the new x64 target.
  - Under 'Configuration' select 'Release'
6. Modify the project to add the folder with inttypes.h ('.\' if following step 4), the root Boost folder ('C:\boost_1_51_0' if following the above instructions), the slugs_parser folder ('..\..\slugs_parser'),  and the root of the MAVLink include folder (with SLUGS generated headers) to the header path
  - Properties->C/C++->General->Additional Include Directories
7. Modify the project to add the Boost libraries for linking ('C:\boost_1_51_0\stage\lib\' if following the above instructions)
  - Properties->C/C++->Linker->General->Additional Library Directories
8. Disable use of precompiled headers
  - Set Properties->C/C++->Precompiled Headers to 'Not Using Precompiled Headers'
9. Set compilation for a statically-linked multi-threaded library
  - Set Properties->C/C++->Code Generation->Runtime Library to 'Multi-threaded (/MT)'
10. Build it
  - Ctrl-Shift-B or Build->Build Solution
11. Run it
  - The executeable will be in '/x64/Release/' or under '/Release/' for x86.


### Mac OS X 10.6.8 (Snow Leopard) 32-bit (i386)

The SLUGS parser has not been tested on Mac, and instructions to compile with MAVLink are not shown below. You will have to figure this out on your own.

Prerequisites:
 - Compiler: Only tested with g++
 - Boost library 

#### Install Boost (http://www.boost.org/)

Installing Boost with a package manager such as MacPorts, Fink, or Homebrew should also work, but I kept seeing 'unsupported file format' warnings and undefined symbols when attempting to link the Boost libraries. Thus it is recommended to build Boost from source: 

1. Download the Boost library from boost.org
2. Extract the library to a folder in your user directory (ie. ~/Downloads)
3. (Following the build instructions for x86 1.50) Open a terminal window (Applications > Utilities > Terminal) and change to the root directory where you extracted the Boost files.
4. Run `./bootstrap.sh`
5. Compile for x86 by running `sudo ./b2 variant=release link=static threading=multi runtime-link=static architecture=x86`
  - This will build only the multi-threaded statically-linked release build that is necessary for this project. Also, I was seeing permission denied messages without 'sudo'.

#### Building the program

I have only tested this program on 32-bit Snow Leopard, but should be retargetable to other versions of Mac OS X.

1. In a terminal window change into the directory where 'serial2udp.cpp' is.
2. Compile by running `g++ -v -m32 -I/Users/USERNAME/Downloads/boost_1_50_0 serial2udp.cpp -o serial2udp`
    `-L/Users/USERNAME/Downloads/boost_1_50_0/stage/lib -lboost_program_options -lboost_date_time -lboost_system -O2`
   * Note that you'll need to use a full path for the `-L` and `-I` and cannot use `~`.
3. Run it
  - The binary is called 'serial2udp'

### Mac OS X 10.7.4 (Lion) 64-bit (x86-64)

Prerequisites:
 - Compiler: Only tested with g++
 - Boost library

#### Install Boost

Follow the directions for 32-bit OS X except for the following changes:
1. In step 5 add the address-model argument `address-model=64`

#### Building serial2udp 

1. Compile like in Step 2 for 32-bit OS X, but specify `-m64` instead for 64-bit addressing.

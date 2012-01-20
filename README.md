# serial2udp

This project converts a serial port into a UDP port in software. It is used for various autonomous systems projects that require interfacing with Simulink to avoid issues with the serial port on some platforms.

## License/distribution

This project follows the terms of the Mozilla Public License v2.0. Any distribution of the source code of this project must follow those licensing terms.

## Usage

This program is run on the command line and all CLI options are enumerated when the program is run without options. Specifying -h or --help at the command line will also list all options.
Be sure to specify a packet size for both the serial and UDP lines. This size will specify how many bytes the program waits for before triggering an interrupt on the serial side. On the UDP
side it must be larger than the largest expected UDP datagram size or the program may not work properly.

## Building

Prerequisites: Proper compiler, Boost library

### Install Boost
1. Download the Boost library from boost.org
2. Extract the library to a specified location
3. Compile the boost library following step 5 in the "Getting Started with Boost in Windows" page.
  - You only need to build for a static multi-threaded release library. So compiling should look something like: `.\b2 variant=release link=static threading=multi runtime-link=static`

### Building the program

I have only tested this program on 32-bit Windows and as such have no instructions for compiling on other platforms. This program only relies on standard library functions as well as the cross-platform C++ library Boost and as such should be retargetable to many platforms.

#### Windows 32-bit

1. Install Microsoft Visual Studio/C++ 2010
  - Available free from https://www.microsoft.com/visualstudio/en-us

2. Create a new MSVC++ 2010 Win32 Console Application project
  - Specify Empty Project within the wizard
3. Modify the project to add the root Boost folder to the header path
  - Under Properties->C/C++->General->Additional Include Directories
4. Modify the project to use the static non-debug version of the Boost library
  - Under Properties->C/C++->Code Generation->Runtime Library
5. Modify the project to add the Boost libraries for linking
  - Under Properties->C/C++->Linker->General->Additional Library Directories
6. Set the version of Windows being compiled for to Windows XP
  - Add "-D_WIN32_WINNT=0x0501" to Properties->C/C++->Command Line->Additional Options
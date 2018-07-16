
# MPC Forklift Simulator

Simulator created to experiment with Model Predictive Control of forklifts.

## Resources

**Udacity course code** - https://github.com/udacity/CarND-MPC-Quizzes/tree/master/mpc_to_line/solution

**MPC Car Simulator** - https://github.com/mithi/mpc

## Tools

**MingW** - C++ Compiler

**CodeBlocks** - IDE

## Dependencies

All of these dependencies needs to be configured in the Code Blocks IDE. Only IPOPT needs to be compiled from source, the rest are either header only libraries or have precompiled binaries available. Tested on 64 bit Windows 10. You probably need to change some paths in the commands because your username is not the same as mine.

**SFML** -  For drawing 2D graphics

https://www.sfml-dev.org/

Download: https://www.sfml-dev.org/files/SFML-2.5.0-windows-gcc-7.3.0-mingw-32-bit.zip

In CodeBlocks do as follows:

Build options -> Search directories -> Compiler: C:\Users\bt6073\Desktop\SFML-2.5.0\include

Build options -> Search directories -> Linker: C:\Users\bt6073\Desktop\SFML-2.5.0\lib

Important to put DDL-files (sfml-system-2.dll, sfml-window-2.dll, sfml-graphics-2.dll) beside the executable in the project (forklift-mpc/bin/Debug/MPC.exe). The DLL-files are found in (SFML-2.5.0/bin/)-folder from the download.

**Eigen** - For linear algebra calculations

http://eigen.tuxfamily.org/index.php?title=Main_Page

Download: http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2

In CodeBlocks do as follows:

Build options -> Search directories -> Compiler: C:\Users\bt6073\Desktop\eigen-eigen-5a0156e40feb

No linked libraries (header only library)

**IOPT** - Optimization

https://projects.coin-or.org/Ipopt

This took me several days to get right... It was difficult to compile. Just read the instructions on the website carefully, I'm not sure what actually made it work for me in the end.

Download: https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.10.zip

Extract to desktop.

Follow the instructions for "Installation with MSYS/MinGW": https://www.coin-or.org/Ipopt/documentation/node15.html

Install MinGW.

Start cmd terminal and type "sh", press enter.

Navigate to: /c/Users/bt6073/Desktop/Ipopt-3.12.10/

```
mkdir build
mkdir build/prefix
mkdir build/lib64
cd ThirdParty
```

Run scripts to download third party dependencies. Make sure they all say OK. You might have problems with certificates, then you might have to edit the scripts and add flags to ignore certificates. Don't proceed until all dependecies are resolved OK.

```
cd Mumps
./get.Mumps
cd ..
cd Metis
./get.Metis
cd ..
cd Lapack
./get.Lapack
cd ..
cd Blas
./get.Blas
cd ..
```

Now build IPOPT.

```
cd build
../configure --enable-debug --prefix=/c/Users/bt6073/Desktop/Ipopt-3.12.10/build/prefix --libdir=/c/Users/bt607
3/Desktop/Ipopt-3.12.10/build/prefix/lib64
make
make install
```

In CodeBlocks do as follows:

Build options -> Search directories -> Compiler: C:\Users\bt6073\Desktop\Ipopt-3.12.10\build\prefix\include

Build options -> Search directories -> Linker: C:\Users\bt6073\Desktop\Ipopt-3.12.10\build\prefix\lib64

** Link libraries in CodeBlocks **

Add the following to Build options -> Linker settings -> Link libraries

The ordering matters!

sfml-graphics
sfml-window
sfml-system
ipopt
coinlapack
coinmumps
coinblas
coinmetis
gfortran

**Done!**

PS. I also tried to compile using TDM-GCC but I don't think that was used in the end. I'm leaving some instructions here just in case you want to try.

Install TDM-GCC.

Make sure TDM-GCC is used by editing: C/MinGW/msys/1.0/etc/fstab

It should say: C:/TDM-GCC-64     /mingw

Start: C:\MinGW\msys\1.0\msys.bat

**CppAD** - For interfacing with IPOPT

https://github.com/coin-or/CppAD

Download: https://www.coin-or.org/download/source/CppAD/cppad-20180000.0.gpl.tgz

Build options -> Search directories -> Compiler: C:\Users\bt6073\Desktop\cppad-20180000.0

No linked libraries (header only library)

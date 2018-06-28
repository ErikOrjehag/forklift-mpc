
# MPC Truck Simulator

Simulator created to experiment with Model Predictive Control.

## Resources

*MPC Car Simulator*
https://github.com/mithi/mpc

## Tools

*MingW* - C++ Compiler
*CodeBlocks* - IDE

## Dependencies

All of these dependencies needs to be configure. None of them needs to be compiled from source. Tested on 64 bit Windows 10.

*SFML* - Graphics
https://www.sfml-dev.org/
Ladda ner: https://www.sfml-dev.org/files/SFML-2.5.0-windows-gcc-7.3.0-mingw-32-bit.zip
Build options -> Search directories -> Compiler: C:\Users\bt6073\Desktop\SFML-2.5.0\include
Build options -> Search directories -> Linker: C:\Users\bt6073\Desktop\SFML-2.5.0\lib

*Eigen* - Math
http://eigen.tuxfamily.org/index.php?title=Main_Page
Ladda ner: http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2
Build options -> Search directories -> Compiler: C:\Users\bt6073\Desktop\eigen-eigen-5a0156e40feb
No linked libraries

*IOPT* - Optimization
(Detta tog mig flera dagar att få rätt)
https://projects.coin-or.org/Ipopt
Ladda ner: https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.10.zip
Extrahera till Desktop.
Följ instruktionerna "Installation with MSYS/MinGW": https://www.coin-or.org/Ipopt/documentation/node15.html
Installera MinGW /*och TDM-GCC. Se till att TDM-GCC används genom att redigera:
C/MinGW/msys/1.0/etc/fstab
Det ska stå: C:/TDM-GCC-64     /mingw
Starta: C:\MinGW\msys\1.0\msys.bat*/
Starta cmd och skriv "sh" tryck enter.
Navigera till: /c/Users/bt6073/Desktop/Ipopt-3.12.10/
mkdir build
mkdir build/prefix
mkdir build/lib64
cd ThirdParty
Se till att dessa skript körs OK
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
cd build
../configure --enable-debug --prefix=/c/Users/bt6073/Desktop/Ipopt-3.12.10/build/prefix --libdir=/c/Users/bt607
3/Desktop/Ipopt-3.12.10/build/prefix/lib64
make
make install
I CodeBlocks gör följande:
Build options -> Search directories -> Compiler: C:\Users\bt6073\Desktop\Ipopt-3.12.10\build\prefix\include
Build options -> Search directories -> Linker: C:\Users\bt6073\Desktop\Ipopt-3.12.10\build\prefix\lib64
Lägg till följande i Build options -> Linker settings -> Link libraries (ordningen spelar roll!):
ipopt
coinlapack
coinmumps
coinblas
coinmetis
gfortran
Klar!

*CppAD* - Interface IPOPT
https://github.com/coin-or/CppAD
Ladda ner: https://www.coin-or.org/download/source/CppAD/cppad-20180000.0.gpl.tgz
Build options -> Search directories -> Compiler: C:\Users\bt6073\Desktop\cppad-20180000.0
No linked libraries...

*NanoGUI* - User interface elements (Not used yet... but want to in the future...)
https://github.com/wjakob/nanogui

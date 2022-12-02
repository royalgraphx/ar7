#!/bin/bash
mingw=D:/a/_temp/msys64/mingw64/
sh ../configure --disable-guest-agent-msi --disable-werror --enable-sdl --enable-opengl --enable-virglrenderer --enable-system --target-list=x86_64-softmmu --enable-whpx --enable-gtk --enable-libusb --extra-cflags="-I$mingw/include" --extra-ldflags="-L$mingw/lib"
make
cd ..
zip -r qemu-sideswipe-w64.zip build-msys

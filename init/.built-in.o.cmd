cmd_init/built-in.o :=  /opt/toolchains/arm-2009q3/bin/arm-none-linux-gnueabi-ld -EL    -r -o init/built-in.o init/main.o init/version.o init/mounts.o init/initramfs.o init/calibrate.o 

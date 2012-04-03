#!/bin/bash
#Thanks to Hanny Liu for the build script and helping me set up kernel!

# Set Default Path
TOP_DIR=$PWD
KERNEL_PATH="/home/hanny/kernel/Lucifr_old"

# Set toolchain and root filesystem path
TOOLCHAIN=""
ROOTFS_PATH="/home/hanny/kernel/initramfs"
FILENAME="Lucifr_1.5uv"
export KERNELDIR=$KERNEL_PATH

echo "Compiling Kernel"
make -j2 CONFIG_INITRAMFS_SOURCE="$ROOTFS_PATH" || exit -1

# Copy Kernel Image
rm -f $KERNEL_PATH/release/zip/*.zip
rm -f $KERNEL_PATH/release/tar/*.tar
cp -f $KERNEL_PATH/arch/arm/boot/zImage .
cp -f $KERNEL_PATH/arch/arm/boot/zImage $KERNEL_PATH/release/zip

cd arch/arm/boot
tar cf $KERNEL_PATH/arch/arm/boot/$FILENAME.tar ../../../zImage && ls -lh $FILENAME.tar

cd ../../..
cd release/zip
zip -r $FILENAME.zip *

cp $KERNEL_PATH/arch/arm/boot/$FILENAME.tar $KERNEL_PATH/release/tar/$FILENAME.tar
rm $KERNEL_PATH/arch/arm/boot/$FILENAME.tar
rm $KERNEL_PATH/release/zip/zImage


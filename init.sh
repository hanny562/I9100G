#This Init.sh is took from cranium kernel .Thanks to Sarthak Acharya

# Set Default Path
TOP_DIR=$PWD
KERNEL_PATH="/home/hanny/kernel/Lucifr"

# Set toolchain and root filesystem path
TOOLCHAIN=""
ROOTFS_PATH="/home/hanny/kernel/initramfs"

if [ -e zImage ]; then
	rm zImage
fi

echo "Cleaning latest build"
make clean mrproper
rm compile.log

make lucifr_defconfig

echo "----------------Lucifr Kernel Compilation----------------"

echo "Making Modules"
make modules

echo "Collecing Compiled Modules"
# Copying kernel modules
cd $KERNEL_PATH
find -name '*.ko' -exec cp -av {} $KERNEL_PATH/modules/ \;

echo "Stripping Modules"
cd  modules
./strip.sh

# Copying kernel modules
echo "Copying Modules to Initramfs Directory"
find -name '*.ko' -exec cp -av {} $ROOTFS_PATH/lib/modules/ \;

echo "Initiating Kernel Compilation"
cd ../
./build.sh



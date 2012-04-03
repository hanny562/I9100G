for i in $(find . | grep .ko | grep './')
do
        echo $i
/opt/toolchains/4.4.3/bin/arm-none-linux-gnueabi-strip --strip-unneeded $i
done

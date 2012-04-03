for i in $(find . | grep .ko | grep './')
do
        echo $i
/opt/toolchains/arm-2010q1/bin/arm-none-linux-gnueabi-strip --strip-unneeded $i
done

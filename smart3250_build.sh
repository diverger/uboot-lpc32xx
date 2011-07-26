#!bin/bash

eldk_path="/home/diverger/work/projects"
uboot_path="/home/diverger/work/projects/lpc32x0/uboot/uboot-lpc32xx"

echo Change to '$eldk_path'
cd $eldk_path

echo Set enviorenment
source eldk42/eldk_init arm

echo Change to '$uboot_path'
cd $uboot_path

echo Cleanup...
make mrproper

echo Build...
make smart3250_config

make

echo Build done!

exit 0
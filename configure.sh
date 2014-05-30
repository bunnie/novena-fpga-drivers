#!/bin/sh
echo "Setting export of reset pin"
echo 135 > /sys/class/gpio/export
echo "setting reset pin to out"
echo out > /sys/class/gpio/gpio135/direction
echo "flipping reset"                       
echo 0 > /sys/class/gpio/gpio135/value
echo 1 > /sys/class/gpio/gpio135/value
                                      
echo "configuring FPGA"               
                                      
dd if=/home/bunnie/code/novena-fpga-drivers/novena_fpga.bit of=/dev/spidev2.0 bs=128
                                                             
echo "turning on clock to FPGA"                              
devmem2 0x020c8160 w 0x00000D2B

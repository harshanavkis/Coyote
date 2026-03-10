#!/usr/bin/env bash

# sudo bash sw/util/hot_reset.sh "e1:00.0"

host=`hostname`
if [[ $host == "rose" ]]; then
  BDF="c1:00.0"
else
  BDF="e1:00.0"
fi

echo 1 | sudo tee /sys/bus/pci/devices/0000:$BDF/remove
echo 1 | sudo tee /sys/bus/pci/rescan

if [[ $host == "clara" ]]; then
  echo "Installing driver for clara."
  sudo insmod driver/build/coyote_driver.ko ip_addr=0x0a000002 mac_addr=000A350E24F2
elif [[ $host == "amy" ]]; then
  echo "Installing driver for amy."
  sudo insmod driver/build/coyote_driver.ko ip_addr=0x0a000001 mac_addr=000A350E24D6
elif [[ $host == "rose" ]]; then
  echo "Installing driver for rose."
  sudo insmod driver/build/coyote_driver.ko ip_addr=0x0a000003 mac_addr=000A350E24E6
fi

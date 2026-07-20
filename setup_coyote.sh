#!/usr/bin/env bash

# sudo bash sw/util/hot_reset.sh "e1:00.0"

host=`hostname`
if [[ $host == "rose" ]]; then
  BDF="c1:00.0"
else
  BDF="e1:00.0"
fi

# Unload any already-loaded instance before touching the device. The driver's
# MODULE_DEVICE_TABLE lets udev auto-load it (without ip/mac parameters!) on
# any PCI rescan if the .ko was ever installed under /lib/modules — leaving a
# misconfigured instance that makes the insmod below fail with "File exists".
sudo rmmod coyote_driver 2>/dev/null

echo 1 | sudo tee /sys/bus/pci/devices/0000:$BDF/remove
echo 1 | sudo tee /sys/bus/pci/rescan

# The rescan above may have re-triggered the udev auto-load — unload again so
# the parameterized insmod below is the one that sticks.
sudo rmmod coyote_driver 2>/dev/null

if ! [[ $host == "clara" || $host == "amy" || $host == "rose" ]]; then
  echo "WARNING: unknown host '$host' — no driver installed."
fi

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

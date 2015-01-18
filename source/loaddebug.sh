#!/bin/sh
cd build
cd Development
echo "file attribute adjusting for  JMicronATA.kext"

sudo rm -rf /System/Library/Extensions.mkext
sudo rm -rf /System/Library/Extensions.kextcache

sudo chown -R root:wheel JMicronATA.kext
sudo find JMicronATA.kext -type d -exec chmod 0755 {} \;
sudo find JMicronATA.kext -type f -exec chmod 0644 {} \; 

sudo kextload -t JMicronATA.kext
sudo kextcache -k /System/Library/Extensions


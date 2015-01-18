#!/bin/sh
cd build
cd Development
echo "file attribute adjusting for  JMicronATA.kext"
sudo chown -R root:wheel JMicronATA.kext
sudo find JMicronATA.kext -type d -exec chmod 0755 {} \;
sudo find JMicronATA.kext -type f -exec chmod 0644 {} \; 

sudo kextunload JMicronATA.kext

echo "restore file attributes..."
sudo chown -R `whoami` JMicronATA.kext
sudo kextcache -k /System/Library/Extensions
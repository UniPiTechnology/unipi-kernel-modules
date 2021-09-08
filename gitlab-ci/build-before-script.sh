#!/bin/bash

echo "Update package definition and install additional packages based on PLATFORM"
apt update

if [ "$PLATFORM" == "axon" ]; then
    apt install -y axon-kernel-headers
elif [ "$PLATFORM" == "g1" ]; then
    apt install -y g1-kernel-headers
elif [ "$PLATFORM" == "zulu" ] || [ "$PLATFORM" == "patron" ] || [ "$PLATFORM" == "iris" ]; then
    sed 's/main g1-main/main zulu-main/g' -i /etc/apt/sources.list.d/unipi.list
    apt install -y zulu-kernel-headers
fi

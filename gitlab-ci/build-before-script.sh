#!/bin/bash

echo "Update package definition and install additional packages based on PLATFORM"

if [ "$PRODUCT" == "axon" ]; then
    apt update
    apt install -y axon-kernel-headers
elif [ "$PRODUCT" == "neuron64" ]; then
    . /ci-scripts/include.sh
    RASPBIAN_REPO="http://archive.raspberrypi.org/debian"
    curl $RASPBIAN_REPO/raspberrypi.gpg.key  | apt-key add -
    echo "deb $RASPBIAN_REPO/ ${DEBIAN_VERSION} main" > /etc/apt/sources.list.d/raspi.list
    apt-get update
    apt-get install -y raspberrypi-kernel-headers
elif [ "$PRODUCT" == "g1" ]; then
    apt update
    apt install -y g1-kernel-headers
elif [ "$PRODUCT" == "zulu" ] || [ "$PRODUCT" == "patron" ] || [ "$PRODUCT" == "iris" ]; then
    sed 's/main g1-main/main zulu-main/g' -i /etc/apt/sources.list.d/unipi.list
    apt update
    apt install -y zulu-kernel-headers
    # modify repo-patch-table
    cat >>/ci-scripts/repo_patch_table.txt <<EOF

bullseye-zulu-main    bullseye-zulu-main  bullseye-patron-main  bullseye-iris-main
bullseye-zulu-test    bullseye-zulu-test  bullseye-patron-test  bullseye-iris-test
EOF
fi

if [ -n "$PLATFORM" ]; then
    echo "Unset the VERSION_SUFFIX"
    unset VERSION_SUFFIX
fi

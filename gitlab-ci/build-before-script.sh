#!/bin/bash

echo "Update package definition and install additional packages based on PLATFORM"

/ci-scripts/fix-product-repository.sh

if [ "$PRODUCT" == "axon" ]; then
    apt update
    apt install -y axon-kernel-headers
    if [ "${DEBIAN_VERSION}" = "buster" ]; then
        cat >/ci-scripts/repo_patch_table.txt <<EOF

buster-axon-main    buster-main  bullseye-axon-main
buster-axon-test    buster-test  bullseye-axon-test
EOF
    fi

elif [ "$PRODUCT" == "neuron64" ]; then
    apt-get update
    apt-get install -y raspberrypi-kernel-headers

elif [ "$PRODUCT" == "g1" ]; then
    apt update
    apt install -y g1-kernel-headers
    if [ "${DEBIAN_VERSION}" = "buster" ]; then
        cat >/ci-scripts/repo_patch_table.txt <<EOF

buster-g1-main    buster-g1-main  bullseye-g1-main
buster-g1-test    buster-g1-test  bullseye-g1-test
EOF
    fi

elif [ "$PRODUCT" == "zulu" ] || [ "$PRODUCT" == "patron" ] || [ "$PRODUCT" == "iris" ]; then
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

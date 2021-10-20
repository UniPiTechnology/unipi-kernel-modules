#!/bin/bash

echo "Update package definition and install additional packages based on PLATFORM"

. /ci-scripts/include.sh
ARCH="$(dpkg-architecture -q DEB_BUILD_ARCH)"
/ci-scripts/fix-product-repository.sh "${DEBIAN_VERSION}" "${PRODUCT}" "${ARCH}"

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

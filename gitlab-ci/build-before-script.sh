#!/bin/bash

echo "Update package definition and install additional packages based on PLATFORM"

. /ci-scripts/include.sh
ARCH="$(dpkg-architecture -q DEB_BUILD_ARCH)"
/ci-scripts/fix-product-repository.sh "${DEBIAN_VERSION}" "${PRODUCT}" "${ARCH}"

apt update
rm -f /etc/pip.conf

#apt upgrade -y
apt-get install dh-dkms || true # only on bookworm required

if [ "$PRODUCT" == "axon" ]; then
    apt install -y axon-kernel-headers

elif [ "$PRODUCT" == "unipi1" ] || [ "$PRODUCT" == "unipi1x64" ] || [ "$PRODUCT" == "neuronu" ] || [ "$PRODUCT" == "unipi1u" ] || [ "$PRODUCT" == "unipi1x64u" ]; then
    echo "Package for product $PRODUCT cannot be built directly!" >&2
    exit 1

elif [ "$PRODUCT" == "neuron64" ]; then
    if [ "$DEBIAN_VERSION" = "bullseye" ]; then
        apt-get install -y raspberrypi-kernel-headers
    else
        apt-get install -y linux-headers-rpi-v8
    fi
    # modify repo-patch-table
    sed "/^$DEBIAN_VERSION-$PRODUCT-/d" -i /ci-scripts/repo_patch_table.txt
    cat >>/ci-scripts/repo_patch_table.txt <<EOF

$DEBIAN_VERSION-$PRODUCT-main    $DEBIAN_VERSION-neuron-main $DEBIAN_VERSION-unipi1-main
$DEBIAN_VERSION-$PRODUCT-test    $DEBIAN_VERSION-neuron-test $DEBIAN_VERSION-unipi1-test
EOF
elif [ "$PRODUCT" == "neuron" ]; then
    if [ "$DEBIAN_VERSION" = "bullseye" ]; then
        apt-get install -y raspberrypi-kernel-headers
    else
        apt-get install -y linux-headers-rpi-v7 linux-headers-rpi-v7l
    fi
    # modify repo-patch-table
    sed "/^$DEBIAN_VERSION-$PRODUCT-/d" -i /ci-scripts/repo_patch_table.txt
    cat >>/ci-scripts/repo_patch_table.txt <<EOF

$DEBIAN_VERSION-$PRODUCT-main    $DEBIAN_VERSION-neuron-main $DEBIAN_VERSION-unipi1-main
$DEBIAN_VERSION-$PRODUCT-test    $DEBIAN_VERSION-neuron-test $DEBIAN_VERSION-unipi1-test
EOF

elif [ "$PRODUCT" == "neuron64u" ]; then
    apt install -y unipi-kernel-headers
    sed "/^$DEBIAN_VERSION-$PRODUCT-/d" -i /ci-scripts/repo_patch_table.txt
    cat >>/ci-scripts/repo_patch_table.txt <<EOF

$DEBIAN_VERSION-$PRODUCT-main    $DEBIAN_VERSION-neuron-main $DEBIAN_VERSION-unipi1-main
$DEBIAN_VERSION-$PRODUCT-test    $DEBIAN_VERSION-neuron-test $DEBIAN_VERSION-unipi1-test
EOF

elif [ "$PRODUCT" == "g1" ]; then
    if [ "$DEBIAN_VERSION" = "buster" ]; then
        apt install -y g1-kernel-headers
    else
        apt install -y unipi-kernel-headers
        cat >>/ci-scripts/repo_patch_table.txt <<EOF

$DEBIAN_VERSION-$PRODUCT-main    $DEBIAN_VERSION-g1-main bookworm-g1-main
$DEBIAN_VERSION-$PRODUCT-test    $DEBIAN_VERSION-g1-test bookworm-g1-test
EOF
    fi

elif [ "$PRODUCT" == "zulu" ] || [ "$PRODUCT" == "patron" ] || [ "$PRODUCT" == "iris" ]; then
    apt install -y unipi-kernel-headers
    # modify repo-patch-table
    sed "/^$DEBIAN_VERSION-$PRODUCT-/d" -i /ci-scripts/repo_patch_table.txt
    cat >>/ci-scripts/repo_patch_table.txt <<EOF

$DEBIAN_VERSION-$PRODUCT-main    $DEBIAN_VERSION-zulu-main  $DEBIAN_VERSION-patron-main  $DEBIAN_VERSION-iris-main
$DEBIAN_VERSION-$PRODUCT-test    $DEBIAN_VERSION-zulu-test  $DEBIAN_VERSION-patron-test  $DEBIAN_VERSION-iris-test
EOF
else
    apt-get install -y dkms
fi

if [ -n "$PLATFORM" ]; then
    echo "Unset the VERSION_SUFFIX"
    unset VERSION_SUFFIX
fi

### create fake package for RPi 32bit system on 64bit kernel
if [ "$PRODUCT" = "neuron64" ] && [ "$DEBIAN_VERSION" = "bullseye" ]; then
    mv /ci-scripts/build-package.sh /ci-scripts/build-package.ish
    cat >/ci-scripts/build-package.sh <<EOF
#!/bin/bash
exec /ci-scripts/build-package.ish --build=any,all unipi-kernel-modules
EOF
    chmod +x /ci-scripts/build-package.sh
fi

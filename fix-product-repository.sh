#!/bin/bash

echo "Update apt repository based on PRODUCT"
#
# create    /etc/apt/sources.list.d/unipi-product.list
# and/or    /etc/apt/sources.list.d/raspi.list
#
# file /etc/apt/sources.list.d/unipi.list already contains
#       deb UNIPI_REPO main
#

. /ci-scripts/include.sh

RASPBIAN_REPO="http://archive.raspberrypi.org/debian"
UNIPI_REPO="https://repo.unipi.technology/debian"

## for Raspberry Pi based products add raspbian repo
if [ "$PRODUCT" == "neuron64" -o "$PRODUCT" == "neuron" ]; then
    #curl $RASPBIAN_REPO/raspberrypi.gpg.key  | apt-key add - # done in bob-the-builder
    echo "deb ${RASPBIAN_REPO}/ ${DEBIAN_VERSION} main" > /etc/apt/sources.list.d/raspi.list
    [ "${DEBIAN_VERSION}" = "buster" ] && exit
fi

# skip all for stretch
if [ "${DEBIAN_VERSION}" = "stretch" ]; then exit; fi

if [ "$PRODUCT" == "axon" -a "${DEBIAN_VERSION}" = "buster" ]; then exit; fi

if [ "${DEBIAN_VERSION}" = "buster" ]; then 
    rm /etc/apt/sources.list.d/unipi.list
fi

echo "deb ${UNIPI_REPO}/ ${DEBIAN_VERSION} $PRODUCT-main" > /etc/apt/sources.list.d/unipi-product.list

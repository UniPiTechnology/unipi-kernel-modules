#!/bin/bash
#
# call /ci-scripts/fix-product-repository.sh  PRODUCT DEBIAN_VESRION
#
# (re)creates  /etc/apt/sources.list.d/unipi.list
# and/or       /etc/apt/sources.list.d/raspi.list
#
#

product=${1}
debian_version=${2}

echo "Update apt repository based on PRODUCT=$product DEBIAN_VERSION=$debian_version"

RASPBIAN_REPO="http://archive.raspberrypi.org/debian"
UNIPI_REPO="https://repo.unipi.technology/debian"

components="main"

## for Raspberry Pi based products add raspbian repo
if [ "$product" == "neuron64" -o "$product" == "neuron" ]; then
    #curl $RASPBIAN_REPO/raspberrypi.gpg.key  | apt-key add - # done in bob-the-builder
    echo "deb ${RASPBIAN_REPO}/ ${debian_version} main" > /etc/apt/sources.list.d/raspi.list
fi

case "${product}-${debian_version}" in

    neuron-stretch | neuron-buster )
            components="main"
            ;;

    axon-stretch | axon-buster )
            components="main"
            ;;

    zulu-buster | g1-buster )
            components="${product}-main"
            ;;

    * )
            components="${product}-main main"
            ;;
esac

echo "deb ${UNIPI_REPO}/ ${debian_version} ${components}" > /etc/apt/sources.list.d/unipi.list

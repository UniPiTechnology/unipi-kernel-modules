#!/bin/bash


. /ci-scripts/include.sh

arch=`dpkg-architecture -q DEB_BUILD_ARCH`

#  ${PRODUCT}           set by gitlab-ci
#  ${DEBIAN_VERSION}    from bob-the-builder image
#  ${arch}              from building arch

PROJECT_VERSION=$(dpkg-parsechangelog -S Version)

if [ -z "${PRODUCT}" ]; then
    ################## dkms #################333
    cat >debian/rules.in <<EOF
%:
	dh \$@  --with dkms

override_dh_dkms:
	dh_dkms -V ${PROJECT_VERSION}
	make dkms DESTDIR=${PWD}/debian/unipi-kernel-modules-dkms/usr/src/unipi-${PROJECT_VERSION}

override_dh_auto_build:

override_dh_auto_install:

EOF
    exit 0
    ################## end of dkms #################333
fi

case "${PRODUCT}" in
    neuron )
        if [ "${DEBIAN_VERSION}" == "stretch" ]; then
            BINARY_PKG_NAME=neuron-kernel
        else
            BINARY_PKG_NAME=unipi-kernel-modules
        fi
        PKG_KERNEL_HEADERS=raspberrypi-kernel-headers
        PKG_KERNEL_IMAGE=raspberrypi-kernel
        ;;
    neuron64 )
        BINARY_PKG_NAME=unipi-kernel-modules
        PKG_KERNEL_IMAGE=raspberrypi-kernel
        PKG_KERNEL_HEADERS=raspberrypi-kernel-headers
        ;;
    axon )
        BINARY_PKG_NAME=unipi-kernel-modules
        PKG_KERNEL_HEADERS=axon-kernel-headers
        PKG_KERNEL_IMAGE=axon-kernel-image
        ;;
    g1 )
        if [ "${DEBIAN_VERSION}" == "buster" ]; then
            BINARY_PKG_NAME=g1-unipi-kernel-modules
        else
            BINARY_PKG_NAME=unipi-kernel-modules
        fi
        PKG_KERNEL_HEADERS=g1-kernel-headers
        PKG_KERNEL_IMAGE=g1-kernel-image
        ;;
    zulu )
        if [ "${DEBIAN_VERSION}" == "buster" ]; then
            BINARY_PKG_NAME=zulu-unipi-kernel-modules
        else
            BINARY_PKG_NAME=unipi-kernel-modules
        fi
        PKG_KERNEL_HEADERS=zulu-kernel-headers
        PKG_KERNEL_IMAGE=zulu-kernel-image
        ;;
    iris )
        BINARY_PKG_NAME=unipi-kernel-modules
        PKG_KERNEL_HEADERS=zulu-kernel-headers
        PKG_KERNEL_IMAGE=zulu-kernel-image
        ;;
    patron )
        BINARY_PKG_NAME=unipi-kernel-modules
        PKG_KERNEL_HEADERS=zulu-kernel-headers
        PKG_KERNEL_IMAGE=zulu-kernel-image
        ;;
esac

PKG_KERNEL_VER="$(dpkg-query -f='${Version}' -W ${PKG_KERNEL_HEADERS})"
echo "PKG_KERNEL_VER = ${PKG_KERNEL_VER}"
# after rpi kernel 1.20210303-1 the version is prefixed with "1:", e.g. 1:1.20210430-2
# this breaks the combined version since it is no longer a number (1.66.1:1.20210430-2)
# we will strip the first ":" part
# but in dependencies, we need to keep it !!!
PKG_KERNEL_VER_STRIPPED="$(echo ${PKG_KERNEL_VER} | cut -d":" -f 2-)"
echo "PKG_KERNEL_VER_STRIPPED = ${PKG_KERNEL_VER_STRIPPED}"

if [ "${PRODUCT}" = "neuron" ]; then
    # in raspberrypi-kernel-headers can be more than one kernels for different SoC
    LINUX_DIR_ARR=($(dpkg -L ${PKG_KERNEL_HEADERS} | sed -n '/^\/lib\/modules\/.*-v7.*\/build$/p'))
    LINUX_DIR_PATH="${LINUX_DIR_ARR[*]}"
else
    LINUX_DIR_PATH=$(dpkg -L ${PKG_KERNEL_HEADERS} | sed -n '/^\/lib\/modules\/.*\/build$/p')
fi

#####################################################################
### Create changelog for binary packages with modified version string

cat  >debian/${BINARY_PKG_NAME}.changelog <<EOF
unipi-kernel-modules (${PROJECT_VERSION}.${PKG_KERNEL_VER_STRIPPED}) unstable; urgency=medium
  * Compiled for ${PKG_KERNEL_IMAGE}
 -- auto-generator <info@unipi.technology>  $(date -R)

EOF
cat debian/changelog >>debian/${BINARY_PKG_NAME}.changelog

#####################################################################
### Append binary packages definition to control file

if [ ${BINARY_PKG_NAME} == "unipi-kernel-modules" ]; then
    breaks="neuron-kernel"
    provides=""
elif [ ${BINARY_PKG_NAME} == "neuron-kernel" ]; then
    breaks=""
    provides="unipi-kernel-modules(=${PROJECT_VERSION})"
else
    breaks="unipi-kernel-modules, neuron-kernel"
    provides="unipi-kernel-modules(=${PROJECT_VERSION})"
fi

cat >>debian/control <<EOF

Package: ${BINARY_PKG_NAME}
Architecture: ${arch}
Pre-Depends: unipi-common
Depends: ${misc:Depends}, ${PKG_KERNEL_IMAGE}(=${PKG_KERNEL_VER}), unipi-firmware (>=5.50)
Provides: ${provides}
Replaces: ${breaks}
Breaks: ${breaks}
Description: UniPi kernel modules
 Binary kernel modules for UniPi Neuron/Axon/Patron/Iris controller.
 Compiled for ${PKG_KERNEL_IMAGE} version ${PKG_KERNEL_VER}.

EOF

#####################################################################
### Create rules.in

cat  >debian/rules.in <<EOF

%:
	dh \$@

override_dh_auto_install:
	mkdir -p debian/${BINARY_PKG_NAME}
	mv debian/tempdest/* debian/${BINARY_PKG_NAME} || exit 1;

override_dh_auto_build:
		for LDP in ${LINUX_DIR_PATH}; do \
			make clean || exit 1;\
			dh_auto_build -- LINUX_DIR_PATH=\$\${LDP} || exit 1;\
			dh_auto_install --destdir=debian/tempdest -- LINUX_DIR_PATH=\$\${LDP} || exit 1;\
		done
EOF

#cat debian/rules.in
#cat debian/unipi-kernel-modules.changelog
#cat debian/control

#!/bin/bash


. /ci-scripts/include.sh

arch=`dpkg-architecture -q DEB_BUILD_ARCH`

#  ${PRODUCT}           set by gitlab-ci
#  ${DEBIAN_VERSION}    from bob-the-builder image
#  ${arch}              from building arch

PROJECT_VERSION=$(dpkg-parsechangelog -S Version)

if [ -z "${PRODUCT}" ]; then
    ################## dkms #################333
    BINARY_PKG_NAME=unipi-kernel-modules-dkms
    if [ "${DEBIAN_VERSION}" == "stretch" -o "${DEBIAN_VERSION}" == "buster" ]; then
        pre_depends="raspberrypi-kernel-headers | axon-kernel-headers | g1-kernel-headers | zulu-kernel-headers, unipi-common"
        depends="unipi-firmware (>=5.50)"
    else
        unset pre_depends
        depends="raspberrypi-kernel-headers | unipi-kernel-headers, unipi-os-configurator-data"
        #suggests="unipi-firmware"
        unset suggests
    fi
    cat >debian/rules.in <<EOF
%:
	dh \$@  --with dkms

override_dh_prep:
	@dh_prep --exclude=${BINARY_PKG_NAME}.substvars
	@echo unipi:Pre-Depends="${pre_depends}" >> debian/${BINARY_PKG_NAME}.substvars
	@echo unipi:Depends="${depends}" >> debian/${BINARY_PKG_NAME}.substvars
	@echo unipi:Suggests="${suggests}" >> debian/${BINARY_PKG_NAME}.substvars

override_dh_dkms:
	dh_dkms -V ${PROJECT_VERSION}
	make dkms DESTDIR=${PWD}/debian/unipi-kernel-modules-dkms/usr/src/unipi-${PROJECT_VERSION}
	sed '/# insert modules here #/r dkms.conf' \
	    -i ${PWD}/debian/unipi-kernel-modules-dkms/usr/src/unipi-${PROJECT_VERSION}/dkms.conf

override_dh_auto_build:

override_dh_auto_install:

EOF
    exit 0
    ################## end of dkms #################333
fi

case "${PRODUCT}" in
    unipi1 )
        BINARY_PKG_NAME=unipi-kernel-modules
        PKG_KERNEL_HEADERS=raspberrypi-kernel-headers
        PKG_KERNEL_IMAGE=raspberrypi-kernel
        ;;
    neuron )
        if [ "${DEBIAN_VERSION}" == "stretch" ]; then
            BINARY_PKG_NAME=neuron-kernel
        else
            BINARY_PKG_NAME=unipi-kernel-modules
        fi
        PKG_KERNEL_HEADERS=raspberrypi-kernel-headers
        PKG_KERNEL_IMAGE=raspberrypi-kernel
        ;;
    neuron64 | unipi1x64)
        BINARY_PKG_NAME=unipi-kernel-modules
        PKG_KERNEL_IMAGE=raspberrypi-kernel
        PKG_KERNEL_HEADERS=raspberrypi-kernel-headers
        ;;
    neuron64u | neuronu | unipi1u | unipi1x64u)
        BINARY_PKG_NAME=unipi-kernel-modules
        PKG_KERNEL_HEADERS=unipi-kernel-headers
        PKG_KERNEL_IMAGE=unipi-kernel
        ;;
    axon )
        BINARY_PKG_NAME=unipi-kernel-modules
        PKG_KERNEL_HEADERS=axon-kernel-headers
        PKG_KERNEL_IMAGE=axon-kernel-image
        ;;
    g1 )
        if [ "${DEBIAN_VERSION}" == "buster" ]; then
            BINARY_PKG_NAME=g1-unipi-kernel-modules
            PKG_KERNEL_HEADERS=g1-kernel-headers
            PKG_KERNEL_IMAGE=g1-kernel-image
        else
            BINARY_PKG_NAME=unipi-kernel-modules
            PKG_KERNEL_HEADERS=unipi-kernel-headers
            PKG_KERNEL_IMAGE=unipi-kernel
        fi
        ;;
    zulu )
        if [ "${DEBIAN_VERSION}" == "buster" ]; then
            BINARY_PKG_NAME=zulu-unipi-kernel-modules
            PKG_KERNEL_HEADERS=zulu-kernel-headers
            PKG_KERNEL_IMAGE=zulu-kernel-image
        else
            BINARY_PKG_NAME=unipi-kernel-modules
            PKG_KERNEL_HEADERS=unipi-kernel-headers
            PKG_KERNEL_IMAGE=unipi-kernel
        fi
        ;;
    iris )
        # build for >=bullseye
        BINARY_PKG_NAME=unipi-kernel-modules
        PKG_KERNEL_HEADERS=unipi-kernel-headers
        PKG_KERNEL_IMAGE=unipi-kernel
        ;;
    patron )
        # build for >=bullseye
        BINARY_PKG_NAME=unipi-kernel-modules
        PKG_KERNEL_HEADERS=unipi-kernel-headers
        PKG_KERNEL_IMAGE=unipi-kernel
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

if [ "${PRODUCT}" = "neuron" ] || [ "${PRODUCT}" = "unipi1" ] ; then
    # in raspberrypi-kernel-headers can be more than one kernels for different SoC
    LINUX_DIR_ARR=($(dpkg -L ${PKG_KERNEL_HEADERS} | sed -n '/^\/lib\/modules\/.*-v7.*\/build$/p'))
    LINUX_DIR_PATH="${LINUX_DIR_ARR[*]}"
else
    LINUX_DIR_PATH=$(dpkg -L ${PKG_KERNEL_HEADERS} | sed -n '/^\/lib\/modules\/.*\/build$/p')
fi

#####################################################################
### Create changelog for binary packages with modified version string

if [ "$PRODUCT" == "neuron64u" ] || [ "$PRODUCT" == "neuronu" ] || [ "$PRODUCT" == "unipi1u" ] || [ "$PRODUCT" == "unipi1x64u" ]; then
    MODULES_VERSION=${PROJECT_VERSION}~${PKG_KERNEL_VER_STRIPPED}
else
    MODULES_VERSION=${PROJECT_VERSION}.${PKG_KERNEL_VER_STRIPPED}
fi
cat  >debian/${BINARY_PKG_NAME}.changelog <<EOF
unipi-kernel-modules (${MODULES_VERSION}) unstable; urgency=medium
  * Compiled for ${PKG_KERNEL_IMAGE}
 -- auto-generator <info@unipi.technology>  $(date -R)

EOF
cat debian/changelog >>debian/${BINARY_PKG_NAME}.changelog

#####################################################################
### Append binary packages definition to control file

if [ "${DEBIAN_VERSION}" == "stretch" -o "${DEBIAN_VERSION}" == "buster" ]; then
    pre_depends="unipi-common"
    depends="unipi-firmware (>=5.50)"
else
    unset pre_depends
    depends="unipi-os-configurator-data(>=0.7.test.20220815)"
    #suggests="unipi-firmware"
    unset suggests
fi

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

replaces=${breaks}
breaks="unipi-os-configurator (<= 0.36)"
[ -n "${replaces}" ] && breaks="${breaks}, ${replaces}"

cat >>debian/control <<EOF

Package: ${BINARY_PKG_NAME}
Architecture: ${arch}
Pre-Depends: ${pre_depends}
Depends: ${misc:Depends}, ${PKG_KERNEL_IMAGE}(=${PKG_KERNEL_VER}), ${depends}
Suggests: ${suggests}
Provides: ${provides}
Replaces: ${replaces}
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

echo "==============================================================================================================="
echo "debian/rules"
echo "==============================================================================================================="
cat debian/rules.in

echo "==============================================================================================================="
echo "debian/control"
echo "==============================================================================================================="
cat debian/control

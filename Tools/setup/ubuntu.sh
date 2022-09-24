#! /usr/bin/env bash

set -e

## Bash script to setup PX4 development environment on Ubuntu LTS (20.04, 18.04).
## Can also be used in docker.
##
## Installs:
## - Common dependencies and tools for NuttX, and Gazebo
## - NuttX toolchain (omit with arg: --no-nuttx)
## - Gazebo simulator (omit with arg: --no-sim-tools)

INSTALL_NUTTX="true"
INSTALL_SIM="true"
INSTALL_ARCH=`uname -m`
INSIDE_DOCKER="false"

# Parse arguments
for arg in "$@"
do
	if [[ $arg == "--no-nuttx" ]]; then
		INSTALL_NUTTX="false"
	fi

	if [[ $arg == "--no-sim-tools" ]]; then
		INSTALL_SIM="false"
	fi


	if [[ $arg == "--from-docker" ]]; then
		INSIDE_DOCKER="true"
	fi

  if [[ $arg == "--help" ]]; then
    echo "#⚡️ PX4 Dependency Installer for Ubuntu"
    echo "# Options:
#
#  --no-nuttx       boolean
#  --no-sim-tools   boolean"
    echo "#"
    exit
  fi

done

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# check requirements.txt exists (script not run in source tree)
REQUIREMENTS_FILE="requirements.txt"
if [[ ! -f "${DIR}/${REQUIREMENTS_FILE}" ]]; then
	echo "FAILED: ${REQUIREMENTS_FILE} needed in same directory as ubuntu.sh (${DIR})."
	return 1
fi


# check ubuntu version
# otherwise warn and point to docker?
UBUNTU_RELEASE="`lsb_release -rs`"

if [[ "${UBUNTU_RELEASE}" == "14.04" ]]; then
	echo "Ubuntu 14.04 is no longer supported"
	exit 1
elif [[ "${UBUNTU_RELEASE}" == "16.04" ]]; then
	echo "Ubuntu 16.04 is no longer supported"
	exit 1
elif [[ "${UBUNTU_RELEASE}" == "18.04" ]]; then
	echo "Ubuntu 18.04"
elif [[ "${UBUNTU_RELEASE}" == "20.04" ]]; then
	echo "Ubuntu 20.04"
fi

VERBOSE_BAR="####################"
echo
echo $VERBOSE_BAR
echo "#⚡️ Starting PX4 Dependency Installer for Ubuntu ${UBUNTU_RELEASE} (${INSTALL_ARCH})"
echo "# Options:
#
#  - Install NuttX = ${INSTALL_NUTTX}
#  - Install Simulation = ${INSTALL_SIM}"
echo $VERBOSE_BAR
echo

echo
echo $VERBOSE_BAR
echo "🍻 Installing System Dependencies"
echo $VERBOSE_BAR
echo

sudo apt-get update -y --quiet
sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
	astyle \
	build-essential \
	cmake \
	cppcheck \
	file \
	g++ \
	gcc \
	gdb \
	git \
	lcov \
	libssl-dev \
	libxml2-dev \
	libxml2-utils \
	make \
	ninja-build \
	python3 \
	python3-dev \
	python3-pip \
	python3-setuptools \
	python3-wheel \
	rsync \
	shellcheck \
	unzip \
	zip \
	;

# Python 3 dependencies
echo
echo $VERBOSE_BAR
echo "🍻 Installing Python dependencies"
echo $VERBOSE_BAR
echo

if [ -n "$VIRTUAL_ENV" ]; then
	# virtual environments don't allow --user option
	python -m pip install -r ${DIR}/requirements.txt
else
	# older versions of Ubuntu require --user option
	if [[ $INSIDE_DOCKER == "true" ]]; then
		# when running inside a docker container we don't need to install
		# under --user since the installer user is root
		# its best to install packages globaly for any user to find
		python3 -m pip install -r /tmp/requirements.txt
	else
		python3 -m pip install --user -r ${DIR}/requirements.txt
	fi
fi

# NuttX toolchain (arm-none-eabi-gcc)
if [[ $INSTALL_NUTTX == "true" ]]; then

	echo
	echo $VERBOSE_BAR
	echo "🍻 Installing NuttX dependencies"
	echo $VERBOSE_BAR
	echo

	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		automake \
		binutils-dev \
		bison \
		flex \
		genromfs \
		gettext \
		gperf \
		libelf-dev \
		libexpat-dev \
		libgmp-dev \
		libisl-dev \
		libmpc-dev \
		libmpfr-dev \
		libncurses5 \
		libncurses5-dev \
		libncursesw5-dev \
		libtool \
		pkg-config \
		screen \
		texinfo \
		u-boot-tools \
		util-linux \
		vim-common \
		g++-arm-linux-gnueabihf \
		gcc-arm-linux-gnueabihf \
		g++-aarch64-linux-gnu \
		gcc-aarch64-linux-gnu \
		;

	if [[ "${UBUNTU_RELEASE}" == "20.04" ]]; then
		sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		kconfig-frontends \
		;
	fi


	if [ -n "$USER" ]; then
		# add user to dialout group (serial port access)
		sudo usermod -a -G dialout $USER
	fi

	NUTTX_GCC_VERSION="9-2020-q2-update"
	NUTTX_GCC_VERSION_SHORT="9-2020q2"
	echo
	echo $VERBOSE_BAR
	echo "🍻 Verifying proper gcc version (${NUTTX_GCC_VERSION}), and installing if not found"
	echo

	source $HOME/.profile # load changed path for the case the script is reran before relogin
	if [ $(which arm-none-eabi-gcc) ]; then
		GCC_VER_STR=$(arm-none-eabi-gcc --version)
		GCC_VER_FOUND=$(echo $GCC_VER_STR | grep -c "${NUTTX_GCC_VERSION}")
	fi

	if [[ $(echo $GCC_VER_STR | grep -c "${NUTTX_GCC_VERSION}") == "1" ]]; then
		echo "📌 Skipping installation, the arm cross compiler was found"
		echo $VERBOSE_BAR
		echo

	else
		echo "📌 The arm cross compiler was not found";
		echo " * Installing arm-none-eabi-gcc-${NUTTX_GCC_VERSION}";
		COMPILER_NAME="gcc-arm-none-eabi-${NUTTX_GCC_VERSION}"
		COMPILER_PATH="/tmp/$COMPILER_NAME-linux.tar.bz2"
    wget -O $COMPILER_PATH https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/${NUTTX_GCC_VERSION_SHORT}/${COMPILER_NAME}-${INSTALL_ARCH}-linux.tar.bz2
		sudo tar -jxf $COMPILER_PATH -C /opt/;

		# add arm-none-eabi-gcc to user's PATH
		exportline="export PATH=\"/opt/${COMPILER_NAME}/bin:\$PATH\""
		if [[ $INSIDE_DOCKER == "true" ]]; then
			# when running on a docker container its best to set the environment globally
			# since we don't know which user is going to be running commands on the container
			touch /etc/profile.d/px4env.sh
			echo $exportline >> /etc/profile.d/px4env.sh
		elif grep -Fxq "$exportline" $HOME/.profile; then
			echo "${NUTTX_GCC_VERSION} path already set.";
		else
			echo $exportline >> $HOME/.profile;
		fi
		echo " * arm-none-eabi-gcc (${NUTTX_GCC_VERSION}) Installed Succesful to /opt/${COMPILER_NAME}/bin"
		echo $VERBOSE_BAR
		echo
	fi
fi

# Simulation tools
if [[ $INSTALL_SIM == "true" ]]; then

	echo
	echo $VERBOSE_BAR
	echo "🍻 Installing PX4 Simulation Tools"
	echo

	# default and Ubuntu 20.04
	gazebo_version=11
	gazebo_packages="gazebo$gazebo_version libgazebo$gazebo_version-dev"
	if [[ "${UBUNTU_RELEASE}" == "18.04" ]]; then
		gazebo_version=9
	elif [[ "${UBUNTU_RELEASE}" == "22.04" ]]; then
		gazebo_version=11
		gazebo_packages="gazebo libgazebo-dev"
	fi

	echo "  * Gazebo Version $gazebo_version"
	echo $VERBOSE_BAR

	# General simulation dependencies
	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		bc \
		ant \
		libvecmath-java \
		;

	# Installing Gazebo and dependencies
	# Setup OSRF Gazebo repository
	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
	# Update list, since new gazebo-stable.list has been added
	sudo apt-get update -y --quiet
	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		dmidecode \
		$gazebo_packages \
		gstreamer1.0-plugins-bad \
		gstreamer1.0-plugins-base \
		gstreamer1.0-plugins-good \
		gstreamer1.0-plugins-ugly \
		gstreamer1.0-libav \
		libeigen3-dev \
		libgstreamer-plugins-base1.0-dev \
		libimage-exiftool-perl \
		libopencv-dev \
		libxml2-utils \
		pkg-config \
		protobuf-compiler \
		;

	if sudo dmidecode -t system | grep -q "Manufacturer: VMware, Inc." ; then
		# fix VMWare 3D graphics acceleration for gazebo
		echo "export SVGA_VGPU10=0" >> ~/.profile
	fi
fi

if [[ $INSIDE_DOCKER == "true" ]]; then
	# cleanup installation
	rm -rf /tmp/
fi

if [[ $INSIDE_DOCKER == "false" ]] && [[ $INSTALL_NUTTX == "true" ]]; then
	echo
	echo $VERBOSE_BAR
	echo "💡 We recommend you relogin/reboot before attempting to build NuttX targets"
	echo $VERBOSE_BAR
	echo
fi

echo
echo
echo $VERBOSE_BAR
echo "#⚡️ PX4 Dependency Installer Ended Succesfully
#
#  For more information on PX4 Autopilot check out our docs
#  at docs.px4.io, if you find a bug please file an issue
#  on GitHub https://github.com/px4/px4-autopilot"
echo $VERBOSE_BAR
echo

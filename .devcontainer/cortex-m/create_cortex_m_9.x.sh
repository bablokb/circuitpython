#!/bin/bash
# -----------------------------------------------------------------------------
# on-create.sh: postCreateCommand-hook for devcontainer.json (Cortex-M build)
#
# Author: Bernhard Bablok
#
# -----------------------------------------------------------------------------

echo -e "[on-create.sh] starting postCreateCommand"
echo -e "[on-create.sh] PWD=$PWD"

echo -e "[on-create.sh] ------------------------------------------------"
git --no-pager tag
git describe --first-parent --dirty --tags --match "[1-9].*"
git rev-parse --short HEAD
echo -e "[on-create.sh] ------------------------------------------------"
make fetch-tags > fetch-tags.log
echo -e "[on-create.sh] ------------------------------------------------"
git --no-pager tag
git describe --first-parent --dirty --tags --match "[1-9].*"
git rev-parse --short HEAD
echo -e "[on-create.sh] ------------------------------------------------"

# --- tooling   --------------------------------------------------------------

echo -e "[on-create.sh] downloading and installing gcc-arm-non-eabi toolchain"
cd /workspaces
echo -e "[on-create.sh] PWD=$PWD"

wget -qO gcc-arm-none-eabi.tar.xz \
  https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz

tar -xJf gcc-arm-none-eabi.tar.xz
ln -s arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi gcc-arm-none-eabi
rm -f gcc-arm-none-eabi.tar.xz
export PATH=/workspaces/gcc-arm-none-eabi/bin:$PATH

# add repository and install tools
echo -e "[on-create.sh] adding pybricks/ppa"
sudo add-apt-repository -y ppa:pybricks/ppa
echo -e "[on-create.sh] installing uncrustify and mtools"
sudo apt-get -y install uncrustify mtools

# dosfstools >= 4.2 needed, standard repo only has 4.1
echo -e "[on-create.sh] downloading and installing dosfstools"
wget https://github.com/dosfstools/dosfstools/releases/download/v4.2/dosfstools-4.2.tar.gz
tar -xzf dosfstools-4.2.tar.gz
(cd dosfstools-4.2/
 ./configure
 make -j $(nproc)
 sudo make install
)
rm -fr dosfstools-4.2 dosfstools-4.2.tar.gz

# --- circuitpython setup   --------------------------------------------------

cd circuitpython
echo -e "[on-create.sh] PWD=$PWD"

# additional python requirements
echo -e "[on-create.sh] pip-installing requirements"
pip install --upgrade -r requirements-dev.txt
pip install --upgrade -r requirements-doc.txt

# add pre-commit
echo -e "[on-create.sh] installing pre-commit"
pre-commit install

echo -e "[on-create.sh] ------------------------------------------------"
git --no-pager tag
git describe --first-parent --dirty --tags --match "[1-9].*"
git rev-parse --short HEAD
echo -e "[on-create.sh] ------------------------------------------------"

# create cross-compiler
echo -e "[on-create.sh] building mpy-cross"
if ! make -j $(nproc) -C mpy-cross; then                   # time: about 36 sec
  echo -e "[on-create.sh] make mpy-cross failed"
  exit 3
fi

# --- port specific submodules   ---------------------------------------------

# prepare source-code tree
(cd ports/raspberrypi
echo -e "[on-create.sh] fetching submodules"
make fetch-port-submodules
)

# --- that's it!   ------------------------------------------------------------

echo -e "[on-create.sh] setup complete"

#commands to actually build CP:
#cd ports/raspberrypi
#time make -j $(nproc) BOARD=pimoroni_tufty2040 TRANSLATION=de_DE

#!/bin/bash


set -e

# Manage arguments
# -------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------

usage()
{
    cat <<EOF

    usage: $0 

    This script install the GOAL-Robots-competiotion env

    OPTIONS:
    -u --uninstall   uninstall
    -r --reinstall   reinstall

EOF
}

INSTALL=true
UNINSTALL=false

# getopt
GOTEMP="$(getopt -o "urh" -l "uninstall,reinstall,help"  -n '' -- "$@")"

# if [[ -z "$(echo -n $GOTEMP |sed -e"s/\-\-\(\s\+.*\|\s*\)$//")" ]]; then
#     usage; exit;
# fi

eval set -- "$GOTEMP"

while true ;
do
    case "$1" in
        -u | --uninstall)
            INSTALL=false
            UNINSTALL=true
            break ;;
        -r | --reinstall)
            INSTALL=true
            UNINSTALL=true
            shift;;
        -h | --help)
            usage; exit;
            shift;
            break;;
        --) shift ;
            break ;;
    esac
done

# -------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------

CURR_DIR=$(pwd)

SCRIPT=$(realpath -s $0)
SCRIPTPATH=$(dirname $SCRIPT)
ROBOSCHOOL_PATH=${HOME}/opt/roboschool

if [[ $UNINSTALL == true ]]; then
    pip uninstall roboschool
    [[ $ROBOSCHOOL_PATH =~ roboschool ]] && rm -fr "$ROBOSCHOOL_PATH"
fi

if [[ $INSTALL == true ]]; then


    if [[ -d "$ROBOSCHOOL_PATH" ]]; then 
        echo "Roboschool already installed"
        exit
    fi
    mkdir -p $ROBOSCHOOL_PATH

    mkdir -p ${HOME}/opt
    sudo apt install -y \
        cmake ffmpeg pkg-config qtbase5-dev \
        libqt5opengl5-dev libassimp-dev libpython3.5-dev \
        libboost-python-dev libtinyxml-dev

    cd $ROBOSCHOOL_PATH

    cp -r ${SCRIPTPATH}/../roboschool/* $ROBOSCHOOL_PATH
    git clone https://github.com/olegklimov/bullet3 -b roboschool_self_collision
    cd bullet3
    git apply ${SCRIPTPATH}/bullet.patch
    mkdir build
    cd build
    cmake \
        -DBUILD_SHARED_LIBS=ON -DUSE_DOUBLE_PRECISION=1 \
        -DCMAKE_INSTALL_PREFIX:PATH=$ROBOSCHOOL_PATH/roboschool/cpp-household/bullet_local_install \
        -DBUILD_CPU_DEMOS=OFF -DBUILD_BULLET2_DEMOS=OFF \
        -DBUILD_EXTRAS=OFF  -DBUILD_UNIT_TESTS=OFF \
        -DBUILD_CLSOCKET=OFF -DBUILD_ENET=OFF \
        -DBUILD_OPENGL3_DEMOS=OFF ..
    make -j4
    make install

    cd $ROBOSCHOOL_PATH
    pip3 install -e .

    cd bullet3
    rm -fr build_cmake || true
    ./build_cmake_pybullet_double.sh || true
    mkdir -p $ROBOSCHOOL_PATH/bin

    cp build_cmake/examples/SharedMemory/App_PhysicsServer_SharedMemory $ROBOSCHOOL_PATH/bin/physics_server

    echo "export PATH=\$PATH:$ROBOSCHOOL_PATH/bin" >> ${HOME}/.bashrc

fi

cd $CURR_DIR

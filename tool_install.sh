CMAKE_VERSION=3.22.1

cd /tmp

wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz

tar -zxvf cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz

sudo mv cmake-${CMAKE_VERSION}-linux-x86_64 /opt/cmake-${CMAKE_VERSION}

sudo ln -sf /opt/cmake-${CMAKE_VERSION}/bin/cmake /usr/local/bin/cmake
sudo ln -sf /opt/cmake-${CMAKE_VERSION}/bin/ctest /usr/local/bin/ctest
sudo ln -sf /opt/cmake-${CMAKE_VERSION}/bin/cpack /usr/local/bin/cpack
sudo ln -sf /opt/cmake-${CMAKE_VERSION}/bin/ccmake /usr/local/bin/ccmake

hash -r

cmake --version
git clone https://github.com/google/or-tools.git
cd or-tools
mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_DEPS=ON \
  -DBUILD_SHARED_LIBS=OFF \
  -DCMAKE_INSTALL_PREFIX=$HOME/local/ortools-static

make -j$(sysctl -n hw.ncpu)
make install

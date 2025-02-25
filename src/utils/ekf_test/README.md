install osqp optimazer first through following step:
git clone https://github.com/osqp/osqp.git
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
make install
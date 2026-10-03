# compile script by Dave

printf "\nCompiling the shared library for use on this architecture (use the other scrip for cross compilation), will add it to the correct file path for systemcontroller to find and use\n"
clang -Wall -Wextra -Wpedantic -Wvla -pedantic -O2 -std=c23 -lm -shared -fPIC --output ../systemController/src/systemcontroller/kinematicsExtLib.so kinematicsExtLib.c

# -fPIC generates position independant code (usually needed for e.g. a shared library)
# -shared and --output x.so required to generate a shared library

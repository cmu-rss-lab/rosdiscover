#! /bin/bash
echo "CRYPTOGRAPHY_DONT_BUILD_RUST = " $CRYPTOGRAPHY_DONT_BUILD_RUST
gcc --version
python -m pip install "$@" --no-binary cryptography

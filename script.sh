#! /bin/bash
echo "CRYPTOGRAPHY_DONT_BUILD_RUST = " $CRYPTOGRAPHY_DONT_BUILD_RUST
python -m pip install --no-binary cryptography "$@"

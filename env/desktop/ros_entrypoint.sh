#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

source /opt/underlay_ws/install/setup.bash

if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
fi

# Optimisations de performance pour les bibliothèques de calcul
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1

exec "$@"
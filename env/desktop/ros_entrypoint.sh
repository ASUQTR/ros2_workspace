#!/bin/bash
set -e

# Base ROS
source /opt/ros/humble/setup.bash

# Underlay (toujours présent)
source /opt/underlay_ws/install/setup.bash

# Overlay (volume dev)
if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
fi

# Perf fix BLAS (repris de ton Docker Jetson 👍)
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1

exec "$@"
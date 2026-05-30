#!/bin/bash
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color (resets to default)

echo "${BLUE}[1/2] Verifying dependencies...${NC}"
rosdep install -i --from-path packages --rosdistro humble -y

echo "${BLUE}[2/2] Building packages...${NC}"
colcon build --symlink-install --event-handlers console_direct+

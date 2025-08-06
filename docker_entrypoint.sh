#!/bin/bash
set -e

echo "✅ Sourcing ROS2 setup files..."
source /opt/ros/humble/setup.bash

if [ -f /app/install/setup.bash ]; then
  echo "✅ Sourcing /app/install/setup.bash"
  source /app/install/setup.bash
else
  echo "⚠️ No workspace setup.bash found. Skipping..."
fi

echo "✅ Executing command: $@"
exec "$@"

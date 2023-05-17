#!/bin/bash

# Get the absolute path of the script
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Check if the script is being run from the src directory
if [[ "$script_dir" != */src ]]; then
  echo "Error: Please run the script from the 'src' directory or its parent directory."
  exit 1
fi

# Get the parent directory of the src directory
parent_dir=$(dirname "$script_dir")

# Create build and install directories if they don't exist
if [ ! -d "$parent_dir/build" ]; then
  mkdir "$parent_dir/build"
fi

if [ ! -d "$parent_dir/install" ]; then
  mkdir "$parent_dir/install"
fi

# Navigate to the build directory
cd "$parent_dir/build"

# Run CMake
cmake ../src/

# Build the project
cmake --build .

# Install the project
cmake --install ../build --prefix ../install
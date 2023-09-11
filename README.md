## Composable Motion Specifications DSL using Popov-Vereshchagin Hybrid Dynamics solver

## Dependencies:
- rdflib
- jinja2
- kdl_parser
- orocos_kinematics_dynamics
- glfw3

## Setup

### Step 1

- clone the repository into a workspace 

    ```bash
    mkdir -p ~/workspace/src && cd ~/workspace/src

    git clone --recurse-submodules -j8 https://github.com/vamsikalagaturu/ms-vs-dsl.git .
    ```

### Step 2
    
- It is recommended to use a virtual environment or you can skip to step 3
- You can install miniconda from [here](https://docs.conda.io/en/latest/miniconda.html).
- Create a virtual environment from the `cosmos_env.yml` file.
    
    ```bash
    cd ~/workspace/src

    conda env create -f cosmos_env.yml -n <name>
    ```
- Activate the environment

    ```bash
    conda activate <name>
    ```

### Step 3

- Install dependencies for mujoco

    ```bash
    cd ~/workspace/src

    sudo apt-get install libglfw3 libglfw3-dev
    ```

- Run this command to add the mujoco path to the `~/.bashrc` file
    ```bash
    echo -e 'export LD_LIBRARY_PATH=<PATH_TO_MUJOCO>mujoco/mujoco210/bin 
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia 
    export PATH="$LD_LIBRARY_PATH:$PATH" 
    export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so' >> ~/.bashrc
    ```


## How to use

### Step 1

- Install the `modelling_tools` package
    
    ```bash
    cd ~/workspace/src/modelling_tools/

    pip3 install -e .
    ```

- Convert the DSL to C++ code

    ```bash
    cd ~/workspace/src/modelling_tools/

    python3 modelling_tools/move_arm_converter.py -d
    ```
- The converted C++ code is written to `~/workspace/src/arm_actions/src/` directory.
- Flags:
    ```bash
    -d         Print debug information
    -dg        Print rdf graph and the debug information
    -nr        Do not render the template 
    ```

### Step 2

- Make and build the packages
  
    ```bash
    cd ~/workspace/src/

    ./build.sh
    ```

- The output executables are written to `~/workspace/outputs/` directory.
 
- Run the executables:

    ```bash
    cd ~/workspace/outputs/

    ./arm_actions/<executable_name>
    ```

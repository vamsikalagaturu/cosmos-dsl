## Composable Motion Specifications DSL using Popov-Vereshchagin Hybrid Dynamics solver

## Dependencies:
- rdflib
- jinja2
- urdfdom
- kdl_parser
- orocos_kinematics_dynamics
- gnuplot
- glfw3

## Setup

### Step 1

- clone the repository into a workspace 

    ```bash
    mkdir -p ~/cosmos_dsl/src && cd ~/cosmos_dsl/src

    git clone --recurse-submodules -j8 https://github.com/vamsikalagaturu/cosmos-dsl.git .
    ```

### Step 2
    

- You can install miniconda from [here](https://docs.conda.io/en/latest/miniconda.html) to create a virtual envrironment.
- Create a virtual environment from the `cosmos_env.yml` file.
    
    ```bash
    cd ~/cosmos_dsl/src

    conda env create -f cosmos_env.yml -n <name>
    ```
- Activate the environment

    ```bash
    conda activate <name>
    ```

### Step 3

- Install the dependencies: kdl_parser, urdfdom, gnuplot, glfw3

    ```bash
    sudo apt-get install libkdl-parser-dev liburdfdom-dev libglfw3 libglfw3-dev libgnuplot-iostream-dev
    ```

- Run this command to add the mujoco path to the `~/.bashrc` file - modify the `<user-name>` accordingly.
    ```bash
    echo -e 'export LD_LIBRARY_PATH=$HOME/.mujoco/mujoco210/bin
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia 
    export PATH="$LD_LIBRARY_PATH:$PATH" 
    export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so' >> ~/.bashrc
    ```


## How to use

### Step 1

- Install the `modelling_tools` package
    
    ```bash
    cd ~/cosmos_dsl/src/modelling_tools/

    (venv_name) pip3 install -e .
    ```

### Step 2

- Convert the DSL to C++ code

    ```bash
    cd ~/cosmos_dsl/src/modelling_tools/

    (venv_name) python3 modelling_tools/move_arm_converter.py -d
    ```

- The converted C++ code is written to `~/workspace/src/arm_actions/src/` directory.
  
- Flags:
    ```bash
    -d         Print debug information
    -dg        Print rdf graph and the debug information
    -nr        Do not render the template 
    ```

### Step 3

- Make and build the packages
  
    ```bash
    cd ~/cosmos_dsl/src/

    ./build.sh
    ```

- The output executables are written to `~/cosmos_dsl/outputs/` directory.
 
- Run the executables:

    ```bash
    cd ~/cosmos_dsl/outputs/

    ./arm_actions/<executable_name>
    ```

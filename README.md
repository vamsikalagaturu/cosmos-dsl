## Repository consisiting DSL for motion specifications using Vereshchagin solver

## Dependencies:
- rdflib
- jinja2
- kdl_parser
- orocos_kinematics_dynamics

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
- Create a virtual environment from the `rnd_env.yml` file.
    
    ```bash
    cd ~/workspace/src

    conda env create -f rnd_env.yml -n rnd_env
    ```
- Activate the environment

    ```bash
    conda activate rnd_env
    ```


## How to use

### Step 1

- Run the `convert_and_build.sh` script

    ```bash
    cd ~/workspace/src/

    ./convert_and_build.sh
    ```
- The script will convert the DSL to C++ code and build the respecive packages in the workspace.
- The output executables are written to `~/workspace/outputs/` directory.

### Step 2
    
- Run the executables from the `~/workspace/outputs/` directory.

    ```bash
    cd ~/workspace/outputs/

    ./arm_actions/<executable_name>
    ```

## Running separately

### Step 1

- Convert the DSL to C++ code

    ```bash
    cd ~/workspace/src/

    python3 -m dsl/scripts/convert.py -d
    ```
- The converted C++ code is written to `~/workspace/src/arm_actions/src/` directory.
- The `-d` flag is used to print the debug information.

### Step 2

- Make and build the packages
  
    ```bash
    cd ~/workspace/src/

    ./build.sh
    ```
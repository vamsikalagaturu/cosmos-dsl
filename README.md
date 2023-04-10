### Repository consisiting DSL for motion specifications using Vereshchagin solver

### Dependencies:
- rdflib
- jinja2

### Folder structure

```bash
├── README.md
└── src
    ├── contexts
    │   ├── entities
    │   │   └── point.jsonld
    │   └── monitors
    │       └── monitor.jsonld
    ├── instances
    │   ├── monitor_instance.jsonld
    │   └── point_instance.jsonld
    ├── scripts
    │   └── convert.py
    ├── templates
    │   └── src
    │       └── distances.cpp.jinja2
    └── ws
        └── test#
```

### Folder structure details

- `src/contexts/` - contains json-ld contexts for entities and monitors
- `src/instances/` - contains json-ld instances for entities and monitors
- `src/scripts/convert.py` - script to read json-ld instances and generate c++ code based on the template
- `src/templates/src/distances.cpp.jinja2` - template code for calculating euclidean distances between 2 3D points
- `src/ws/test#` - contains the generated code files

### How to use

- Step 1

    - clone the repository into a workspace
    ```bash
    mkdir -p ~/workspace/src && cd ~/workspace/src

    git clone https://github.com/vamsikalagaturu/ms-vs-dsl.git .
    ```

- Step 2

    - generate code from instances
    ```bash
    cd ~/workspace/src/

    python3 convert.py
    ```
    - this will generate code in `src/ws/test#/` folder

- Step 3
    
    - compile the generated code
    ```bash
    cd ~/workspace
    mkdir build


    cd ~/workspace/src/ws/test_#/path/to/generated/code

    g++ -o ~/workspace/build/file_out_name file_name.cpp
    ```
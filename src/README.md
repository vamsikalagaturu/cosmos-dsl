### Repository consisiting DSL for motion specifications using Vereshchagin solver

### Dependencies:
- rdflib
- jinja2

### Folder structure

```bash
├── README.md
└── src
    ├── metamodels
    │   ├── entities
    │   │   ├── point.jsonld
    │   │   ├── frame.jsonld
    │   │   └── orientation.jsonld
    │   ├── monitors
    │   │   └── monitor.jsonld
    │   └── relations
    │       ├── coordinate.jsonld
    │       ├── distance.jsonld
    │       └── spatial_relations.jsonld
    ├── models
    │   ├── distances.jsonld
    │   ├── frames.jsonld
    │   ├── monitors.jsonld
    │   ├── points.jsonld
    │   ├── position_coord.jsonld
    │   └── position.jsonld
    ├── scripts
    │   └── convert.py
    ├── templates
    │   └── src
    │       └── distances.cpp.jinja2
    └── ws
        └── test#
```

### Folder structure details

- `src/metamodels/` - contains json-ld metamodels for entities, relations and monitors
- `src/models/` - contains json-ld models
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
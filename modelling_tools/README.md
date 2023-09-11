### This is a Python package that contains tools and templates required to auto-generate code from the DSL.

## Dependencies
- Python 3.10+
- Jinja2
- RDFLib

## Templates
This folder contains Jinja templates to generate C++ code from the DSL.

## modelling_tools
This folder contains Python scripts to retrieve data from the JSON-LD models and generate code using the templates.

> By default, the [move_arm_task](../cosmos_dsl/models/tasks/move_arm_task.jsonld) model and the [move_arm_template](templates/src/move_arm_template.cpp.jinja2) is used to generate C++ code.
## types-mujoco
This repository provides unofficial Python type stubs for parts of the [MuJoCo](https://mujoco.readthedocs.io/en/stable/APIreference/index.html) API. 

##  Warning
- The repo is WIP state. Thus, not all functions or types from MuJoCo are currently exported in these stubs.
- Most of the code was semi-automatically generated using LLMs and pattern matching and manual tweaking / fixing.
- As a result, there is a high chance some type hints might be incorrect or incomplete.  
- Always refer to the [MuJoCo official API reference](https://mujoco.readthedocs.io/en/stable/APIreference/index.html) for accurate documentation.

## Usage
```bash
pip install . -v
```
Then your IDE or LSP should be able to pick up the type hints.

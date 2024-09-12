# OpenPANDA

[![build-test](https://github.com/CMaybe/OpenPANDA/actions/workflows/build-test.yaml/badge.svg)](https://github.com/CMaybe/OpenPANDA/actions/workflows/build-test.yaml)

## Prerequisite

To achieve real-time performance for robot control, you need to install a real-time kernel on your system. The RT kernel is specifically designed to handle time-sensitive operations with minimal latency.



## Getting Started on Devcontainer
This repository is configured with a `devcontainer` for Visual Studio Code, allowing you to quickly set up and use the development environment using Docker.

### 1. Clone the Repository
```bash
git clone https://github.com/CMaybe/OpenPANDA.git
```
### 2. Open in VSCode

Open the cloned repository in Visual Studio Code. VSCode will detect the devcontainer configuration and prompt you to reopen the folder in the container.

### 3. Reopen in Container

Follow the prompt to reopen the repository in the Docker container. VSCode will build and start the container as defined in the .devcontainer directory, setting up the development environment according to the configuratio

### Option: Access the Container Directly
If you prefer to access the container directly, you can use the following command:

```bash
docker exec -it dev-panda-project /bin/bash
```


## Getting Started on Your Machine

If you prefer not to use VSCode or Docker, follow these steps to set up and run the project directly on your machine:

###  1. Install Dependencies
To ensure the proper functionality of this project, you need to install the following dependencies:

1. **OpenSai**
   - **Description**: A library for SAI2 (Visualization and Simulation).
   - **Repository**: [OpenSai GitHub Repository](https://github.com/manips-sai-org/OpenSai)

2. **libfranka**
   - **Description**: A library for controlling Franka Emika robots.
   - **Repository**: [libfranka GitHub Repository](https://github.com/CMaybe/libfranka)

Please refer to the respective repositories for installation instructions and further details.

### 2. Clone the Repository

```bash
git clone https://github.com/CMaybe/OpenPANDA.git
cd OpenPANDA
```


## Configuration Files
### c_cpp_properties.json
This configuration file is used by VS Code to set up IntelliSense and include paths for C++ development.
```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/sai2/**",
                "/usr/include/eigen3"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "gnu++17",
            "intelliSenseMode": "linux-gcc-x64",
            "configurationProvider": "ms-vscode.cmake-tools"
        }
    ],
    "version": 4
}
```

### settings.json
This file contains general VS Code settings to enhance the development experience.
```json
{
	"editor.formatOnSave": true,
	"editor.tabSize": 4, // Set tab size
	"[cpp]": {
		"editor.defaultFormatter": "xaver.clang-format"
	},
}
```

## Build & Run
### 1. Build

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

### 2. Run
```bash
cd ../bin
sudo ./PandaSim
```
## Todo

*   \[ ] Documentation
*   \[ ] Parameterization 
*   \[ ] TBA

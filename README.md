# source-volume-renderer
Volume renderer for sources from astronomical sourveys.

# Development Setup
Clone this repository recursive
  - Cuda
  - OptiX
  - vcpkg
  - Visual Studio 2022

Duplicate `CMakeUserPresets.json.example` as `CMakeUserPresets.json` and set valid values to the cacheVariables.

Open the project folder with VS 2022 or VS Code and configure cmake.

## VS Code
- Duplicate `./vscode/launch.example.json` as `./vscode/launch.json` and replace the path to the files inside the args list.
- Debug with VS Code

## Console
- Build SimpleSourceRenderer
- `./SimpleSourceRenderer -vol path/to/vol.fits -cat /path/to/cat.xml`



# Team OKC 2022 C++ Code


## Installation

1. Install the WPILib from [here](https://github.com/wpilibsuite/allwpilib/releases)
    - Currently using the 2023 Beta 3 Release
2. Install Visual Studio Desktop development with C++ as shown [here](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html)
    - Note: you only need the one checkbox that WPILib shows
    - If you don't want to install this, you can still develop by turning off desktop support. However, this disables unit tests and simulation.
3. Set up development tools:
    - clang-format VS.Code extension: [Install](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format)
4. Add the following to your settings.json:
```
"[cpp]": {
    "editor.defaultFormatter": "xaver.clang-format"
},
"C_Cpp.default.cppStandard": "c++17",
"editor.formatOnSave": true,
"clang-format.language.cpp.style": "file",
```

## Known Issues
- navX has not officially released their beta test code. For now we will use the unofficial vendor:
```
https://raw.githubusercontent.com/rzblue/navx-frc/maven/navx_frc.json
```

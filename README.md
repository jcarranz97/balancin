# Balancin

## Project Structure

```
balancin/
├── docs/                     # Documentation (markdown, images, diagrams, etc.)
│   └── assets/               # Images or other media for docs
├── firmware/                 # Folder for all firmware components
│   ├── robot/           # Each firmware module/subproject
│   ├── component2/
│   └── common/               # Shared firmware code or utilities
├── hardware/                 # KiCad or other hardware design files
│   └── pcb1/      # Specific board/project (supports expansion)
│       ├── schematics/
│       └── pcb/
├── scripts/                  # Utility scripts (build, flash, analysis, etc.)
├── LICENSE.txt
├── README.md
└── .gitignore
```

### Firmware folders

```
component1/
├── CMakeLists.txt           # CMake config for this component
├── include/                 # Public headers
│   └── component1/          # Namespaced headers
│       └── component1.h
├── src/                     # Source files
│   ├── component1.c
│   └── main.c
│   └── utils.c
├── lib/                     # Optional: Local copies of libraries or third-party code
│   └── some-lib/
├── README.md                # Description of what this firmware does
└── pico_sdk_import.cmake    # Only if building standalone; usually imported from root
```

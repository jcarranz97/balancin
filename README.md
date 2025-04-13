# Raspberry Pi Pico SDK + FreeRTOS (Project Template)

The purpose of this project is to define a base template about for new projects which use
Raspberry Pi Pico SDK and FreeRTOS.

The project structure was based on:
- [Modern C++ Project Structuring](https://gregorykelleher.com/interview_practice_questions)
- [The optimal CMake project structure](https://palikar.github.io/posts/cmake_structure/#folder-structure)

## FreeRTOS Example
The FreeRTOS configuration example was taken directly from the [pico-examples](https://github.com/raspberrypi/pico-examples?tab=readme-ov-file#freertos) repository.

## RP2350 Instructions

Everything below this section is from the stock pico-examples, so ignore URLs etc., but generally instructions are the same.

The Pico SDK default continues to be to build for RP2040 (PICO_PLATFORM=rp2040), so to build for RP2350, you need to pass
`-DPICO_PLATFORM=rp2350` to CMake (or `-DPICO_PLATFORM=rp2350-riscv` for RISC-V).

Most, but not all examples, currently work on RP2350 however you should be able to do a full build with any of the above platforms (PICO_PLATFORM=host however currently fails on some examples)

For RISC-V compilation, you should take a compiler from here: https://www.embecosm.com/resources/tool-chain-downloads/#riscv-stable

## Getting started

See [Getting Started with the Raspberry Pi Pico](https://rptl.io/pico-get-started) and the README in the [pico-sdk](https://github.com/raspberrypi/pico-sdk) for information
on getting up and running.

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

# TUBES-CAKRAI-16

## Struktur Direktori
```
- Workspace
  |- .vscode
  |- KRAI_library
  |- mbed-os
  |- TUBES-CAKRAI-16
     |- .vscode
     |- KRAI-library
     |- Projects
        |- project_1
        |- project_2 
```
Catatan: path KRAI_library yang digunakan adalah yang memiliki parent `Workspace`

## Create Project
```
mbed-tools new -c <nama_project>
```

## Setup CMakeLists.txt
### Lokasi File
```
- Workspace
  |- .vscode
  |- KRAI_library
  |- mbed-os
  |- TUBES-CAKRAI-16
     |- .vscode
     |- KRAI-library
     |- Projects
        |- project_1
           |- CMakeLists.txt <--
```

### Line 6
```
set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../mbed-os CACHE INTERNAL "")
```

### Line 14
```
add_subdirectory(${MBED_PATH} ${CMAKE_CURRENT_BINARY_DIR}/mbed-os)
```

## Setup mbed_app.json
### Lokasi file
```
- Workspace
  |- .vscode
  |- KRAI_library
  |- mbed-os
  |- TUBES-CAKRAI-16
     |- .vscode
     |- KRAI-library
     |- Projects
        |- project_1
           |- mbed_app.json <--
```

Ganti semua jadi:
```
{
    "target_overrides": {
    "*": {
        "target.printf_lib": "minimal-printf",
        "platform.minimal-printf-enable-floating-point": true,
        "platform.minimal-printf-set-floating-point-max-decimals": 3,
        "platform.minimal-printf-enable-64-bit": false,
        "target.OUTPUT_EXT":"bin"
        }
    }
    
}
```

## Cara Compile untuk Bluepill
1. Configure board
```
mbed-tools configure -m NUCLEO_F103RB -t GCC_ARM --mbed-os-path "../../../mbed-os"
```
2. Run cmake untuk set dependencies
```
cmake -S . -B cmake_build/NUCLEO_F103RB/develop/GCC_ARM -GNinja
```
3. Compile dengan ninja
```
ninja -C cmake_build/NUCLEO_F103RB/develop/GCC_ARM
```
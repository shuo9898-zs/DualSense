# Runtime and source-build boundary

The separately maintained Windows package contains `WindowsNoEditor/CarlaUE4.exe`, `CarlaUE4/`, and `Engine/`. Its inspected local footprint is approximately 16.26 GB across 845 files. The package is not included in GitHub or the anonymous code snapshot. The EXE alone is insufficient.

An Unreal Editor project with VR and custom work-zone maps is maintained on another machine. That project has not been inspected in this release preparation. Do not infer engine versions, plugins, build targets, VR launch commands, or asset redistribution rights from these Python files.

Before documenting a source build, obtain the `.uproject`, engine/CARLA versions, plugin list and versions, target platform, map list, input/OpenXR settings, cooking/packaging settings, and licenses. A packaged binary is not equivalent to releasing the project source.

Historical dependency pins, runtime distribution, full hardware smoke tests and source-build instructions remain outstanding.

---
name: dual-user-platform-setup
description: Inspect and configure the Python-side driver-worker collection platform with an existing packaged simulator; identify prerequisites for Unreal source builds.
---

# Setup workflow

1. Read ../../AGENTS.md via the repository root (the file is at `AGENTS.md`), README.md and docs/RUNTIME.md. Inventory local files before suggesting commands.
2. Ask whether the user wants to run an existing package or rebuild Unreal sources. Request OS, Python version, custom CARLA version, scenario, runtime path, SUMO installation, wheel and gaze hardware.
3. Run `python tools/check_release.py` from the repository root. Report failures without starting simulation.
4. For a packaged runtime, inspect existence of CarlaUE4.exe, CarlaUE4/ and Engine/. Do not execute untrusted binaries automatically. Obtain matching CARLA client version; do not pick the latest arbitrarily.
5. Propose a dedicated virtual environment. After approval install requirements and the matching CARLA API. Obtain vendor SDK from its distributor rather than copying an unlicensed bundle. Verify imports, SUMO executable availability, and wheel mapping.
6. Ask the user to launch the appropriate custom map/VR environment or authorize launch. Run from the selected scenario directory. Offer `--no-gaze` only when the user accepts that gaze will not be recorded.
7. With authorization, perform a brief smoke test using a nonparticipant test session. Verify timestamps and output presence. Record which modalities actually worked and stop the client cleanly.
8. For Unreal source builds, require the .uproject, engine/CARLA versions, VR plugins and packaging settings. Inspect them before producing build instructions. If the project is missing, give a missing-input checklist, not guessed build commands.
9. Finish with exact commands used, version inventory, test results and unresolved dependencies. Never describe an untested installation as working.

# DUALSense

<img width="1973" height="846" alt="DUALSense platform teaser" src="https://github.com/user-attachments/assets/df828c68-6f37-4da3-a128-5bb53ca27357" />

Python-side data collection and SUMO traffic co-simulation for a dual-user driver–worker research platform. This is a code release, not a standalone simulator distribution.

## Included and excluded

Included: six scenario implementations, steering/pedal control, driving UI, camera and vehicle logging, gaze integration, segmentation collection, weather utilities, and matching SUMO networks/routes.

Excluded: participant recordings, biosignals, demos, packaged simulator binaries, Unreal project sources/assets, and third-party eye-tracking SDKs. The custom VR-worker/work-zone environment is required separately. Stock CARLA alone does not reproduce the complete experiment.

## Structure

```text
scenarios/
  S1_Town02_Normal/
  S2_Town10_NoWarnings/
  S3_Town04_Rainy/
  S4_Town05_Curve/
  S5_Town02_2_Nighttime/
  S6_Town10HD_2_Trucks/
    main.py                     # scenario entry point
    vehicle_controller.py
    sensor_data_collection_clean.py
    standalone_eye_tracking.py
    driving_ui.py
    ui_recorder.py
    segmentation_collector.py
    config.ini
    wheel_config.ini
    SUMO/
      main_sumo_sync.py
      sumo_integration/
      sumo_files/                # matching network/routes/configuration
    weather/
    visualization/
legacy/                         # previous root snapshot, not current entry point
docs/
tools/
skills/setup/SKILL.md
AGENTS.md
requirements.txt
```

Scenario code is intentionally separate because maps, work-zone coordinates, and traffic settings differ. No experimental algorithms were consolidated during packaging.

## Running an existing packaged build

1. Obtain the custom simulator separately. Preserve its complete `WindowsNoEditor` folder, including `CarlaUE4.exe`, `CarlaUE4/`, and `Engine/`.
2. Use Windows and a Python environment compatible with that CARLA build. Exact historical version pins are not yet verified.
3. Install `requirements.txt`, the matching CARLA Python API, and SUMO. Put SUMO on `PATH`. Install the vendor `eyeware.beam_eye_tracker` SDK/application separately if collecting gaze.
4. Start the custom simulator with the intended map/VR environment. Verify the scenario-to-map pairing; do not substitute a stock map for custom work-zone assets.
5. Connect/calibrate controls and check `wheel_config.ini`. Its historical `G29 Racing Wheel` section name does not guarantee compatibility with every wheel.
6. Run from the selected scenario directory:

```powershell
cd scenarios/S1_Town02_Normal
python main.py --no-gaze
# Or, after configuring eye tracking:
python main.py
```

The client connects to `localhost:2000`. Several paths depend on the scenario working directory. Inspect each module for actual rates/settings; `config.ini` is not necessarily an exhaustive or fully wired configuration interface. Offline visualization scripts require paths to your own recordings.

## Data and reproducibility

Collection writes timestamped files under scenario-local `data_collected/` and `gaze_data/`, including camera images, vehicle/control CSVs, driving UI screenshots and segmentation. No participant data is included here.

Packaging validation is static only: Python syntax and resource references. Hardware, simulator startup, VR interaction and end-to-end collection have not been re-tested in this release preparation. Source-based rebuilding is a separate workflow: the Unreal Editor project is maintained elsewhere and is not included.

## Agent-assisted setup

Start with [AGENTS.md](AGENTS.md), then [the setup skill](skills/setup/SKILL.md). A copyable prompt is in [docs/AGENT_PROMPT.md](docs/AGENT_PROMPT.md). Run `python tools/check_release.py` for offline packaging checks.

See [runtime notes](docs/RUNTIME.md) and [third-party notes](docs/THIRD_PARTY.md). No new license grant is introduced by this update.

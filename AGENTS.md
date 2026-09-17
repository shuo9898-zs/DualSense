# Instructions for coding agents

Read README.md and docs/RUNTIME.md first. Use skills/setup/SKILL.md for installation requests.

- This repository is Python-side collection code, not the complete simulator.
- Preserve the six scenario implementations and their map/traffic pairings. Do not refactor scenario-specific geometry or modify experimental rates as part of setup.
- Never invent CARLA/Unreal/SDK versions or claim a hardware test passed without running it.
- Ask for the packaged-runtime location. Source rebuilding additionally requires the actual Unreal project; stop that branch if absent.
- Use a new virtual environment; explain dependency installation before executing it. Do not replace system Python or install global packages by default.
- Static checks may run without starting CARLA. Do not launch a simulator, collect participant data, or change device settings without user approval.
- Keep generated recordings, vendor SDKs, binaries, credentials and personal filesystem paths out of commits.
- Retain third-party notices and AI-assistance provenance. Do not create a license on behalf of the authors.
- Report separately: checked, passed, failed, and not tested. A syntax check is not an end-to-end validation.

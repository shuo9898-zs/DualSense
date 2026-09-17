# Copyable setup prompt

Please help me set up this driver–worker data collection repository. First read AGENTS.md, README.md, docs/RUNTIME.md and skills/setup/SKILL.md. Inspect the repository and run the offline release checker. Ask whether I have the packaged simulator or the Unreal source project, and ask for the exact versions and hardware needed for the selected workflow. Propose a clean virtual environment and explain installation commands before running them. Do not start simulation or record data until I approve. Do not invent missing build steps, install unverified latest CARLA versions, or change scenario geometry. End with a checklist separating verified functionality from items that remain untested.

## Terminology

A skill is a reusable, structured instruction set for an agent. A harness is the surrounding execution and validation machinery: commands, checks, logs, and success criteria. Here, tools/check_release.py is a small offline validation harness, not a complete simulator installer or hardware test suite. Agent clients vary; some discover AGENTS.md/SKILL.md automatically, while others require this prompt and explicit file references.

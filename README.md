# Hand-control
This Repository contains the firmware that will control the prosthetic hand actuation and control 

In this branch, each folder is a separate file with its own platformio.ini so users are suggest to run the folder they want to use


## Repository Layout 

Folders listed underneath (with a `platformio.ini`) is in its own PlatformIO project. Build and flash them seperately. 

| Folder | Purpose | 
| -- | -- | 
| `EMG-Control/dry-EMG-control/`  | Current EMG control |
| `EMG-Control/EMG-control-archive/` | Earlier MyoWare-based sketches, kept for referencing | 
| `Without-EMG/` | Manual control over serial, no sensor needed |


## PlatformIO setup in VS Code

This guide is for building and flashing the firmware from VS Code with PlatformIO.

Board: Arduino Nano ESP32 (PlatformIO board `arduino_nano_esp32`). Each folder containing a `platformio.ini` is its own project, e.g. `EMG-Control/Dry-EMG-Control/`. Open one project at a time.

If you use Claude Code, you can paste this to it: "Follow the PlatformIO setup in VS Code section of README.md to get this repo compiling on my machine." Otherwise do the steps yourself.

### 1. Install the tools

VS Code with one extension. Open the Extensions panel (Cmd/Ctrl+Shift+X), search for **PlatformIO IDE**, and click Install. It downloads its own toolchain and ESP32 board support on the first build, so there is no separate CLI or board core to install.

Or from a terminal (the extension ID is from memory, so if it fails, use the Extensions panel):

```
code --install-extension platformio.platformio-ide
```

If you have the Arduino extension (Arduino Community Edition or Microsoft's older "Arduino"), disable it for this workspace, because it conflicts with PlatformIO.

Wait for PlatformIO to finish initialising (the alien-head icon appears in the left bar and the status bar shows a check icon).

### 2. Open the project

`platformio.ini` is committed, and `.pio/` and `.vscode/` are gitignored because they are machine-specific. VS Code and PlatformIO generate them for you.

1. Use File > Open Folder and pick the project folder (the one containing `platformio.ini`), not the repo root.
2. If VS Code asks how to configure IntelliSense, choose "Do not configure with a compiler". PlatformIO supplies the include paths.
3. If it asks to save an untitled workspace, choose Don't Save.

To work on a different project, open its folder instead.

| Project | Folder |
|---|---|
| EMG control (current) | `EMG-Control/Dry-EMG-Control/` |
| Serial control | `Without-EMG/` | 

### 3. Build and upload

1. Click the check icon (Build) in the bottom status bar. The first build downloads the toolchain and takes a few minutes. It should end with SUCCESS.
2. Plug in the board. PlatformIO picks the port automatically, or you can set it from the plug icon in the status bar.
3. Click the arrow icon (Upload). If it fails, double-tap the reset button to enter bootloader mode and retry.
4. Click the plug icon (Serial Monitor). Baud is 115200, and the line ending must be **New Line**.

To build a different environment (for example the hardware checks), open PlatformIO in the left bar, expand Project Tasks, pick the environment, and click Build or Upload.

### Troubleshooting

- **`pio` is not recognized in PowerShell:** PlatformIO's CLI is not on your PATH. Use the terminal from the plug icon in the status bar (New Terminal), which has `pio` available, or use the sidebar buttons.
- **`cannot open source file "Arduino.h"` or other include errors in the editor:** this is an IntelliSense problem, not a build problem. Build once, then run "PlatformIO: Rebuild IntelliSense Index" and "Developer: Reload Window" from the command palette (Cmd/Ctrl+Shift+P). If it persists, delete `.vscode/c_cpp_properties.json` and repeat.
- **`multiple definition of setup` or `loop`:** every `.ino` and `.cpp` in `src/` is compiled into one firmware, so `src/` must hold exactly one sketch. Move extra sketches out of `src/`, and delete the template `main.cpp` the PlatformIO wizard creates.
- **Check the build outside the editor:** from the PlatformIO terminal, this does the same as Build and is a good way to tell a code problem from a setup problem:
  ```
  pio run -d EMG-Control/Dry-EMG-Control
  ```
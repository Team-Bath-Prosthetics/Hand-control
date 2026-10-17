# Hand-control
This Repository contains the firmware that will control the prosthetic hand actuation and control 

## Arduino setup in VS Code

The sketches also work in the Arduino IDE (open the `.ino`, choose **Arduino Nano ESP32**, press Upload). This guide is for working on them in VS Code instead.

Board: **Arduino Nano ESP32** (FQBN `arduino:esp32:nano_nora`). Sketches live in subfolders, e.g. `dry-EMG-control/dual_channel_proportional_control/`. Each sketch folder must have the same name as its `.ino` file.

If you use Claude Code, you can paste this to it: *"Follow the Arduino setup in VS Code section of README.md to get this repo compiling on my machine."* Otherwise do the steps yourself.

### 1. Install the tools
1. [VS Code](https://code.visualstudio.com/) with two extensions. Open the Extensions panel (Cmd/Ctrl+Shift+X), search for each name, and click Install:
   - **Arduino Community Edition**
   - **C/C++** (published by Microsoft)

   Or from a terminal (the Arduino extension ID is from memory, so if it fails, use the Extensions panel):
   ```
   code --install-extension vscode-arduino-community.vscode-arduino-community
   code --install-extension ms-vscode.cpptools
   ```
   If you have Microsoft's older, deprecated "Arduino" extension (`vsciot-vscode.vscode-arduino`), uninstall it first, because the two conflict.
2. Arduino CLI (pick your OS, or see the [install docs](https://arduino.github.io/arduino-cli/latest/installation/)):
   - macOS: `brew install arduino-cli`
   - Windows: `winget install ArduinoSA.CLI` (or download the zip from the install docs and add it to your PATH)
   - Linux: `curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh`
   - Check: `arduino-cli version`
3. The ESP32 board core:
   ```
   arduino-cli core update-index
   arduino-cli core install arduino:esp32
   ```
   Check: `arduino-cli core list` should show `arduino:esp32`.

If you already have the Arduino IDE installed, the core may already be there. `arduino-cli core list` will tell you.

### 2. Configure VS Code
The `.vscode/` folder is gitignored because its contents are machine-specific, so you create it yourself once.

1. Open the **Hand-control** folder (the repo root, not a subfolder) in VS Code.
2. Create `.vscode/arduino.json` with the sketch you want to work on and the board:
   ```json
   {
       "sketch": "dry-EMG-control/dual_channel_proportional_control/dual_channel_proportional_control.ino",
       "board": "arduino:esp32:nano_nora"
   }
   ```
   Or let the extension write it: open the `.ino`, then run **Arduino: Select Sketch** and **Arduino: Change Board Type** from the command palette (Cmd/Ctrl+Shift+P).
3. Create `.vscode/settings.json` so the extension uses the CLI:
   ```json
   {
       "arduino.useArduinoCli": true,
       "files.associations": { "*.ino": "cpp" }
   }
   ```
   If VS Code says it cannot find `arduino-cli`, add `"arduino.path": "<folder containing arduino-cli>"` (find it with `which arduino-cli` on macOS/Linux or `where arduino-cli` on Windows). GUI apps on macOS often don't see Homebrew's `/opt/homebrew/bin`, so this is the usual fix there.

To work on a different sketch, change the `sketch` path in `.vscode/arduino.json` (or run **Arduino: Select Sketch**).

### 3. Verify and upload
1. Open the `.ino` and run **Arduino: Verify**. It should end with a "Sketch uses ... bytes" message.
2. Plug in the board and run **Arduino: Select Serial Port**. The port looks like `COM3` on Windows, `/dev/ttyACM0` on Linux, or `/dev/cu.usbmodem...` on macOS.
3. Run **Arduino: Upload**. If it fails, double-tap the reset button to enter bootloader mode and retry.

### Troubleshooting
- **`file_opts: No such file or directory`, or "Failed to generate IntelliSense configuration":** the ESP32 core breaks when the build output path contains a space (for example a repo folder like `TBP Git`). Do not set `output` in `arduino.json`, so the extension builds in a temp folder. Cloning the repo into a path without spaces also avoids it.
- **Errors in the editor like `cannot open source file "freertos/FreeRTOS.h"`:** this is IntelliSense only and does not affect compiling. The extension drops the ESP32 core's include flags. You can ignore it.
- **Check the build outside VS Code:** from the repo root, this does the same thing as Verify and is a good way to tell a code problem from a setup problem:
  ```
  arduino-cli compile --fqbn arduino:esp32:nano_nora dry-EMG-control/dual_channel_proportional_control
  ```

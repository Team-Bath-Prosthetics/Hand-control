# Hand-control
This Repository contains the firmware that will control the prosthetic hand actuation and control 

Github test

## Arduino setup

Board: **Arduino Nano ESP32** (FQBN `arduino:esp32:nano_nora`). Sketches live in subfolders, e.g. `dry-EMG-control/dual_channel_proportional_control/`. Each sketch folder must have the same name as its `.ino` file.

If you use Claude Code, you can paste this to it: *"Follow the Arduino setup section of README.md to get this repo compiling on my machine."* Otherwise do the steps yourself.

### 1. Install the tools
- Arduino CLI (pick your OS, or see the [install docs](https://arduino.github.io/arduino-cli/latest/installation/)):
  - macOS: `brew install arduino-cli`
  - Windows: `winget install ArduinoSA.CLI` (or download the zip from the install docs and add it to your PATH)
  - Linux: `curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh`
  - Check: `arduino-cli version`
- ESP32 board core:
  ```
  arduino-cli core update-index
  arduino-cli core install arduino:esp32
  ```
- Check it worked: `arduino-cli core list` should show `arduino:esp32`.

### 2. Compile
Open a terminal in the repo root (Terminal on macOS/Linux, PowerShell on Windows). The commands below are the same on all systems:
```
arduino-cli compile --fqbn arduino:esp32:nano_nora dry-EMG-control/dual_channel_proportional_control
```
It should end with a "Sketch uses ... bytes" message. Replace the folder with whichever sketch you are working on.

### 3. Upload
1. Plug in the board and find its port with `arduino-cli board list`. It looks like `COM3` on Windows, `/dev/ttyACM0` on Linux, or `/dev/cu.usbmodem...` on macOS.
2. Upload:
   ```
   arduino-cli upload -p <PORT> --fqbn arduino:esp32:nano_nora dry-EMG-control/dual_channel_proportional_control
   ```
3. If the upload fails, double-tap the reset button to enter bootloader mode and retry.

Alternatively, install the [Arduino IDE](https://www.arduino.cc/en/software), open the `.ino` file, choose **Arduino Nano ESP32** and press Upload.

### Optional: VS Code
The extensions **Arduino Community Edition** and **C/C++** (Microsoft) let you use Verify/Upload from VS Code. This is not required, and `.vscode/` is gitignored because its settings are machine-specific. Known issues:
- Do not set the build `output` to a path containing spaces (e.g. a folder like `TBP Git`). The ESP32 core fails with `file_opts: No such file or directory`. Leaving `output` unset avoids this.
- The editor may show errors such as `cannot open source file "freertos/FreeRTOS.h"`. This is IntelliSense only and does not affect compiling, so you can ignore it.
- If VS Code cannot find `arduino-cli`, set `arduino.path` in your local settings to the folder that contains it (`which arduino-cli` on macOS/Linux, `where arduino-cli` on Windows).

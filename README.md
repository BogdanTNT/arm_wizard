# ARM Wizard

A native GTK GUI application for configuring robot arm packages in ROS2 workspaces. **Builds in seconds, runs instantly!**

## What it does

- Builds your workspace (skipping MoveIt config packages)
- Launches MoveIt Setup Assistant (auto-sources ROS 2)
- Copies MoveIt config into your play overlay and patches:
  - `joint_limits.yaml` (acceleration = velocity, floats, limits enabled)
  - controller YAML (adds `action_ns`, sets default on `arm_controller`)

## Usage

### Option 1: Run the Pre-built Executable (Easiest)

The executable is in `dist/`:
```bash
# From repo root or from file manager
./dist/arm_wizard

# Or double-click it in your file manager!
```

**Runtime Requirements:**
```bash
sudo apt install libgtk-3-0
```

### Option 2: Build from Source

**Build Requirements:**
```bash
sudo apt install rustc cargo libgtk-3-dev
```

**Build Command:**
```bash
./build.sh
```

The executable will be created at: `dist/arm_wizard`

## How It Works

1. **Select project folder**: Pick your robot project folder in the GUI.
2. **Build workspace**: Runs `colcon build` from the parent folder, skipping any MoveIt config packages.
3. **Optional compile check**: Runs `build.sh` if it exists in your project folder.
4. **Launch MoveIt Setup Assistant**: Sources `/opt/ros/<distro>/setup.bash` and your workspace `install/setup.bash` (if present).
5. **Patch overlay config**: Copies MoveIt config into `<play_pkg>/moveit_overlay/config` and applies patches.

## Project Structure

```
arm_wizard/
|-- src/main.rs           # Rust source code
|-- Cargo.toml            # Dependencies (gtk, serde_yaml, walkdir)
|-- Cargo.lock            # Locked dependency versions
|-- build.sh              # Build script
|-- dist/arm_wizard       # Compiled binary
|-- arm_wizard.sh         # Bash version (for reference)
`-- README.md             # This file
```

## Technical Details

- **Language**: Rust (ultra-fast, single binary)
- **GUI**: GTK3 (native Linux look & feel)
- **Binary Size**: ~600KB
- **Build Time**: ~6 seconds
- **Startup Time**: Instant
- **Dependencies**: Only gtk-rs, serde_yaml, walkdir

## Troubleshooting

**"Symbol lookup error" when running:**
- Ensure you're running from a native Linux filesystem (not WSL or shared folders)
- The binary in workspace root should work fine

**"Failed to load module" warnings:**
- These are just sound effect warnings and don't affect functionality

**MoveIt Setup Assistant not found:**
- Install it with: `sudo apt install ros-$ROS_DISTRO-moveit-setup-assistant`
- If the wizard offers to install it, it will prompt for your sudo password in the terminal

**Overlay path not found:**
- Ensure your play package exists (e.g., `<robot>_play/`)
- The wizard writes patches to: `<play_pkg>/moveit_overlay/config/`

## License

See the main project license.

# 🤖 ARM Wizard

A native GTK GUI application for configuring robot arm packages in ROS2 workspaces. **Builds in seconds, runs instantly!**

## What it does

- ✓ Automatically builds your description package
- ✓ Checks for MoveIt configuration (with setup guide if missing)
- ✓ Patches `joint_limits.yaml` (sets acceleration = velocity, enables limits)
- ✓ Patches `moveit_controllers.yaml` (adds action_ns, sets default controller)

## Usage

### Option 1: Run the Pre-built Executable (Easiest)

The executable is in the workspace root:
```bash
# From workspace root or from file manager
./arm_wizard

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

The executable will be created at: `dist/arm_wizard` and `../arm_wizard`

## How It Works

1. **Auto-detects workspace**: Walks up from executable location to find ROS2 workspace
2. **Finds packages**: Scans for `_description` and config packages with SRDF files
3. **One-click configuration**: Click "Run All Steps" button to perform all patches
4. **Backups**: Creates `.yaml.bak` files before modifying configs
5. **Detailed logging**: Shows real-time output in the GUI

## Project Structure

```
tools/arm_wizard/
├── src/main.rs           # Rust source code
├── Cargo.toml            # Dependencies (gtk, serde_yaml, walkdir)
├── Cargo.lock            # Locked dependency versions
├── build.sh              # Build script
├── dist/arm_wizard       # Compiled binary (~600KB)
├── arm_wizard.sh         # Bash version (for reference)
└── README.md             # This file
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

**Can't find description package:**
- Make sure your package folder ends with `_description`
- Ensure it has a `urdf/` subdirectory

## License

See the main project license.

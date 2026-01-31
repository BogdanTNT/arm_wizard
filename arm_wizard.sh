#!/usr/bin/env bash
# ============================================================================
#  ARM WIZARD - Robot Arm Configuration Helper
#  
#  Double-click this script or run: ./arm_wizard.sh
#  
#  What it does:
#    1. Builds your robot description package
#    2. Checks if MoveIt config exists
#    3. Patches joint_limits.yaml (adds acceleration, enables limits)
#    4. Patches moveit_controllers.yaml (adds action_ns, sets default)
# ============================================================================

# ─────────────────────────────────────────────────────────────────────────────
# AUTO-LAUNCH IN TERMINAL IF DOUBLE-CLICKED
# ─────────────────────────────────────────────────────────────────────────────
if [ ! -t 0 ] && [ ! -t 1 ]; then
    # Not running in a terminal - launch one
    SCRIPT_PATH="$(readlink -f "$0")"
    SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
    
    # Try different terminal emulators
    if command -v gnome-terminal >/dev/null 2>&1; then
        exec gnome-terminal --working-directory="$SCRIPT_DIR" -- bash -c "'$SCRIPT_PATH'; echo ''; echo 'Press ENTER to close...'; read"
    elif command -v xfce4-terminal >/dev/null 2>&1; then
        exec xfce4-terminal --working-directory="$SCRIPT_DIR" -e "bash -c \"'$SCRIPT_PATH'; echo ''; echo 'Press ENTER to close...'; read\""
    elif command -v konsole >/dev/null 2>&1; then
        exec konsole --workdir "$SCRIPT_DIR" -e bash -c "'$SCRIPT_PATH'; echo ''; echo 'Press ENTER to close...'; read"
    elif command -v xterm >/dev/null 2>&1; then
        exec xterm -e bash -c "cd '$SCRIPT_DIR' && '$SCRIPT_PATH'; echo ''; echo 'Press ENTER to close...'; read"
    elif command -v x-terminal-emulator >/dev/null 2>&1; then
        exec x-terminal-emulator -e bash -c "cd '$SCRIPT_DIR' && '$SCRIPT_PATH'; echo ''; echo 'Press ENTER to close...'; read"
    else
        # Last resort - show a message with zenity/kdialog
        if command -v zenity >/dev/null 2>&1; then
            zenity --error --text="Please run this script from a terminal:\n\n$SCRIPT_PATH"
        elif command -v kdialog >/dev/null 2>&1; then
            kdialog --error "Please run this script from a terminal:\n\n$SCRIPT_PATH"
        fi
        exit 1
    fi
fi

set -euo pipefail

# ─────────────────────────────────────────────────────────────────────────────
# COLORS AND HELPERS
# ─────────────────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

print_header() {
    echo ""
    echo -e "${BLUE}==============================================================${NC}"
    echo -e "${BOLD}                         ARM WIZARD                           ${NC}"
    echo -e "                 Robot Arm Configuration Helper               "
    echo -e "${BLUE}==============================================================${NC}"
    echo ""
}

print_step() {
    echo ""
    echo -e "${CYAN}--------------------------------------------------------------${NC}"
    echo -e "${BOLD}  STEP $1: $2${NC}"
    echo -e "${CYAN}--------------------------------------------------------------${NC}"
    echo ""
}

info()    { echo -e "${BLUE}INFO:${NC} $*"; }
success() { echo -e "${GREEN}OK:${NC} $*"; }
warn()    { echo -e "${YELLOW}WARN:${NC} $*"; }
error()   { echo -e "${RED}ERROR:${NC} $*"; }

wait_for_enter() {
    echo ""
    echo -e "${YELLOW}Press ENTER to continue...${NC}"
    read -r
}

# ─────────────────────────────────────────────────────────────────────────────
# FIND WORKSPACE
# ─────────────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT=""

find_workspace() {
    local dir="$SCRIPT_DIR"
    while [[ "$dir" != "/" ]]; do
        # Look for typical ROS2 workspace markers
        if [[ -d "$dir/src" ]] || [[ -d "$dir/build" ]] || [[ -d "$dir/install" ]]; then
            WORKSPACE_ROOT="$dir"
            return 0
        fi
        # Or look for package.xml files at the root level
        if ls "$dir"/*/package.xml >/dev/null 2>&1; then
            WORKSPACE_ROOT="$dir"
            return 0
        fi
        dir="$(dirname "$dir")"
    done
    
    # Fallback: assume parent of tools folder
    if [[ -d "$SCRIPT_DIR/../.." ]]; then
        WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
    fi
}

# ─────────────────────────────────────────────────────────────────────────────
# FIND PACKAGES
# ─────────────────────────────────────────────────────────────────────────────
DESCRIPTION_PKG=""
DESCRIPTION_NAME=""
MOVEIT_CONFIG_PKG=""
MOVEIT_CONFIG_NAME=""

find_packages() {
    # Find description package (ends with _description and has urdf folder)
    for pkg_dir in "$WORKSPACE_ROOT"/*/; do
        if [[ -f "$pkg_dir/package.xml" ]]; then
            local name
            name=$(basename "$pkg_dir")
            
            # Check for description package
            if [[ "$name" == *_description ]] && [[ -d "$pkg_dir/urdf" ]]; then
                DESCRIPTION_PKG="$pkg_dir"
                DESCRIPTION_NAME="$name"
            fi
            
            # Check for moveit config package (has .srdf file in config)
            if [[ -d "$pkg_dir/config" ]]; then
                if ls "$pkg_dir/config"/*.srdf >/dev/null 2>&1; then
                    MOVEIT_CONFIG_PKG="$pkg_dir"
                    MOVEIT_CONFIG_NAME="$name"
                fi
            fi
        fi
    done
}

# ─────────────────────────────────────────────────────────────────────────────
# STEP 1: BUILD DESCRIPTION PACKAGE
# ─────────────────────────────────────────────────────────────────────────────
build_description() {
    print_step "1" "Build Description Package"
    
    if [[ -z "$DESCRIPTION_PKG" ]]; then
        error "Could not find a description package!"
        info "Looking for a folder ending with '_description' that contains a 'urdf' subfolder."
        info ""
        info "Please create your robot description package first."
        return 1
    fi
    
    success "Found description package: ${BOLD}$DESCRIPTION_NAME${NC}"
    info "Location: $DESCRIPTION_PKG"
    echo ""
    info "This will build your URDF so you can visualize your robot."
    info "Building with: colcon build --packages-select $DESCRIPTION_NAME"
    echo ""
    
    wait_for_enter
    
    cd "$WORKSPACE_ROOT"
    
    info "Building..."
    echo ""
    
    if colcon build --packages-select "$DESCRIPTION_NAME"; then
        echo ""
        success "Build completed successfully."
        info ""
        info "To visualize your robot, run:"
        echo -e "    ${CYAN}source install/setup.bash${NC}"
        echo -e "    ${CYAN}ros2 launch $DESCRIPTION_NAME display.launch.py${NC}"
    else
        error "Build failed. Please check the errors above."
        return 1
    fi
}

# ─────────────────────────────────────────────────────────────────────────────
# STEP 2: CHECK MOVEIT CONFIG
# ─────────────────────────────────────────────────────────────────────────────
check_moveit_config() {
    print_step "2" "Check MoveIt Configuration"
    
    if [[ -z "$MOVEIT_CONFIG_PKG" ]]; then
        echo ""
        warn "MoveIt configuration not found."
        echo ""
        info "No worries! You just need to create it first."
        info ""
        info "Here's how to set up MoveIt for your robot:"
        echo ""
        echo -e "  ${BOLD}1.${NC} Install MoveIt Setup Assistant:"
        echo -e "     ${CYAN}sudo apt install ros-\$ROS_DISTRO-moveit-setup-assistant${NC}"
        echo ""
        echo -e "  ${BOLD}2.${NC} Source your workspace:"
        echo -e "     ${CYAN}source install/setup.bash${NC}"
        echo ""
        echo -e "  ${BOLD}3.${NC} Launch the Setup Assistant:"
        echo -e "     ${CYAN}ros2 launch moveit_setup_assistant setup_assistant.launch.py${NC}"
        echo ""
        echo -e "  ${BOLD}4.${NC} In the Setup Assistant:"
        echo -e "     - Load your URDF from: ${CYAN}$DESCRIPTION_PKG/urdf/${NC}"
        echo -e "     - Configure your planning groups, end effectors, etc."
        echo -e "     - Generate the MoveIt config package"
        echo ""
        info "Once you've created the MoveIt config, run this wizard again!"
        echo ""
        return 1
    fi
    
    success "Found MoveIt config package: ${BOLD}$MOVEIT_CONFIG_NAME${NC}"
    info "Location: $MOVEIT_CONFIG_PKG"
    
    # Show what config files exist
    echo ""
    info "Config files found:"
    for f in "$MOVEIT_CONFIG_PKG/config"/*.yaml; do
        if [[ -f "$f" ]]; then
            echo "    - $(basename "$f")"
        fi
    done
    
    success "MoveIt configuration looks good."
}

# ─────────────────────────────────────────────────────────────────────────────
# STEP 3: PATCH JOINT LIMITS
# ─────────────────────────────────────────────────────────────────────────────
patch_joint_limits() {
    print_step "3" "Patch Joint Limits"
    
    local config_dir="$MOVEIT_CONFIG_PKG/config"
    local joint_limits_file=""
    
    # Find joint_limits.yaml
    if [[ -f "$config_dir/joint_limits.yaml" ]]; then
        joint_limits_file="$config_dir/joint_limits.yaml"
    else
        # Search for any yaml with joint_limits key
        for f in "$config_dir"/*.yaml; do
            if grep -q "joint_limits:" "$f" 2>/dev/null; then
                joint_limits_file="$f"
                break
            fi
        done
    fi
    
    if [[ -z "$joint_limits_file" ]] || [[ ! -f "$joint_limits_file" ]]; then
        error "Could not find joint_limits.yaml in $config_dir"
        return 1
    fi
    
    success "Found: $(basename "$joint_limits_file")"
    info ""
    info "This step will ensure each joint has:"
    echo "    - ${BOLD}has_velocity_limits: true${NC}"
    echo "    - ${BOLD}has_acceleration_limits: true${NC}"
    echo "    - ${BOLD}max_acceleration${NC} set equal to ${BOLD}max_velocity${NC}"
    echo "    - All values as ${BOLD}floats${NC} (e.g., 1.0 not 1)"
    echo ""
    info "A backup will be created before making changes."
    echo ""
    
    wait_for_enter
    
    # Create backup
    local backup_file="${joint_limits_file}.bak.$(date +%Y%m%d_%H%M%S)"
    cp "$joint_limits_file" "$backup_file"
    success "Backup created: $(basename "$backup_file")"
    
    # Use Python to patch the file (preserves formatting better)
    python3 << PYTHON_SCRIPT
import sys
try:
    from ruamel.yaml import YAML
    yaml = YAML()
    yaml.preserve_quotes = True
except ImportError:
    import yaml as pyyaml
    class FakeYAML:
        def load(self, f):
            return pyyaml.safe_load(f)
        def dump(self, data, f):
            pyyaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)
    yaml = FakeYAML()

file_path = "$joint_limits_file"

with open(file_path, 'r') as f:
    data = yaml.load(f)

if not isinstance(data, dict) or 'joint_limits' not in data:
    print("ERROR: joint_limits key not found in file")
    sys.exit(1)

jl = data['joint_limits']
changed = False

for joint_name, values in jl.items():
    if not isinstance(values, dict):
        continue
    
    # Get velocity
    vel = values.get('max_velocity')
    if vel is None:
        continue
    
    try:
        vel_f = float(vel)
    except:
        continue
    
    # Ensure velocity is a float
    if values.get('max_velocity') != vel_f:
        values['max_velocity'] = vel_f
        changed = True
    
    # Set acceleration = velocity (as float)
    if values.get('max_acceleration') != vel_f:
        values['max_acceleration'] = vel_f
        changed = True
        print(f"  - {joint_name}: max_acceleration = {vel_f}")
    
    # Enable velocity limits
    if values.get('has_velocity_limits') is not True:
        values['has_velocity_limits'] = True
        changed = True
        print(f"  - {joint_name}: has_velocity_limits = true")
    
    # Enable acceleration limits
    if values.get('has_acceleration_limits') is not True:
        values['has_acceleration_limits'] = True
        changed = True
        print(f"  - {joint_name}: has_acceleration_limits = true")

if changed:
    with open(file_path, 'w') as f:
        yaml.dump(data, f)
    print("")
    print("SUCCESS: Joint limits updated!")
else:
    print("No changes needed - joint limits already configured correctly.")
PYTHON_SCRIPT
    
    echo ""
    success "Joint limits patching complete."
}

# ─────────────────────────────────────────────────────────────────────────────
# STEP 4: PATCH MOVEIT CONTROLLERS
# ─────────────────────────────────────────────────────────────────────────────
patch_controllers() {
    print_step "4" "Patch MoveIt Controllers"
    
    local config_dir="$MOVEIT_CONFIG_PKG/config"
    local controller_file=""
    
    # Find moveit_controllers.yaml or similar
    for name in "moveit_controllers.yaml" "controllers.yaml" "ros2_controllers.yaml"; do
        if [[ -f "$config_dir/$name" ]]; then
            # Check if it has the controller manager config
            if grep -q "moveit_simple_controller_manager\|controller_names" "$config_dir/$name" 2>/dev/null; then
                controller_file="$config_dir/$name"
                break
            fi
        fi
    done
    
    # If not found, search all yaml files
    if [[ -z "$controller_file" ]]; then
        for f in "$config_dir"/*.yaml; do
            if grep -q "moveit_simple_controller_manager\|controller_names" "$f" 2>/dev/null; then
                controller_file="$f"
                break
            fi
        done
    fi
    
    if [[ -z "$controller_file" ]] || [[ ! -f "$controller_file" ]]; then
        warn "Could not find moveit_controllers.yaml"
        info "This is optional - your robot may work without this patch."
        return 0
    fi
    
    success "Found: $(basename "$controller_file")"
    info ""
    info "This step will add to each FollowJointTrajectory controller:"
    echo "    - ${BOLD}action_ns: follow_joint_trajectory${NC}"
    echo "    - ${BOLD}default: true${NC} (on the first controller)"
    echo ""
    info "A backup will be created before making changes."
    echo ""
    
    wait_for_enter
    
    # Create backup
    local backup_file="${controller_file}.bak.$(date +%Y%m%d_%H%M%S)"
    cp "$controller_file" "$backup_file"
    success "Backup created: $(basename "$backup_file")"
    
    # Use Python to patch the file
    python3 << PYTHON_SCRIPT
import sys
try:
    from ruamel.yaml import YAML
    yaml = YAML()
    yaml.preserve_quotes = True
except ImportError:
    import yaml as pyyaml
    class FakeYAML:
        def load(self, f):
            return pyyaml.safe_load(f)
        def dump(self, data, f):
            pyyaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)
    yaml = FakeYAML()

file_path = "$controller_file"

with open(file_path, 'r') as f:
    data = yaml.load(f)

if not isinstance(data, dict):
    print("ERROR: File is not a valid YAML mapping")
    sys.exit(1)

# Find the controller manager section
mgr = None
for key, value in data.items():
    if isinstance(value, dict) and 'controller_names' in value:
        mgr = value
        break

if mgr is None:
    print("WARNING: controller_names not found in file")
    print("This file may have a different structure.")
    sys.exit(0)

controller_names = mgr.get('controller_names', [])
if not isinstance(controller_names, list):
    print("ERROR: controller_names is not a list")
    sys.exit(1)

changed = False
default_set = False

# Check if any controller already has default: true
for name in controller_names:
    ctrl = mgr.get(name, {})
    if isinstance(ctrl, dict) and ctrl.get('default') is True:
        default_set = True
        break

# Patch each controller
for i, name in enumerate(controller_names):
    ctrl = mgr.get(name, {})
    if not isinstance(ctrl, dict):
        continue
    
    # Only patch FollowJointTrajectory controllers
    if ctrl.get('type') != 'FollowJointTrajectory':
        continue
    
    # Add action_ns
    if ctrl.get('action_ns') != 'follow_joint_trajectory':
        ctrl['action_ns'] = 'follow_joint_trajectory'
        changed = True
        print(f"  - {name}: action_ns = follow_joint_trajectory")
    
    # Set first controller as default if none set
    if not default_set and i == 0:
        ctrl['default'] = True
        default_set = True
        changed = True
        print(f"  - {name}: default = true")
    
    mgr[name] = ctrl

if changed:
    with open(file_path, 'w') as f:
        yaml.dump(data, f)
    print("")
    print("SUCCESS: Controllers updated!")
else:
    print("No changes needed - controllers already configured correctly.")
PYTHON_SCRIPT
    
    echo ""
    success "Controller patching complete."
}

# ─────────────────────────────────────────────────────────────────────────────
# COMPLETION
# ─────────────────────────────────────────────────────────────────────────────
show_completion() {
    echo ""
    echo -e "${GREEN}==============================================================${NC}"
    echo -e "${BOLD}                           ALL DONE                           ${NC}"
    echo -e "${GREEN}==============================================================${NC}"
    echo ""
    info "Your robot arm is now configured."
    echo ""
    info "Next steps:"
    echo ""
    echo -e "  ${BOLD}1.${NC} Rebuild all packages:"
    echo -e "     ${CYAN}cd $WORKSPACE_ROOT${NC}"
    echo -e "     ${CYAN}colcon build${NC}"
    echo ""
    echo -e "  ${BOLD}2.${NC} Source your workspace:"
    echo -e "     ${CYAN}source install/setup.bash${NC}"
    echo ""
    echo -e "  ${BOLD}3.${NC} Launch MoveIt:"
    echo -e "     ${CYAN}ros2 launch $MOVEIT_CONFIG_NAME demo.launch.py${NC}"
    echo ""
    info "Happy robotics."
    echo ""
}

# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────
main() {
    print_header
    
    # Find workspace
    find_workspace
    if [[ -z "$WORKSPACE_ROOT" ]]; then
        error "Could not find ROS2 workspace!"
        info "Please run this script from within your workspace."
        exit 1
    fi
    
    success "Workspace found: $WORKSPACE_ROOT"
    
    # Find packages
    find_packages
    
    echo ""
    info "This wizard will guide you through configuring your robot arm."
    info "Just follow the prompts and press ENTER to proceed at each step."
    
    wait_for_enter
    
    # Step 1: Build description
    if ! build_description; then
        error "Cannot continue without description package."
        exit 1
    fi
    
    wait_for_enter
    
    # Step 2: Check MoveIt config
    if ! check_moveit_config; then
        echo ""
        info "Run this wizard again after creating your MoveIt config!"
        exit 0
    fi
    
    wait_for_enter
    
    # Step 3: Patch joint limits
    patch_joint_limits
    
    wait_for_enter
    
    # Step 4: Patch controllers
    patch_controllers
    
    # Done!
    show_completion
}

# Run main
main "$@"

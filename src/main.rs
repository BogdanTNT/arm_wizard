use gtk::prelude::*;
use gtk::{Application, ApplicationWindow, Box as GtkBox, Button, Label, ScrolledWindow, TextView, Orientation, FileChooserDialog, FileChooserAction, Entry, MessageDialog, DialogFlags, MessageType, ButtonsType, ResponseType};
use gtk::glib::{self, ControlFlow, Sender};
use std::cell::RefCell;
use std::fs;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::rc::Rc;
use std::sync::mpsc;
use std::thread;
use std::collections::HashSet;
use serde_yaml::Value;
use walkdir::WalkDir;

const APP_ID: &str = "com.armwizard.app";

enum LogEvent {
    Clear,
    Append(String),
    PromptInstall { distro: String, reply_tx: mpsc::Sender<bool> },
}

fn main() {
    let app = Application::builder().application_id(APP_ID).build();
    app.connect_activate(build_ui);
    app.run();
}

fn build_ui(app: &Application) {
    let window = ApplicationWindow::builder()
        .application(app)
        .title("ARM Wizard")
        .default_width(700)
        .default_height(600)
        .build();

    let main_box = GtkBox::new(Orientation::Vertical, 10);
    main_box.set_margin_top(15);
    main_box.set_margin_bottom(15);
    main_box.set_margin_start(15);
    main_box.set_margin_end(15);

    let title = Label::new(Some("ARM Wizard"));
    main_box.pack_start(&title, false, false, 0);

    let subtitle = Label::new(Some("Robot Arm Configuration Helper"));
    main_box.pack_start(&subtitle, false, false, 0);

    // Project folder selection section
    let ws_box = GtkBox::new(Orientation::Horizontal, 5);
    let ws_label = Label::new(Some("Project folder:"));
    ws_box.pack_start(&ws_label, false, false, 0);

    let ws_path_entry = Entry::new();
    ws_path_entry.set_editable(false);
    
    // Set default workspace path to parent of executable
    let default_ws = get_default_workspace();
    if let Some(path) = &default_ws {
        ws_path_entry.set_text(&path.display().to_string());
    }

    let workspace_state = Rc::new(RefCell::new(default_ws));
    let workspace_state_clone = workspace_state.clone();
    
    ws_path_entry.set_hexpand(true);
    ws_box.pack_start(&ws_path_entry, true, true, 0);

    let browse_btn = Button::with_label("Browse...");
    let ws_state_browse = workspace_state.clone();
    let ws_entry_clone = ws_path_entry.clone();
    let window_clone = window.clone();
    
    browse_btn.connect_clicked(move |_| {
        let dialog = FileChooserDialog::new(
            Some("Select Workspace"),
            Some(&window_clone),
            FileChooserAction::SelectFolder,
        );
        dialog.add_buttons(&[("Cancel", gtk::ResponseType::Cancel), ("Select", gtk::ResponseType::Accept)]);
        
        if dialog.run() == gtk::ResponseType::Accept {
            if let Some(path) = dialog.filename() {
                ws_entry_clone.set_text(&path.display().to_string());
                *ws_state_browse.borrow_mut() = Some(path);
            }
        }
        dialog.close();
    });
    
    ws_box.pack_start(&browse_btn, false, false, 0);
    main_box.pack_start(&ws_box, false, false, 0);

    let scroll = ScrolledWindow::new(gtk::Adjustment::NONE, gtk::Adjustment::NONE);
    scroll.set_vexpand(true);
    scroll.set_hexpand(true);
    
    let log_view = TextView::new();
    log_view.set_editable(false);
    log_view.set_wrap_mode(gtk::WrapMode::Word);
    log_view.set_monospace(true);
    scroll.add(&log_view);
    main_box.pack_start(&scroll, true, true, 0);

    let log_buffer = log_view.buffer().unwrap();
    let log = Rc::new(RefCell::new(log_buffer));

    let (log_tx, log_rx) = glib::MainContext::channel::<LogEvent>(glib::Priority::default());
    let log_for_rx = log.clone();
    let window_for_prompt = window.clone();
    log_rx.attach(None, move |event| {
        match event {
            LogEvent::Clear => {
                log_for_rx.borrow().set_text("");
            }
            LogEvent::Append(msg) => {
                append_log(&log_for_rx, &msg);
            }
            LogEvent::PromptInstall { distro, reply_tx } => {
                let dialog = MessageDialog::new(
                    Some(&window_for_prompt),
                    DialogFlags::MODAL,
                    MessageType::Question,
                    ButtonsType::None,
                    "MoveIt Setup Assistant not found",
                );
                dialog.set_secondary_text(Some(
                    &format!(
                        "Install MoveIt Setup Assistant now for ROS 2 {}?\n\nCommand:\n  sudo apt install ros-{}-moveit-setup-assistant\n\nThis will ask for your password in the terminal.",
                        distro, distro
                    ),
                ));
                dialog.add_button("Install", ResponseType::Yes);
                dialog.add_button("Cancel", ResponseType::No);
                let response = dialog.run();
                dialog.close();
                let _ = reply_tx.send(response == ResponseType::Yes);
            }
        }
        ControlFlow::Continue
    });

    let btn_box = GtkBox::new(Orientation::Horizontal, 10);
    btn_box.set_halign(gtk::Align::Center);

    let run_btn = Button::with_label("Run All Steps");
    let clear_btn = Button::with_label("Clear Log");

    btn_box.pack_start(&run_btn, false, false, 0);
    btn_box.pack_start(&clear_btn, false, false, 0);
    main_box.pack_start(&btn_box, false, false, 0);

    window.add(&main_box);

    let log_clear = log.clone();
    clear_btn.connect_clicked(move |_| {
        log_clear.borrow().set_text("");
    });

    let log_tx_run = log_tx.clone();
    let ws_run = workspace_state_clone.clone();
    run_btn.connect_clicked(move |_| {
        let workspace = ws_run.borrow().clone();
        let log_tx = log_tx_run.clone();
        thread::spawn(move || {
            run_wizard(&log_tx, workspace);
        });
    });

    append_log(&log, "Welcome to ARM Wizard!\n\n");
    append_log(&log, "This tool will:\n");
    append_log(&log, "  1. Build your project (colcon build, build.sh if present)\n");
    append_log(&log, "  2. Launch MoveIt Setup Assistant\n");
    append_log(&log, "  3. Patch MoveIt config (joint limits + controllers)\n\n");
    append_log(&log, "Click 'Run All Steps' to begin.\n");

    window.show_all();
}

fn append_log(log: &Rc<RefCell<gtk::TextBuffer>>, msg: &str) {
    let buffer = log.borrow();
    let mut end = buffer.end_iter();
    buffer.insert(&mut end, msg);
}

fn send_log(log_tx: &Sender<LogEvent>, msg: &str) {
    let _ = log_tx.send(LogEvent::Append(msg.to_string()));
}

fn clear_log(log_tx: &Sender<LogEvent>) {
    let _ = log_tx.send(LogEvent::Clear);
}

fn get_default_workspace() -> Option<PathBuf> {
    let exe = std::env::current_exe().ok()?;
    exe.parent().map(|p| p.to_path_buf())
}

fn run_wizard(log_tx: &Sender<LogEvent>, workspace: Option<PathBuf>) {
    clear_log(log_tx);
    
    send_log(log_tx, "Finding project folder...\n");
    let project_folder = match workspace {
        Some(ws) => ws,
        None => {
            send_log(log_tx, "ERROR: No project folder selected!\n");
            send_log(log_tx, "   Please select a project folder using the Browse button.\n");
            return;
        }
    };
    send_log(log_tx, &format!("OK: Project folder: {}\n", project_folder.display()));
    let build_root = project_folder
        .parent()
        .map(|p| p.to_path_buf())
        .unwrap_or_else(|| project_folder.clone());
    if build_root == project_folder {
        send_log(log_tx, "WARN: Could not determine parent folder; using project folder as build root.\n");
    }
    send_log(log_tx, &format!("OK: Build root: {}\n\n", build_root.display()));

    send_log(log_tx, "----------------------------------------\n");
    send_log(log_tx, "  STEP 1: Build Project\n");
    send_log(log_tx, "----------------------------------------\n\n");
    send_log(log_tx, "  Building...\n");

    let mut cmd = Command::new("colcon");
    cmd.arg("build").current_dir(&build_root);

    let mut selected: Vec<String> = Vec::new();
    if let Some(desc_pkg) = find_description_package(&build_root) {
        if let Some(name) = desc_pkg.file_name().and_then(|n| n.to_str()) {
            selected.push(name.to_string());
        }
    }
    if let Some(play_pkg) = find_play_package(&build_root) {
        if let Some(name) = play_pkg.file_name().and_then(|n| n.to_str()) {
            if !selected.contains(&name.to_string()) {
                selected.push(name.to_string());
            }
        }
    }

    if selected.is_empty() {
        send_log(log_tx, "WARN: No description/play packages found; building all packages.\n");
    } else {
        send_log(log_tx, "  Building packages:\n");
        for name in &selected {
            send_log(log_tx, &format!("    - {}\n", name));
        }
        cmd.arg("--packages-select");
        for name in &selected {
            cmd.arg(name);
        }
    }
    match run_command_streaming(log_tx, cmd) {
        Ok(()) => {
            send_log(log_tx, "OK: Build successful!\n\n");
        }
        Err(e) => {
            send_log(log_tx, "ERROR: Build failed:\n");
            send_log(log_tx, &format!("{}\n\n", e));
            return;
        }
    }

    let build_script = project_folder.join("build.sh");
    if build_script.is_file() {
        send_log(log_tx, "  Running build.sh (compilation check)...\n");

        let mut cmd = Command::new("bash");
        cmd.arg("build.sh").current_dir(&project_folder);
        match run_command_streaming(log_tx, cmd) {
            Ok(()) => {
                send_log(log_tx, "OK: build.sh completed successfully.\n\n");
            }
            Err(e) => {
                send_log(log_tx, "ERROR: build.sh failed:\n");
                send_log(log_tx, &format!("{}\n\n", e));
                return;
            }
        }
    } else {
        send_log(log_tx, "INFO: build.sh not found in project folder. Skipping compilation check.\n\n");
    }

    send_log(log_tx, "----------------------------------------\n");
    send_log(log_tx, "  STEP 2: MoveIt Setup Assistant\n");
    send_log(log_tx, "----------------------------------------\n\n");
    send_log(log_tx, "  Sourcing ROS environment...\n");
    let candidates = ros_env_candidates();
    if candidates.is_empty() {
        send_log(log_tx, "ERROR: No ROS 2 setup found.\n");
        send_log(log_tx, "   Please install ROS 2 Jazzy or Humble and source it:\n");
        send_log(log_tx, "     source /opt/ros/jazzy/setup.bash\n");
        send_log(log_tx, "     # or\n");
        send_log(log_tx, "     source /opt/ros/humble/setup.bash\n\n");
        return;
    }

    if let Ok(distro) = std::env::var("ROS_DISTRO") {
        if !candidates.iter().any(|c| c.distro == distro) {
            send_log(log_tx, &format!("WARN: ROS_DISTRO is set to '{}', but /opt/ros/{}/setup.bash not found.\n", distro, distro));
            send_log(log_tx, "  Falling back to available ROS 2 installs (jazzy/humble).\n");
        }
    } else {
        send_log(log_tx, "INFO: ROS_DISTRO not set; probing jazzy and humble.\n");
    }

    let mut chosen: Option<RosEnv> = None;
    let mut ros2_found_any = false;
    for env in &candidates {
        match check_moveit_available(&build_root, env) {
            Ok(check) => {
                if check.ros2_found {
                    ros2_found_any = true;
                }
                if check.moveit_found {
                    chosen = Some(env.clone());
                    break;
                }
            }
            Err(e) => {
                send_log(log_tx, &format!("WARN: Error checking MoveIt for {}: {}\n", env.distro, e));
            }
        }
    }

    if chosen.is_none() {
        if !ros2_found_any {
            send_log(log_tx, "ERROR: ros2 command not found after sourcing available ROS installs.\n");
            send_log(log_tx, "   Please install ROS 2 and source it before running this wizard.\n\n");
            return;
        }

        let install_env = candidates[0].clone();
        send_log(log_tx, &format!("WARN: MoveIt Setup Assistant not found for ROS 2 {}.\n", install_env.distro));
        send_log(log_tx, "   Install with:\n");
        send_log(log_tx, &format!("     sudo apt install ros-{}-moveit-setup-assistant\n", install_env.distro));
        send_log(log_tx, "   This will ask for your sudo password in the terminal.\n\n");

        let install_ok = request_install_moveit(log_tx, &install_env.distro);
        if !install_ok {
            send_log(log_tx, "Install canceled. Please install MoveIt and re-run.\n\n");
            return;
        }

        send_log(log_tx, "  Installing MoveIt Setup Assistant...\n");
        send_log(log_tx, "  If prompted, enter your sudo password in the terminal.\n");
        send_log(log_tx, "  If you launched the app by double-clicking, run the install command in a terminal.\n\n");

        let mut install_cmd = Command::new("bash");
        install_cmd
            .arg("-lc")
            .arg(format!(
                "sudo apt install -y ros-{}-moveit-setup-assistant",
                install_env.distro
            ))
            .current_dir(&build_root);
        if let Err(e) = run_command_streaming(log_tx, install_cmd) {
            send_log(log_tx, &format!("ERROR: Install failed: {}\n", e));
            send_log(log_tx, "   Please run the install command manually in a terminal.\n\n");
            return;
        }

        send_log(log_tx, "OK: Install completed.\n\n");
        match check_moveit_available(&build_root, &install_env) {
            Ok(check) if check.moveit_found => {
                chosen = Some(install_env);
            }
            _ => {
                send_log(log_tx, "ERROR: MoveIt Setup Assistant still not found.\n");
                send_log(log_tx, "   Please verify your ROS setup and MoveIt installation.\n\n");
                return;
            }
        }
    }

    let ros_env = chosen.unwrap();
    send_log(log_tx, &format!("OK: Using ROS 2 {}.\n\n", ros_env.distro));

    send_log(log_tx, "  Launching MoveIt Setup Assistant...\n");
    send_log(log_tx, "  (Sourcing /opt/ros and workspace install if present)\n");
    send_log(log_tx, "  Close the MoveIt window to continue.\n\n");

    let mut moveit_cmd = Command::new("bash");
    moveit_cmd
        .arg("-lc")
        .arg(format!(
            "cd '{}'; source '{}' >/dev/null 2>&1 || true; if [ -f install/setup.bash ]; then source install/setup.bash; fi; export LIBGL_ALWAYS_SOFTWARE=1; export QT_QPA_PLATFORM=xcb; ros2 run moveit_setup_assistant moveit_setup_assistant",
            build_root.display(),
            ros_env.setup_path.display()
        ))
        .current_dir(&build_root);

    match run_command_streaming(log_tx, moveit_cmd) {
        Ok(()) => {
            send_log(log_tx, "OK: MoveIt Setup Assistant closed.\n\n");
        }
        Err(e) => {
            send_log(log_tx, &format!("ERROR: MoveIt Setup Assistant failed: {}\n\n", e));
            return;
        }
    }

    send_log(log_tx, "----------------------------------------\n");
    send_log(log_tx, "  STEP 3: Patch MoveIt Config\n");
    send_log(log_tx, "----------------------------------------\n\n");

    let config_pkg = match find_config_package(&build_root) {
        Some(pkg) => {
            send_log(log_tx, &format!("OK: Found config package: {}\n", pkg.display()));
            pkg
        }
        None => {
            send_log(log_tx, "WARN: MoveIt config package not found.\n");
            send_log(log_tx, "   Make sure you generated a config package in the Setup Assistant.\n");
            send_log(log_tx, "   Then re-run this wizard.\n\n");
            return;
        }
    };

    let overlay = match ensure_overlay_config_dir(&build_root) {
        Ok(dir) => {
            send_log(log_tx, &format!("OK: Play package: {}\n", dir.play_pkg.display()));
            send_log(log_tx, &format!("OK: Overlay config: {}\n", dir.config_dir.display()));
            dir
        }
        Err(e) => {
            send_log(log_tx, &format!("ERROR: {}\n", e));
            return;
        }
    };

    send_log(log_tx, "\n  3.1 Joint Limits\n");
    if let Some(jl_file) = find_joint_limits(&config_pkg) {
        send_log(log_tx, &format!("  OK: Found: {}\n", jl_file.file_name().unwrap().to_string_lossy()));
        match copy_to_overlay(&jl_file, &overlay.config_dir) {
            Ok(dest) => {
                send_log(log_tx, &format!("  -> Copied to: {}\n", dest.display()));
                match patch_joint_limits(&dest) {
                    Ok(msg) => send_log(log_tx, &format!("  OK: {}\n", msg)),
                    Err(e) => send_log(log_tx, &format!("  ERROR: {}\n", e)),
                }
            }
            Err(e) => send_log(log_tx, &format!("  ERROR: {}\n", e)),
        }
    } else {
        send_log(log_tx, "  WARN: joint_limits.yaml not found\n");
    }

    send_log(log_tx, "\n  3.2 Controllers\n");
    if let Some(ctrl_file) = find_controller_config(&config_pkg) {
        send_log(log_tx, &format!("  OK: Found: {}\n", ctrl_file.file_name().unwrap().to_string_lossy()));
        match copy_to_overlay(&ctrl_file, &overlay.config_dir) {
            Ok(dest) => {
                send_log(log_tx, &format!("  -> Copied to: {}\n", dest.display()));
                match patch_controller_config(&dest) {
                    Ok(msg) => send_log(log_tx, &format!("  OK: {}\n", msg)),
                    Err(e) => send_log(log_tx, &format!("  ERROR: {}\n", e)),
                }
            }
            Err(e) => send_log(log_tx, &format!("  ERROR: {}\n", e)),
        }
    } else {
        send_log(log_tx, "  WARN: moveit_controllers.yaml not found\n");
    }

    send_log(log_tx, "----------------------------------------\n");
    send_log(log_tx, "  DONE\n");
    send_log(log_tx, "----------------------------------------\n\n");
    send_log(log_tx, "More steps can be added later.\n");
}

#[derive(Clone)]
struct RosEnv {
    distro: String,
    setup_path: PathBuf,
}

struct MoveItCheck {
    ros2_found: bool,
    moveit_found: bool,
}

fn ros_env_candidates() -> Vec<RosEnv> {
    let mut out: Vec<RosEnv> = Vec::new();

    if let Ok(distro) = std::env::var("ROS_DISTRO") {
        let path = PathBuf::from(format!("/opt/ros/{}/setup.bash", distro));
        if path.exists() {
            out.push(RosEnv { distro, setup_path: path });
        }
    }

    for distro in ["jazzy", "humble"] {
        if out.iter().any(|e| e.distro == distro) {
            continue;
        }
        let path = PathBuf::from(format!("/opt/ros/{}/setup.bash", distro));
        if path.exists() {
            out.push(RosEnv {
                distro: distro.to_string(),
                setup_path: path,
            });
        }
    }

    out
}

fn check_moveit_available(build_root: &Path, ros_env: &RosEnv) -> Result<MoveItCheck, String> {
    let status = Command::new("bash")
        .arg("-lc")
        .arg(format!(
            "source '{}' >/dev/null 2>&1 || true; if ! command -v ros2 >/dev/null 2>&1; then exit 10; fi; ros2 pkg prefix moveit_setup_assistant >/dev/null 2>&1",
            ros_env.setup_path.display()
        ))
        .current_dir(build_root)
        .status()
        .map_err(|e| e.to_string())?;

    if status.success() {
        Ok(MoveItCheck {
            ros2_found: true,
            moveit_found: true,
        })
    } else if status.code() == Some(10) {
        Ok(MoveItCheck {
            ros2_found: false,
            moveit_found: false,
        })
    } else {
        Ok(MoveItCheck {
            ros2_found: true,
            moveit_found: false,
        })
    }
}

fn request_install_moveit(log_tx: &Sender<LogEvent>, distro: &str) -> bool {
    let (reply_tx, reply_rx) = mpsc::channel();
    let _ = log_tx.send(LogEvent::PromptInstall {
        distro: distro.to_string(),
        reply_tx,
    });
    reply_rx.recv().unwrap_or(false)
}

fn run_command_streaming(log_tx: &Sender<LogEvent>, mut cmd: Command) -> Result<(), String> {
    let mut child = cmd
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| e.to_string())?;

    let stdout = child
        .stdout
        .take()
        .ok_or_else(|| "Failed to capture stdout".to_string())?;
    let stderr = child
        .stderr
        .take()
        .ok_or_else(|| "Failed to capture stderr".to_string())?;

    let tx_out = log_tx.clone();
    let stdout_handle = thread::spawn(move || {
        let reader = BufReader::new(stdout);
        for line in reader.lines().flatten() {
            let _ = tx_out.send(LogEvent::Append(format!("{}\n", line)));
        }
    });

    let tx_err = log_tx.clone();
    let stderr_handle = thread::spawn(move || {
        let reader = BufReader::new(stderr);
        for line in reader.lines().flatten() {
            let _ = tx_err.send(LogEvent::Append(format!("{}\n", line)));
        }
    });

    let status = child.wait().map_err(|e| e.to_string())?;
    let _ = stdout_handle.join();
    let _ = stderr_handle.join();

    if status.success() {
        Ok(())
    } else {
        Err(format!("Process exited with status: {}", status))
    }
}

struct OverlayDir {
    play_pkg: PathBuf,
    config_dir: PathBuf,
}

fn ensure_overlay_config_dir(build_root: &Path) -> Result<OverlayDir, String> {
    let play_pkg = find_play_package(build_root)
        .ok_or_else(|| "Play package not found (expected *_play or a package with moveit_overlay/)".to_string())?;
    let dir = play_pkg.join("moveit_overlay").join("config");
    fs::create_dir_all(&dir).map_err(|e| format!("Failed to create overlay config dir: {}", e))?;
    Ok(OverlayDir {
        play_pkg,
        config_dir: dir,
    })
}

fn copy_to_overlay(src: &Path, overlay_dir: &Path) -> Result<PathBuf, String> {
    let file_name = src
        .file_name()
        .ok_or_else(|| "Invalid source file name".to_string())?;
    let dest = overlay_dir.join(file_name);
    if dest.exists() {
        let backup = dest.with_extension("yaml.bak");
        let _ = fs::copy(&dest, &backup);
    }
    fs::copy(src, &dest).map_err(|e| format!("Failed to copy {}: {}", src.display(), e))?;
    Ok(dest)
}

fn find_play_package(root: &Path) -> Option<PathBuf> {
    let mut fallback: Option<PathBuf> = None;
    for entry in WalkDir::new(root).max_depth(4).into_iter().filter_map(|e| e.ok()) {
        let path = entry.path();
        if path.file_name().map(|n| n == "package.xml").unwrap_or(false) {
            let parent = path.parent()?;
            let name = parent.file_name()?.to_string_lossy();
            if parent.join("moveit_overlay").exists() {
                return Some(parent.to_path_buf());
            }
            if name.ends_with("_play") && fallback.is_none() {
                fallback = Some(parent.to_path_buf());
            }
        }
    }
    fallback
}

fn find_description_package(root: &Path) -> Option<PathBuf> {
    for entry in WalkDir::new(root).max_depth(4).into_iter().filter_map(|e| e.ok()) {
        let path = entry.path();
        if path.file_name().map(|n| n == "package.xml").unwrap_or(false) {
            let parent = path.parent()?;
            let name = parent.file_name()?.to_string_lossy();
            if name.ends_with("_description") && parent.join("urdf").exists() {
                return Some(parent.to_path_buf());
            }
        }
    }
    None
}

fn find_config_package(root: &Path) -> Option<PathBuf> {
    find_config_packages_with_srdf(root).into_iter().next()
}

fn find_config_packages_with_srdf(root: &Path) -> Vec<PathBuf> {
    let mut out: Vec<PathBuf> = Vec::new();
    let mut seen: HashSet<PathBuf> = HashSet::new();
    for entry in WalkDir::new(root).max_depth(4).into_iter().filter_map(|e| e.ok()) {
        let path = entry.path();
        if path.file_name().map(|n| n == "package.xml").unwrap_or(false) {
            if let Some(parent) = path.parent() {
                let config_dir = parent.join("config");
                if config_dir.exists() {
                    let has_srdf = fs::read_dir(&config_dir).ok().map(|mut it| {
                        it.any(|e| {
                            e.ok()
                                .and_then(|e| e.path().extension().map(|x| x == "srdf"))
                                .unwrap_or(false)
                        })
                    }).unwrap_or(false);
                    if has_srdf && seen.insert(parent.to_path_buf()) {
                        out.push(parent.to_path_buf());
                    }
                }
            }
        }
    }
    out
}

fn find_moveit_config_packages_to_skip(root: &Path) -> Vec<PathBuf> {
    let mut out: Vec<PathBuf> = Vec::new();
    let mut seen: HashSet<PathBuf> = HashSet::new();
    for entry in WalkDir::new(root).max_depth(4).into_iter().filter_map(|e| e.ok()) {
        let path = entry.path();
        if path.file_name().map(|n| n == "package.xml").unwrap_or(false) {
            if let Some(parent) = path.parent() {
                let name = parent.file_name().and_then(|n| n.to_str()).unwrap_or("");
                let mut is_moveit_config = name.ends_with("_moveit_config");
                if !is_moveit_config {
                    let config_dir = parent.join("config");
                    let has_srdf = fs::read_dir(&config_dir).ok().map(|mut it| {
                        it.any(|e| {
                            e.ok()
                                .and_then(|e| e.path().extension().map(|x| x == "srdf"))
                                .unwrap_or(false)
                        })
                    }).unwrap_or(false);
                    is_moveit_config = has_srdf;
                }

                if is_moveit_config && seen.insert(parent.to_path_buf()) {
                    out.push(parent.to_path_buf());
                }
            }
        }
    }
    out
}

fn find_joint_limits(config_pkg: &Path) -> Option<PathBuf> {
    let config_dir = config_pkg.join("config");
    for entry in fs::read_dir(&config_dir).ok()?.filter_map(|e| e.ok()) {
        let path = entry.path();
        if path.extension().map(|x| x == "yaml").unwrap_or(false) {
            if let Ok(content) = fs::read_to_string(&path) {
                if content.contains("joint_limits:") {
                    return Some(path);
                }
            }
        }
    }
    None
}

fn find_controller_config(config_pkg: &Path) -> Option<PathBuf> {
    let config_dir = config_pkg.join("config");
    for entry in fs::read_dir(&config_dir).ok()?.filter_map(|e| e.ok()) {
        let path = entry.path();
        if path.extension().map(|x| x == "yaml").unwrap_or(false) {
            if let Ok(content) = fs::read_to_string(&path) {
                if content.contains("controller_names") || content.contains("moveit_simple_controller_manager") {
                    return Some(path);
                }
            }
        }
    }
    None
}

fn patch_joint_limits(file_path: &Path) -> Result<String, String> {
    let content = fs::read_to_string(file_path).map_err(|e| e.to_string())?;
    let mut data: Value = serde_yaml::from_str(&content).map_err(|e| e.to_string())?;
    
    let jl = data.get_mut("joint_limits").ok_or("No joint_limits key")?;
    let map = jl.as_mapping_mut().ok_or("joint_limits is not a map")?;
    
    let mut changed = false;
    for (_joint, values) in map.iter_mut() {
        if let Some(v_map) = values.as_mapping_mut() {
            if let Some(vel) = v_map.get("max_velocity").and_then(|v| v.as_f64()) {
                let num: serde_yaml::Number = serde_yaml::Number::from(vel);
                v_map.insert(Value::String("max_velocity".into()), Value::Number(num.clone()));
                v_map.insert(Value::String("max_acceleration".into()), Value::Number(num));
                v_map.insert(Value::String("has_velocity_limits".into()), Value::Bool(true));
                v_map.insert(Value::String("has_acceleration_limits".into()), Value::Bool(true));
                changed = true;
            }
        }
    }
    
    if changed {
        let backup = file_path.with_extension("yaml.bak");
        fs::copy(file_path, &backup).ok();
        let yaml_str = serde_yaml::to_string(&data).map_err(|e| e.to_string())?;
        fs::write(file_path, yaml_str).map_err(|e| e.to_string())?;
        Ok("Joint limits updated (backup created)".into())
    } else {
        Ok("No changes needed".into())
    }
}

fn patch_controller_config(file_path: &Path) -> Result<String, String> {
    let content = fs::read_to_string(file_path).map_err(|e| e.to_string())?;
    let mut data: serde_yaml::Mapping = serde_yaml::from_str(&content).map_err(|e| e.to_string())?;
    
    let mut controllers: Vec<String> = Vec::new();
    let mut default_already_set = false;
    for (_key, value) in data.iter() {
        if let Some(mgr) = value.as_mapping() {
            if mgr.contains_key("controller_names") {
                if let Some(names) = mgr.get("controller_names").and_then(|v| v.as_sequence()) {
                    for (i, name) in names.iter().enumerate() {
                        if let Some(n) = name.as_str() {
                            if let Some(ctrl) = mgr.get(n).and_then(|v| v.as_mapping()) {
                                if ctrl.get("type").and_then(|v| v.as_str()) == Some("FollowJointTrajectory") {
                                    if ctrl.get("default").and_then(|v| v.as_bool()) == Some(true) {
                                        default_already_set = true;
                                    }
                                    if i == 0 || !controllers.contains(&n.to_string()) {
                                        controllers.push(n.to_string());
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    let preferred_default = if controllers.iter().any(|n| n == "arm_controller") {
        Some("arm_controller".to_string())
    } else {
        controllers.first().cloned()
    };
    
    let mut changed = false;
    let mut default_set = default_already_set;
    
    for (_key, value) in data.iter_mut() {
        if let Some(mgr) = value.as_mapping_mut() {
            if mgr.contains_key("controller_names") {
                for name in &controllers {
                    if let Some(ctrl) = mgr.get_mut(name).and_then(|v| v.as_mapping_mut()) {
                        if ctrl.get("action_ns").and_then(|v| v.as_str()) != Some("follow_joint_trajectory") {
                            ctrl.insert(
                                Value::String("action_ns".into()),
                                Value::String("follow_joint_trajectory".into()),
                            );
                            changed = true;
                        }
                        if !default_set {
                            if let Some(pref) = &preferred_default {
                                if pref == name {
                                    if ctrl.get("default").and_then(|v| v.as_bool()) != Some(true) {
                                        ctrl.insert(Value::String("default".into()), Value::Bool(true));
                                        changed = true;
                                    }
                                    default_set = true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    if changed {
        let backup = file_path.with_extension("yaml.bak");
        fs::copy(file_path, &backup).ok();
        let yaml_str = serde_yaml::to_string(&data).map_err(|e| e.to_string())?;
        fs::write(file_path, yaml_str).map_err(|e| e.to_string())?;
        Ok("Controllers updated (backup created)".into())
    } else {
        Ok("No changes needed".into())
    }
}

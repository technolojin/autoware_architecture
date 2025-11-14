import os
import shutil
import shlex
import tempfile
import logging

from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

logger = logging.getLogger(__name__)


def detect_terminal_method() -> str:
    """
    Detect available terminal method for launching separate terminals.
    
    Returns:
        'tmux': If tmux is available (works over SSH)
        'none': Fallback to regular launch
    """
    # Check for GUI availability (DISPLAY environment variable)
    has_display = os.environ.get('DISPLAY') is not None

    # Check if tmux is available (works over SSH)
    if shutil.which('tmux') is not None:
        return 'tmux'
    
    # Fallback to regular launch
    return 'none'


def detect_gui_terminal() -> str:
    """
    Detect available GUI terminal emulator for opening new terminal windows.
    
    Returns:
        Terminal command name if found, empty string otherwise
    """
    # Check for GUI availability
    if os.environ.get('DISPLAY') is None:
        return ''
    
    # Try common terminal emulators in order of preference
    terminals = ['gnome-terminal', 'xterm', 'konsole', 'xfce4-terminal', 'mate-terminal', 'terminator']
    
    for term in terminals:
        if shutil.which(term) is not None:
            return term
    
    return ''


def format_launch_args_for_command(context, launch_arguments_names: list[str]) -> list[str]:
    """
    Convert launch arguments to command-line format for ros2 launch command.
    
    Args:
        context: Launch context
        launch_arguments_names: List of launch argument names
        
    Returns:
        List of command-line arguments in format ['arg_name:=arg_value', ...]
    """
    args_list = []
    for arg_name in launch_arguments_names:
        arg_value = LaunchConfiguration(arg_name).perform(context)
        if arg_value is not None:
            # Escape special characters in values if needed
            args_list.append(f"{arg_name}:={arg_value}")
    return args_list


def escape_dollar_signs_for_bash(cmd_str: str) -> str:
    """
    Escape dollar signs in a command string so they're not interpreted by bash.
    
    This preserves ROS2 launch substitutions like $(var ...) and $(find-pkg-share ...)
    by escaping $ as \$ so bash treats them as literal dollar signs.
    """
    # Escape dollar signs: $ -> \$
    # This needs to be done before the string is used in bash
    return cmd_str.replace("$", "\\$")


def create_tmux_launcher_script(launcher_paths: list[str], launch_args_cmd: list[str], 
                                launcher_pkg_install_dir: str, 
                                launcher_pkg_name: str = 'tier4_perception_launch',
                                session_name: str = 'ros2_launchers',
                                window_name: str = 'launchers',
                                launch_pointcloud_container: bool = False) -> str:
    """
    Create a bash script that launches tmux with split panes and runs launchers.
    
    Args:
        launcher_paths: List of launcher file paths (relative to package install directory)
        launch_args_cmd: List of launch arguments in command-line format
        launcher_pkg_install_dir: Package install directory where launchers are located
        launcher_pkg_name: Package name for ros2 launch command
        session_name: Name for the tmux session
        window_name: Name for the tmux window
        launch_pointcloud_container: If True, launch pointcloud container in first pane
        
    Returns:
        Path to the created script file
    """
    # Build the script content
    script_content = "#!/bin/bash\n"
    script_content += "# Auto-generated tmux launcher script\n"
    script_content += "set -e\n"
    script_content += "set -x  # Enable command tracing for debugging\n\n"
    
    script_content += f"SESSION_NAME={shlex.quote(session_name)}\n"
    script_content += f"WINDOW_NAME={shlex.quote(window_name)}\n"
    
    # Calculate workspace install directory
    # launcher_pkg_install_dir is like /path/to/workspace/install/tier4_perception_launch/share/tier4_perception_launch
    # workspace_install_dir should be /path/to/workspace/install
    # Need to go up 3 levels: share/pkg -> pkg -> install
    workspace_install_dir = os.path.dirname(os.path.dirname(os.path.dirname(launcher_pkg_install_dir)))
    setup_bash_path = os.path.join(workspace_install_dir, 'setup.bash')
    script_content += f"WORKSPACE_SETUP={shlex.quote(setup_bash_path)}\n\n"
    
    # Kill existing session if it exists
    script_content += "# Kill existing session if it exists\n"
    script_content += "echo \"Checking for existing tmux session: $SESSION_NAME\"\n"
    script_content += f"if tmux has-session -t $SESSION_NAME 2>/dev/null; then\n"
    script_content += f"  echo \"Killing existing session: $SESSION_NAME\"\n"
    script_content += f"  tmux kill-session -t $SESSION_NAME || true\n"
    script_content += f"  sleep 0.5\n"
    script_content += f"fi\n\n"
    
    # Check if tmux is available
    script_content += "# Verify tmux is available\n"
    script_content += "if ! command -v tmux &> /dev/null; then\n"
    script_content += "  echo \"ERROR: tmux is not installed or not in PATH\" >&2\n"
    script_content += "  exit 1\n"
    script_content += "fi\n"
    script_content += "echo \"Using tmux: $(which tmux)\"\n"
    script_content += "echo \"Tmux version: $(tmux -V)\"\n\n"
    
    # Create session with first pane
    script_content += f"# Create session with first pane\n"
    script_content += f"echo \"Creating tmux session: $SESSION_NAME\"\n"
    # Calculate window size based on number of panes needed
    # Each pane needs at least ~10-15 lines, so we need more height for more panes
    total_panes = len(launcher_paths) + (1 if launch_pointcloud_container else 0)
    window_height = max(80, total_panes * 5)  # At least 80 lines, or 15 lines per pane
    window_width = 200  # Keep width at 200
    # Create session with bash shell and larger window size
    script_content += f"TMUX_ERROR=$(tmux new-session -d -s $SESSION_NAME -n $WINDOW_NAME -x {window_width} -y {window_height} 'bash' 2>&1)\n"
    script_content += f"TMUX_EXIT_CODE=$?\n"
    script_content += f"if [ $TMUX_EXIT_CODE -ne 0 ]; then\n"
    script_content += f"  echo \"ERROR: Failed to create tmux session (exit code: $TMUX_EXIT_CODE)\" >&2\n"
    script_content += f"  echo \"Tmux error output: $TMUX_ERROR\" >&2\n"
    script_content += f"  exit 1\n"
    script_content += f"fi\n"
    script_content += "if [ -n \"$TMUX_ERROR\" ]; then\n"
    script_content += "  echo \"Tmux warning: $TMUX_ERROR\" >&2\n"
    script_content += "fi\n"
    script_content += "sleep 0.5\n"
    script_content += "# Verify session exists\n"
    script_content += "if ! tmux has-session -t $SESSION_NAME 2>/dev/null; then\n"
    script_content += "  echo \"ERROR: Session was not created\" >&2\n"
    script_content += "  exit 1\n"
    script_content += "fi\n"
    
    # CRITICAL: Resize tmux window BEFORE any terminal attaches
    # Once a terminal attaches, it constrains the tmux window size
    # We must set the size while the session is detached
    script_content += "# Resize tmux window to ensure it's large enough for all panes\n"
    script_content += f"echo \"Resizing tmux window to {window_width}x{window_height} (before terminal attach)...\"\n"
    script_content += f"tmux resize-window -t $SESSION_NAME:0 -x {window_width} -y {window_height} 2>/dev/null || true\n"
    script_content += "sleep 0.3\n"
    
    # Open new terminal window to monitor tmux session (after resizing, before adding panes)
    gui_terminal = detect_gui_terminal()
    if gui_terminal:
        script_content += "# Open new terminal window to monitor tmux session\n"
        script_content += "echo \"Opening new terminal window to monitor tmux session...\"\n"
        # Use $SESSION_NAME (bash variable) in the command - need to escape $ for Python f-string
        session_var = "$SESSION_NAME"
        # Calculate terminal window size (geometry format: WIDTHxHEIGHT+X+Y)
        # Use the calculated window size for the terminal geometry
        term_cols = window_width
        term_rows = window_height
        if gui_terminal == 'gnome-terminal':
            # gnome-terminal: --geometry WIDTHxHEIGHT+X+Y
            script_content += f"gnome-terminal --geometry {term_cols}x{term_rows}+0+0 -- bash -c \"tmux attach-session -t {session_var} || bash\" &\n"
        elif gui_terminal == 'xterm':
            # xterm: -geometry WIDTHxHEIGHT+X+Y
            script_content += f"xterm -geometry {term_cols}x{term_rows} -e bash -c \"tmux attach-session -t {session_var} || bash\" &\n"
        elif gui_terminal == 'konsole':
            # Konsole: --geometry WIDTHxHEIGHT+X+Y
            script_content += f"konsole --geometry {term_cols}x{term_rows} -e bash -c \"tmux attach-session -t {session_var} || bash\" &\n"
        elif gui_terminal == 'xfce4-terminal':
            # xfce4-terminal: --geometry WIDTHxHEIGHT+X+Y
            script_content += f"xfce4-terminal --geometry {term_cols}x{term_rows} -e \"bash -c \\\"tmux attach-session -t {session_var} || bash\\\"\" &\n"
        elif gui_terminal == 'mate-terminal':
            # mate-terminal: --geometry WIDTHxHEIGHT+X+Y
            script_content += f"mate-terminal --geometry {term_cols}x{term_rows} -e \"bash -c \\\"tmux attach-session -t {session_var} || bash\\\"\" &\n"
        elif gui_terminal == 'terminator':
            # Terminator doesn't support geometry directly, but we can try to resize after
            script_content += f"terminator -e \"bash -c \\\"tmux attach-session -t {session_var} || bash\\\"\" &\n"
        script_content += "sleep 1\n"
        script_content += "echo \"Terminal window opened. You can monitor the tmux session there.\"\n"
        script_content += "echo \"\"\n"
    
    # Initialize LAUNCHERS_SKIPPED variable (used to track if we need to skip launchers)
    script_content += "# Initialize LAUNCHERS_SKIPPED variable\n"
    script_content += "LAUNCHERS_SKIPPED=false\n"
    
    # Launch first item in pane 0 (pointcloud container or first launcher)
    if launch_pointcloud_container:
        script_content += f"# Launch pointcloud container in pane 0\n"
        script_content += f"echo \"Launching pointcloud container in pane 0\"\n"
        # Build ros2 run command for pointcloud container
        # ros2 run rclcpp_components component_container --ros-args -r __node:=pointcloud_container --log-level info
        container_cmd_parts = ['ros2', 'run', 'rclcpp_components', 'component_container', 
                              '--ros-args', '-r', '__node:=pointcloud_container', '--log-level', 'info']
        # Join parts with spaces, quoting each part to handle spaces in values
        container_cmd_quoted = ' '.join(shlex.quote(part) for part in container_cmd_parts)
        # Combine source and container command: source setup.bash && ros2 run ...
        script_content += f"CONTAINER_CMD_PART={shlex.quote(container_cmd_quoted)}\n"
        script_content += f"CONTAINER_CMD=\"source $WORKSPACE_SETUP && $CONTAINER_CMD_PART\"\n"
        script_content += f"tmux send-keys -t $SESSION_NAME:0.0 \"$CONTAINER_CMD\" Enter\n"
        script_content += "sleep 1\n"
    
    # Launch first launcher if there are launchers and we haven't used pane 0 yet
    if launcher_paths and not launch_pointcloud_container:
        first_launcher_path = launcher_paths[0]
        first_launcher_name = os.path.basename(first_launcher_path)
        script_content += f"# Launch first launcher in pane 0\n"
        script_content += f"echo \"Launching launcher 0: {first_launcher_name}\"\n"
        # Build ros2 launch command
        # Use ros2 launch with full absolute path to the XML file
        # Construct full path: launcher_pkg_install_dir + launcher_path
        full_launcher_path = os.path.join(launcher_pkg_install_dir, first_launcher_path)
        launch_cmd_parts = ['ros2', 'launch', full_launcher_path]
        launch_cmd_parts.extend(launch_args_cmd)
        # Join parts with spaces, quoting each part to handle spaces in values
        # Then escape $ characters so ROS2 substitutions like $(var ...) are not interpreted by bash
        launch_cmd_quoted = ' '.join(shlex.quote(part) for part in launch_cmd_parts)
        launch_cmd_escaped = escape_dollar_signs_for_bash(launch_cmd_quoted)
        # Combine source and launch command: source setup.bash && ros2 launch ...
        # We need $WORKSPACE_SETUP to be expanded by bash, so we construct the command
        # so that the source part uses $WORKSPACE_SETUP directly (not quoted)
        # and the launch command part is properly escaped
        # Store launch command in a variable first
        script_content += f"LAUNCH_CMD_PART_0={shlex.quote(launch_cmd_escaped)}\n"
        # Then combine with source command - $WORKSPACE_SETUP will be expanded by bash
        script_content += f"LAUNCH_CMD_0=\"source $WORKSPACE_SETUP && $LAUNCH_CMD_PART_0\"\n"
        # Use the variable in tmux send-keys - bash will expand $WORKSPACE_SETUP and $LAUNCH_CMD_PART_0
        script_content += f"tmux send-keys -t $SESSION_NAME:0.0 \"$LAUNCH_CMD_0\" Enter\n"
        script_content += "sleep 1\n"
    elif launcher_paths and launch_pointcloud_container:
        # We already launched pointcloud container, now launch first launcher in a new pane
        first_launcher_path = launcher_paths[0]
        first_launcher_name = os.path.basename(first_launcher_path)
        script_content += f"# Split and add pane 1 with first launcher\n"
        script_content += f"echo \"Splitting window and adding pane 1: {first_launcher_name}\"\n"
        script_content += f"SPLIT_ERROR_0=$(tmux split-window -v -t $SESSION_NAME:0 'bash' 2>&1)\n"
        script_content += f"SPLIT_EXIT_CODE_0=$?\n"
        script_content += f"if [ $SPLIT_EXIT_CODE_0 -ne 0 ]; then\n"
        script_content += f"  echo \"WARNING: Failed to split window for pane 1: $SPLIT_ERROR_0\" >&2\n"
        script_content += f"  echo \"Continuing with pointcloud container only (launchers will not be started)\" >&2\n"
        script_content += f"  # Skip remaining launchers\n"
        script_content += f"  LAUNCHERS_SKIPPED=true\n"
        script_content += f"else\n"
        script_content += "sleep 0.3\n"
        # Build and send ros2 launch command to the newly created pane
        full_launcher_path = os.path.join(launcher_pkg_install_dir, first_launcher_path)
        launch_cmd_parts = ['ros2', 'launch', full_launcher_path]
        launch_cmd_parts.extend(launch_args_cmd)
        launch_cmd_quoted = ' '.join(shlex.quote(part) for part in launch_cmd_parts)
        launch_cmd_escaped = escape_dollar_signs_for_bash(launch_cmd_quoted)
        script_content += f"  LAUNCH_CMD_PART_0={shlex.quote(launch_cmd_escaped)}\n"
        script_content += f"  LAUNCH_CMD_0=\"source $WORKSPACE_SETUP && $LAUNCH_CMD_PART_0\"\n"
        script_content += f"  tmux send-keys -t $SESSION_NAME:0. \"$LAUNCH_CMD_0\" Enter\n"
        script_content += f"  sleep 1\n"
        script_content += f"fi\n\n"
    
    script_content += "echo \"Session created successfully\"\n"
    script_content += "echo \"Session info: $(tmux list-sessions | grep $SESSION_NAME)\"\n\n"
    
    # Split and add remaining panes with launchers
    # Stack panes from top to bottom using vertical splits only
    # Start from index 1 since first launcher is already handled (either in pane 0 or pane 1)
    script_content += "# Track successful pane creations\n"
    # Count initial panes based on what was already created
    if launch_pointcloud_container:
        if launcher_paths:
            # Pointcloud container in pane 0, first launcher in pane 1 (if split succeeded)
            script_content += "if [ \"$LAUNCHERS_SKIPPED\" = \"false\" ]; then\n"
            script_content += "  SUCCESSFUL_PANES=2\n"  # Pointcloud container + first launcher
            script_content += "else\n"
            script_content += "  SUCCESSFUL_PANES=1\n"  # Only pointcloud container
            script_content += "fi\n"
        else:
            script_content += "SUCCESSFUL_PANES=1\n"  # Only pointcloud container
    else:
        if launcher_paths:
            script_content += "SUCCESSFUL_PANES=1\n"  # First launcher in pane 0
        else:
            script_content += "SUCCESSFUL_PANES=0\n"  # No panes yet (shouldn't happen)
    
    # Only try to add remaining launchers if we haven't skipped them
    script_content += "if [ \"$LAUNCHERS_SKIPPED\" != \"true\" ]; then\n"
    for i, launcher_path in enumerate(launcher_paths[1:], 1):
        launcher_name = os.path.basename(launcher_path)
        
        # Always use vertical splits to stack panes top to bottom
        script_content += f"# Split and add pane {i+1} with launcher\n"
        script_content += f"echo \"Splitting window and adding pane {i+1}: {launcher_name}\"\n"
        # Split window vertically - the new pane becomes the active one
        script_content += f"SPLIT_ERROR=$(tmux split-window -v -t $SESSION_NAME:0 'bash' 2>&1)\n"
        script_content += f"SPLIT_EXIT_CODE=$?\n"
        script_content += f"if [ $SPLIT_EXIT_CODE -ne 0 ]; then\n"
        script_content += f"  echo \"WARNING: Failed to split window for pane {i+1}: $SPLIT_ERROR\" >&2\n"
        script_content += f"  echo \"Continuing with {i} panes (some launchers may not be started)\" >&2\n"
        script_content += f"  break\n"
        script_content += f"fi\n"
        script_content += "SUCCESSFUL_PANES=$((SUCCESSFUL_PANES + 1))\n"
        script_content += "sleep 0.3\n"
        # Build and send ros2 launch command to the newly created pane
        # Use ros2 launch with full absolute path to the XML file
        # Construct full path: launcher_pkg_install_dir + launcher_path
        full_launcher_path = os.path.join(launcher_pkg_install_dir, launcher_path)
        launch_cmd_parts = ['ros2', 'launch', full_launcher_path]
        launch_cmd_parts.extend(launch_args_cmd)
        # Join parts with spaces, quoting each part to handle spaces in values
        # Then escape $ characters so ROS2 substitutions like $(var ...) are not interpreted by bash
        launch_cmd_quoted = ' '.join(shlex.quote(part) for part in launch_cmd_parts)
        launch_cmd_escaped = escape_dollar_signs_for_bash(launch_cmd_quoted)
        # Combine source and launch command: source setup.bash && ros2 launch ...
        # We need $WORKSPACE_SETUP to be expanded by bash, so we construct the command
        # so that the source part uses $WORKSPACE_SETUP directly (not quoted)
        # and the launch command part is properly escaped
        # Store launch command in a variable first
        script_content += f"LAUNCH_CMD_PART_{i}={shlex.quote(launch_cmd_escaped)}\n"
        # Then combine with source command - $WORKSPACE_SETUP will be expanded by bash
        script_content += f"LAUNCH_CMD_{i}=\"source $WORKSPACE_SETUP && $LAUNCH_CMD_PART_{i}\"\n"
        # Use the variable in tmux send-keys - bash will expand $WORKSPACE_SETUP and $LAUNCH_CMD_PART_{i}
        script_content += f"tmux send-keys -t $SESSION_NAME:0. \"$LAUNCH_CMD_{i}\" Enter\n"
        script_content += "sleep 1\n\n"
    script_content += "fi\n"  # End of if LAUNCHERS_SKIPPED check
    
    # Make all panes evenly sized
    script_content += "# Make all panes evenly sized\n"
    script_content += "echo \"Making panes evenly sized...\"\n"
    script_content += f"tmux select-layout -t $SESSION_NAME:0 even-vertical\n"
    script_content += "sleep 0.3\n"
    
    total_panes = len(launcher_paths) + (1 if launch_pointcloud_container else 0)
    script_content += f"echo \"Tmux session $SESSION_NAME created with $SUCCESSFUL_PANES panes (requested {total_panes})\"\n"
    if launch_pointcloud_container:
        script_content += f"if [ $SUCCESSFUL_PANES -lt {total_panes} ]; then\n"
        script_content += f"  echo \"WARNING: Only $SUCCESSFUL_PANES panes created out of {total_panes} requested\" >&2\n"
        script_content += f"  echo \"Some launchers may not have been started. Try resizing the terminal or reducing the number of launchers.\" >&2\n"
        script_content += f"fi\n"
        script_content += f"echo \"Launched pointcloud container and $((SUCCESSFUL_PANES - 1)) ROS2 launchers in separate panes\"\n"
    else:
        script_content += f"if [ $SUCCESSFUL_PANES -lt {total_panes} ]; then\n"
        script_content += f"  echo \"WARNING: Only $SUCCESSFUL_PANES panes created out of {total_panes} requested\" >&2\n"
        script_content += f"  echo \"Some launchers may not have been started. Try resizing the terminal or reducing the number of launchers.\" >&2\n"
        script_content += f"fi\n"
        script_content += f"echo \"Launched $SUCCESSFUL_PANES ROS2 launchers in separate panes\"\n"
    script_content += "echo \"\"\n"
    
    # Try to automatically attach to the tmux session (only if GUI terminal wasn't opened)
    script_content += "# Try to automatically attach to tmux session (if GUI terminal wasn't used)\n"
    if not gui_terminal:
        # No GUI terminal available, try to attach in current terminal
        script_content += "if [ -z \"$TMUX\" ]; then\n"
        script_content += "  # Not already in tmux, check if we have a real terminal\n"
        script_content += "  if [ -t 0 ] && [ -t 1 ]; then\n"
        script_content += "    # We have a real terminal, try to attach\n"
        script_content += "    echo \"Attaching to tmux session...\"\n"
        script_content += "    echo \"(You can detach with Ctrl+B, then D)\"\n"
        script_content += "    exec tmux attach-session -t $SESSION_NAME 2>/dev/null\n"
        script_content += "    # If we reach here, exec failed\n"
        script_content += "    echo \"Attachment failed - session is running in background\"\n"
        script_content += "  else\n"
        script_content += "    echo \"No terminal available - session is running in background\"\n"
        script_content += "  fi\n"
        script_content += "  echo \"Attach manually with: tmux attach-session -t $SESSION_NAME\"\n"
        script_content += "else\n"
        script_content += "  # Already in tmux\n"
        script_content += "  echo \"Already in tmux session. Switch to launcher session with:\"\n"
        script_content += "  echo \"  tmux switch-client -t $SESSION_NAME\"\n"
        script_content += "fi\n"
    else:
        # GUI terminal was opened, skip exec attempt
        script_content += "# GUI terminal window was opened to monitor the session\n"
        script_content += "# No need to attach in this terminal - session is running in background\n"
        script_content += "echo \"Session is running. Monitor it in the opened terminal window.\"\n"
        script_content += "if [ -n \"$TMUX\" ]; then\n"
        script_content += "  echo \"(You're already in tmux. Switch with: tmux switch-client -t $SESSION_NAME)\"\n"
        script_content += "fi\n"
    script_content += "echo \"\"\n"
    
    # Setup signal handler for graceful termination
    script_content += "# Setup signal handler for graceful termination\n"
    script_content += "cleanup_and_exit() {\n"
    script_content += "  echo \"\"\n"
    script_content += "  echo \"Received termination signal (Ctrl+C). Shutting down all launchers...\"\n"
    script_content += "  \n"
    script_content += "  # Check if session still exists\n"
    script_content += "  if tmux has-session -t $SESSION_NAME 2>/dev/null; then\n"
    script_content += "    # Get list of panes in the session (format: window.pane)\n"
    script_content += "    PANES=$(tmux list-panes -t $SESSION_NAME -F '#{window_index}.#{pane_index}' 2>/dev/null || true)\n"
    script_content += "    \n"
    script_content += "    if [ -n \"$PANES\" ]; then\n"
    script_content += "      echo \"Sending Ctrl+C to all panes...\"\n"
    script_content += "      # Send Ctrl+C (SIGINT) to each pane\n"
    script_content += "      for pane in $PANES; do\n"
    script_content += "        tmux send-keys -t $SESSION_NAME:$pane C-c 2>/dev/null || true\n"
    script_content += "      done\n"
    script_content += "      \n"
    script_content += "      # Wait a bit for processes to handle the signal\n"
    script_content += "      sleep 2\n"
    script_content += "      \n"
    script_content += "      # Force kill the session if it still exists\n"
    script_content += "      echo \"Killing tmux session...\"\n"
    script_content += "      tmux kill-session -t $SESSION_NAME 2>/dev/null || true\n"
    script_content += "      sleep 0.5\n"
    script_content += "    else\n"
    script_content += "      # No panes found, just kill the session\n"
    script_content += "      echo \"Killing tmux session...\"\n"
    script_content += "      tmux kill-session -t $SESSION_NAME 2>/dev/null || true\n"
    script_content += "    fi\n"
    script_content += "  fi\n"
    script_content += "  \n"
    script_content += "  echo \"All launchers terminated.\"\n"
    script_content += "  exit 0\n"
    script_content += "}\n"
    script_content += "\n"
    script_content += "# Trap SIGINT (Ctrl+C) and SIGTERM\n"
    script_content += "trap cleanup_and_exit SIGINT SIGTERM\n"
    script_content += "\n"
    script_content += "# Keep script running to handle termination signals\n"
    script_content += "# The main terminal stays active to propagate Ctrl+C to all sessions\n"
    script_content += "echo \"System is running. Press Ctrl+C to terminate all launchers.\"\n"
    script_content += "echo \"\"\n"
    script_content += "\n"
    script_content += "# Wait loop - keep script running until signal is received\n"
    script_content += "while true; do\n"
    script_content += "  # Check if session still exists\n"
    script_content += "  if ! tmux has-session -t $SESSION_NAME 2>/dev/null; then\n"
    script_content += "    echo \"Tmux session $SESSION_NAME no longer exists. Exiting...\"\n"
    script_content += "    exit 0\n"
    script_content += "  fi\n"
    script_content += "  # Sleep and check again\n"
    script_content += "  sleep 1\n"
    script_content += "done\n"
    
    # Write script to temporary file
    fd, script_path = tempfile.mkstemp(suffix='.sh', prefix='tmux_launcher_')
    with os.fdopen(fd, 'w') as f:
        f.write(script_content)
    os.chmod(script_path, 0o755)
    
    return script_path


def launch_in_tmux(context, launch_arguments_names: list[str], launcher_paths: list[str],
                   launcher_pkg_install_dir: str,
                   launcher_pkg_name: str = 'tier4_perception_launch',
                   session_name: str = 'ros2_launchers',
                   window_name: str = 'launchers',
                   launch_pointcloud_container: bool = False) -> list[ExecuteProcess]:
    """
    Launch multiple launchers in a single tmux session with split panes.
    
    Args:
        context: Launch context
        launch_arguments_names: List of launch argument names
        launcher_paths: List of launcher file paths (relative to package install directory)
        launcher_pkg_install_dir: Package install directory where launchers are located
        launcher_pkg_name: Package name for ros2 launch command
        session_name: Name for the tmux session
        window_name: Name for the tmux window
        launch_pointcloud_container: If True, launch pointcloud container in first pane
        
    Returns:
        List of ExecuteProcess actions (single action that runs the tmux script)
    """
    # Convert launch arguments to command-line format
    launch_args_cmd = format_launch_args_for_command(context, launch_arguments_names)
    
    # Create the tmux launcher script
    script_path = create_tmux_launcher_script(
        launcher_paths, launch_args_cmd, launcher_pkg_install_dir,
        launcher_pkg_name, session_name, window_name, launch_pointcloud_container
    )
    
    # Return the action to execute the script
    return [
        ExecuteProcess(
            cmd=['bash', script_path],
            output='screen'
        )
    ]


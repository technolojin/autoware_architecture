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
                                window_name: str = 'launchers') -> str:
    """
    Create a bash script that launches tmux with split panes and runs launchers.
    
    Args:
        launcher_paths: List of launcher file paths (relative to package install directory)
        launch_args_cmd: List of launch arguments in command-line format
        launcher_pkg_install_dir: Package install directory where launchers are located
        launcher_pkg_name: Package name for ros2 launch command
        session_name: Name for the tmux session
        window_name: Name for the tmux window
        
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
    # Create session with bash shell
    script_content += f"TMUX_ERROR=$(tmux new-session -d -s $SESSION_NAME -n $WINDOW_NAME -x 200 -y 50 'bash' 2>&1)\n"
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
    
    # Open new terminal window to monitor tmux session (do this early, before adding panes)
    gui_terminal = detect_gui_terminal()
    if gui_terminal:
        script_content += "# Open new terminal window to monitor tmux session\n"
        script_content += "echo \"Opening new terminal window to monitor tmux session...\"\n"
        # Wait a moment for session to be fully ready, then open terminal
        script_content += "sleep 0.5\n"
        # Use $SESSION_NAME (bash variable) in the command - need to escape $ for Python f-string
        session_var = "$SESSION_NAME"
        if gui_terminal == 'gnome-terminal':
            # Use -- to separate options from command, use double quotes so $SESSION_NAME expands
            script_content += f"gnome-terminal -- bash -c \"tmux attach-session -t {session_var} || bash\" &\n"
        elif gui_terminal == 'xterm':
            script_content += f"xterm -e bash -c \"tmux attach-session -t {session_var} || bash\" &\n"
        elif gui_terminal == 'konsole':
            script_content += f"konsole -e bash -c \"tmux attach-session -t {session_var} || bash\" &\n"
        elif gui_terminal == 'xfce4-terminal':
            script_content += f"xfce4-terminal -e \"bash -c \\\"tmux attach-session -t {session_var} || bash\\\"\" &\n"
        elif gui_terminal == 'mate-terminal':
            script_content += f"mate-terminal -e \"bash -c \\\"tmux attach-session -t {session_var} || bash\\\"\" &\n"
        elif gui_terminal == 'terminator':
            script_content += f"terminator -e \"bash -c \\\"tmux attach-session -t {session_var} || bash\\\"\" &\n"
        script_content += "sleep 1\n"
        script_content += "echo \"Terminal window opened. You can monitor the tmux session there.\"\n"
        script_content += "echo \"\"\n"
    
    # Launch first launcher in pane 0
    if launcher_paths:
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
    
    script_content += "echo \"Session created successfully\"\n"
    script_content += "echo \"Session info: $(tmux list-sessions | grep $SESSION_NAME)\"\n\n"
    
    # Split and add remaining panes with launchers
    # Stack panes from top to bottom using vertical splits only
    for i, launcher_path in enumerate(launcher_paths[1:], 1):
        launcher_name = os.path.basename(launcher_path)
        
        # Always use vertical splits to stack panes top to bottom
        script_content += f"# Split and add pane {i+1} with launcher\n"
        script_content += f"echo \"Splitting window and adding pane {i+1}: {launcher_name}\"\n"
        # Split window vertically - the new pane becomes the active one
        script_content += f"if ! tmux split-window -v -t $SESSION_NAME:0 'bash' 2>&1; then\n"
        script_content += f"  echo \"ERROR: Failed to split window for pane {i+1}\" >&2\n"
        script_content += f"  exit 1\n"
        script_content += f"fi\n"
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
    
    # Make all panes evenly sized
    script_content += "# Make all panes evenly sized\n"
    script_content += "echo \"Making panes evenly sized...\"\n"
    script_content += f"tmux select-layout -t $SESSION_NAME:0 even-vertical\n"
    script_content += "sleep 0.3\n"
    
    script_content += f"echo \"Tmux session $SESSION_NAME created with {len(launcher_paths)} panes\"\n"
    script_content += f"echo \"Launched {len(launcher_paths)} ROS2 launchers in separate panes\"\n"
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
    
    # Keep script running to maintain the session (only if exec above didn't work)
    script_content += "# Keep script running to maintain the session\n"
    script_content += "# (This only runs if exec above didn't replace the process)\n"
    script_content += "trap 'echo \"Tmux launcher script stopped, but tmux session remains\"' EXIT\n"
    script_content += "while tmux has-session -t $SESSION_NAME 2>/dev/null; do\n"
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
                   window_name: str = 'launchers') -> list[ExecuteProcess]:
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
        
    Returns:
        List of ExecuteProcess actions (single action that runs the tmux script)
    """
    # Convert launch arguments to command-line format
    launch_args_cmd = format_launch_args_for_command(context, launch_arguments_names)
    
    # Create the tmux launcher script
    script_path = create_tmux_launcher_script(
        launcher_paths, launch_args_cmd, launcher_pkg_install_dir,
        launcher_pkg_name, session_name, window_name
    )
    
    # Return the action to execute the script
    return [
        ExecuteProcess(
            cmd=['bash', script_path],
            output='screen'
        )
    ]


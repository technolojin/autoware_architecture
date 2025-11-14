import os
import shutil
import shlex
import tempfile
import logging

from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

logger = logging.getLogger(__name__)


def detect_terminal_method() -> str:
    """Detect available terminal method for launching separate terminals."""
    if shutil.which('terminator') is not None:
        return 'terminator'
    elif shutil.which('tmux') is not None:
        return 'tmux'
    else:
        return 'none'


def detect_gui_terminal() -> str:
    """Detect available GUI terminal emulator."""
    if os.environ.get('DISPLAY') is None:
        return ''
    terminals = ['gnome-terminal', 'xterm', 'konsole', 'xfce4-terminal', 'mate-terminal', 'terminator']
    return next((t for t in terminals if shutil.which(t)), '')


def format_launch_args_for_command(context, launch_arguments_names: list[str]) -> list[str]:
    """Convert launch arguments to command-line format."""
    args_list = []
    for arg_name in launch_arguments_names:
        arg_value = LaunchConfiguration(arg_name).perform(context)
        if arg_value is not None:
            args_list.append(f"{arg_name}:={arg_value}")
    return args_list


def escape_dollar_signs_for_bash(cmd_str: str) -> str:
    """Escape dollar signs to preserve ROS2 launch substitutions."""
    return cmd_str.replace("$", "\\$")


def create_tmux_launcher_script(launcher_paths: list[str], launch_args_cmd: list[str], 
                                launcher_pkg_install_dir: str, 
                                launcher_pkg_name: str = 'tier4_perception_launch',
                                session_name: str = 'ros2_launchers',
                                window_name: str = 'launchers',
                                launch_pointcloud_container: bool = False) -> str:
    """Create a bash script that launches tmux with split panes and runs launchers."""
    script_content = "#!/bin/bash\nset -e\n\n"
    script_content += f"SESSION_NAME={shlex.quote(session_name)}\n"
    script_content += f"WINDOW_NAME={shlex.quote(window_name)}\n"
    
    workspace_install_dir = os.path.dirname(os.path.dirname(os.path.dirname(launcher_pkg_install_dir)))
    setup_bash_path = os.path.join(workspace_install_dir, 'setup.bash')
    script_content += f"WORKSPACE_SETUP={shlex.quote(setup_bash_path)}\n\n"
    
    script_content += f"tmux has-session -t $SESSION_NAME 2>/dev/null && tmux kill-session -t $SESSION_NAME || true\n"
    script_content += "command -v tmux &> /dev/null || { echo 'ERROR: tmux not found' >&2; exit 1; }\n\n"
    
    total_panes = len(launcher_paths) + (1 if launch_pointcloud_container else 0)
    window_height = max(80, total_panes * 5)
    window_width = 200
    
    script_content += f"tmux new-session -d -s $SESSION_NAME -n $WINDOW_NAME -x {window_width} -y {window_height} 'bash' || {{ echo 'ERROR: Failed to create tmux session' >&2; exit 1; }}\n"
    script_content += f"tmux resize-window -t $SESSION_NAME:0 -x {window_width} -y {window_height} 2>/dev/null || true\n"
    script_content += "sleep 0.3\n"
    
    gui_terminal = detect_gui_terminal()
    session_var = "$SESSION_NAME"
    if gui_terminal:
        term_commands = {
            'gnome-terminal': f"gnome-terminal --geometry {window_width}x{window_height}+10+10 -- bash -c \"tmux attach-session -t {session_var} || bash\"",
            'xterm': f"xterm -geometry {window_width}x{window_height} -e bash -c \"tmux attach-session -t {session_var} || bash\"",
            'konsole': f"konsole --geometry {window_width}x{window_height} -e bash -c \"tmux attach-session -t {session_var} || bash\"",
            'xfce4-terminal': f"xfce4-terminal --geometry {window_width}x{window_height} -e \"bash -c \\\"tmux attach-session -t {session_var} || bash\\\"\"",
            'mate-terminal': f"mate-terminal --geometry {window_width}x{window_height} -e \"bash -c \\\"tmux attach-session -t {session_var} || bash\\\"\"",
            'terminator': f"terminator -e \"bash -c \\\"tmux attach-session -t {session_var} || bash\\\"\""
        }
        script_content += f"{term_commands.get(gui_terminal, '')} &\n"
        script_content += "sleep 1\n"
    
    script_content += "LAUNCHERS_SKIPPED=false\n"
    
    def build_cmd_part(cmd_parts):
        """Build command part (without source) that will be combined in bash script."""
        cmd_quoted = ' '.join(shlex.quote(p) for p in cmd_parts)
        return escape_dollar_signs_for_bash(cmd_quoted)
    
    if launch_pointcloud_container:
        container_cmd_part = build_cmd_part(['ros2', 'run', 'rclcpp_components', 'component_container',
                                            '--ros-args', '-r', '__node:=pointcloud_container', '--log-level', 'info'])
        script_content += f"CONTAINER_CMD_PART={shlex.quote(container_cmd_part)}\n"
        script_content += f"CONTAINER_CMD=\"source $WORKSPACE_SETUP && $CONTAINER_CMD_PART\"\n"
        script_content += f"echo \"Launching pointcloud container\"\n"
        script_content += f"tmux send-keys -t $SESSION_NAME:0.0 \"$CONTAINER_CMD\" Enter\n"
        script_content += "sleep 1\n"
    
    if launcher_paths:
        first_launcher_path = launcher_paths[0]
        first_launcher_name = os.path.basename(first_launcher_path)
        full_launcher_path = os.path.join(launcher_pkg_install_dir, first_launcher_path)
        launch_cmd_part = build_cmd_part(['ros2', 'launch', full_launcher_path] + launch_args_cmd)
        
        if not launch_pointcloud_container:
            script_content += f"LAUNCH_CMD_PART_0={shlex.quote(launch_cmd_part)}\n"
            script_content += f"LAUNCH_CMD_0=\"source $WORKSPACE_SETUP && $LAUNCH_CMD_PART_0\"\n"
            script_content += f"echo \"Launching: {first_launcher_name}\"\n"
            script_content += f"tmux send-keys -t $SESSION_NAME:0.0 \"$LAUNCH_CMD_0\" Enter\n"
            script_content += "sleep 1\n"
        else:
            script_content += f"tmux split-window -v -t $SESSION_NAME:0 'bash' || {{ LAUNCHERS_SKIPPED=true; }}\n"
            script_content += "[ \"$LAUNCHERS_SKIPPED\" = \"false\" ] && {\n"
            script_content += f"  LAUNCH_CMD_PART_0={shlex.quote(launch_cmd_part)}\n"
            script_content += f"  LAUNCH_CMD_0=\"source $WORKSPACE_SETUP && $LAUNCH_CMD_PART_0\"\n"
            script_content += f"  echo \"Launching: {first_launcher_name}\"\n"
            script_content += f"  tmux send-keys -t $SESSION_NAME:0. \"$LAUNCH_CMD_0\" Enter\n"
            script_content += "  sleep 1\n"
            script_content += "}\n"
    
    script_content += "SUCCESSFUL_PANES=$((1"
    if launch_pointcloud_container and launcher_paths:
        script_content += " + 1"
    script_content += "))\n"
    
    script_content += "[ \"$LAUNCHERS_SKIPPED\" != \"true\" ] && {\n"
    for i, launcher_path in enumerate(launcher_paths[1:], 1):
        launcher_name = os.path.basename(launcher_path)
        full_launcher_path = os.path.join(launcher_pkg_install_dir, launcher_path)
        launch_cmd_part = build_cmd_part(['ros2', 'launch', full_launcher_path] + launch_args_cmd)
        
        script_content += f"  tmux split-window -v -t $SESSION_NAME:0 'bash' || break\n"
        script_content += f"  LAUNCH_CMD_PART_{i}={shlex.quote(launch_cmd_part)}\n"
        script_content += f"  LAUNCH_CMD_{i}=\"source $WORKSPACE_SETUP && $LAUNCH_CMD_PART_{i}\"\n"
        script_content += f"  echo \"Launching: {launcher_name}\"\n"
        script_content += f"  tmux send-keys -t $SESSION_NAME:0. \"$LAUNCH_CMD_{i}\" Enter\n"
        script_content += "  SUCCESSFUL_PANES=$((SUCCESSFUL_PANES + 1))\n"
        script_content += "  sleep 1\n"
    script_content += "}\n"
    
    script_content += f"tmux select-layout -t $SESSION_NAME:0 even-vertical\n"
    script_content += f"echo \"Launched $SUCCESSFUL_PANES/{total_panes} panes\"\n\n"
    
    if not gui_terminal:
        script_content += "[ -z \"$TMUX\" ] && [ -t 0 ] && [ -t 1 ] && exec tmux attach-session -t $SESSION_NAME 2>/dev/null || true\n"
    
    script_content += "cleanup_and_exit() {\n"
    script_content += "  if tmux has-session -t $SESSION_NAME 2>/dev/null; then\n"
    script_content += "    for pane in $(tmux list-panes -t $SESSION_NAME -F '#{window_index}.#{pane_index}' 2>/dev/null); do\n"
    script_content += "      tmux send-keys -t $SESSION_NAME:$pane C-c 2>/dev/null || true\n"
    script_content += "    done\n"
    script_content += "    sleep 2\n"
    script_content += "    tmux kill-session -t $SESSION_NAME 2>/dev/null || true\n"
    script_content += "  fi\n"
    script_content += "  exit 0\n"
    script_content += "}\n"
    script_content += "trap cleanup_and_exit SIGINT SIGTERM\n"
    script_content += "while tmux has-session -t $SESSION_NAME 2>/dev/null; do sleep 1; done\n"
    
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
    """Launch multiple launchers in a single tmux session with split panes."""
    launch_args_cmd = format_launch_args_for_command(context, launch_arguments_names)
    script_path = create_tmux_launcher_script(
        launcher_paths, launch_args_cmd, launcher_pkg_install_dir,
        launcher_pkg_name, session_name, window_name, launch_pointcloud_container
    )
    return [ExecuteProcess(cmd=['bash', script_path], output='screen')]


def create_terminator_launcher_script(launcher_paths: list[str], launch_args_cmd: list[str],
                                     launcher_pkg_install_dir: str,
                                     launcher_pkg_name: str = 'tier4_perception_launch',
                                     layout_name: str = 'ros2_launchers',
                                     launch_pointcloud_container: bool = False) -> str:
    """Create a bash script that launches terminator with a layout and runs launchers."""
    script_content = "#!/bin/bash\nset -e\n\n"
    
    workspace_install_dir = os.path.dirname(os.path.dirname(os.path.dirname(launcher_pkg_install_dir)))
    setup_bash_path = os.path.join(workspace_install_dir, 'setup.bash')
    script_content += f"WORKSPACE_SETUP={shlex.quote(setup_bash_path)}\n"
    script_content += f"LAYOUT_NAME={shlex.quote(layout_name)}\n\n"
    
    script_content += "command -v terminator &> /dev/null || { echo 'ERROR: terminator not found' >&2; exit 1; }\n\n"
    
    total_panes = len(launcher_paths) + (1 if launch_pointcloud_container else 0)
    script_content += f"echo \"Launching terminator with {total_panes} panes\"\n\n"
    
    # Build commands for each launcher
    commands = []
    
    def build_cmd_part(cmd_parts):
        """Build command part (without source) that will be combined in bash script."""
        cmd_quoted = ' '.join(shlex.quote(p) for p in cmd_parts)
        return escape_dollar_signs_for_bash(cmd_quoted)
    
    if launch_pointcloud_container:
        container_cmd_part = build_cmd_part(['ros2', 'run', 'rclcpp_components', 'component_container',
                                            '--ros-args', '-r', '__node:=pointcloud_container', '--log-level', 'info'])
        commands.append(('pointcloud_container', container_cmd_part))
    
    for launcher_path in launcher_paths:
        launcher_name = os.path.basename(launcher_path)
        full_launcher_path = os.path.join(launcher_pkg_install_dir, launcher_path)
        launch_cmd_part = build_cmd_part(['ros2', 'launch', full_launcher_path] + launch_args_cmd)
        commands.append((launcher_name, launch_cmd_part))
    
    # Create terminator config with profiles and layout
    # Terminator uses INI-style config with [profiles] and [layouts] sections
    script_content += "TERMINATOR_CONFIG_DIR=\"$HOME/.config/terminator\"\n"
    script_content += "TERMINATOR_CONFIG_FILE=\"$TERMINATOR_CONFIG_DIR/config\"\n"
    script_content += "mkdir -p \"$TERMINATOR_CONFIG_DIR\"\n"
    script_content += "# Backup existing config if it exists\n"
    script_content += "if [ -f \"$TERMINATOR_CONFIG_FILE\" ]; then\n"
    script_content += "  cp \"$TERMINATOR_CONFIG_FILE\" \"$TERMINATOR_CONFIG_FILE.bak.$$\"\n"
    script_content += "fi\n"
    
    # Create profiles section for each launcher
    # Use unquoted heredoc so $WORKSPACE_SETUP expands
    # For \$ in commands, we need \\$ so bash interprets it as \$ in the final command
    script_content += "# Create profiles for each launcher\n"
    script_content += "PROFILES_SECTION=$(cat << PROFILES_EOF\n"
    
    # Generate profiles for each command
    for i, (title, cmd) in enumerate(commands, 1):
        profile_name = f"{layout_name}_profile_{i}"
        # The cmd has \$ for ROS2 substitutions (from escape_dollar_signs_for_bash)
        # and ${WORKSPACE_SETUP} which should expand
        # In unquoted heredoc, \$ becomes $ (bash interprets backslash-dollar as dollar)
        # But we want ROS2 to see $ for substitutions, so we need \$ in the final command
        # Solution: In unquoted heredoc, \\$ becomes \$ (backslash escapes backslash, then dollar)
        # So we need to double-escape: convert \$ to \\$
        cmd_double_escaped = cmd.replace('\\$', '\\\\$')
        # Escape double quotes for INI format
        cmd_escaped_for_ini = cmd_double_escaped.replace('"', '\\"')
        bash_cmd = f'bash -ic "{cmd_escaped_for_ini}; bash"'
        # Escape double quotes in the bash command for INI format
        bash_cmd_escaped = bash_cmd.replace('"', '\\"')
        script_content += f"  [[{profile_name}]]\n"
        script_content += f'    custom_command = "{bash_cmd_escaped}"\n'
    
    script_content += "PROFILES_EOF\n"
    script_content += ")\n\n"
    
    # Create layout section
    script_content += "# Create layout section\n"
    script_content += "LAYOUT_SECTION=$(cat << 'LAYOUT_EOF'\n"
    
    # Generate layout in terminator's INI format
    script_content += f"  [[{layout_name}]]\n"
    script_content += "    [[[window0]]]\n"
    script_content += "      type = Window\n"
    script_content += "      parent = \"\"\n"
    
    if len(commands) == 0:
        # Empty layout
        script_content += "    [[[child1]]]\n"
        script_content += "      type = Terminal\n"
        script_content += "      parent = window0\n"
        script_content += "      profile = default\n"
    elif len(commands) == 1:
        # Single terminal
        title, _ = commands[0]
        profile_name = f"{layout_name}_profile_1"
        script_content += f"    [[[child1]]]\n"
        script_content += f"      type = Terminal\n"
        script_content += f"      parent = window0\n"
        script_content += f"      profile = {profile_name}\n"
        script_content += f"      title = {shlex.quote(title)}\n"
    else:
        # Multiple terminals - create vertical split using VPaned
        script_content += "    [[[child1]]]\n"
        script_content += "      type = VPaned\n"
        script_content += "      parent = window0\n"
        
        # Add terminals as children of the VPaned
        for i, (title, _) in enumerate(commands, 1):
            profile_name = f"{layout_name}_profile_{i}"
            script_content += f"    [[[[child{i}]]]]\n"
            script_content += f"      type = Terminal\n"
            script_content += f"      parent = child1\n"
            script_content += f"      profile = {profile_name}\n"
            script_content += f"      title = {shlex.quote(title)}\n"
    
    script_content += "LAYOUT_EOF\n"
    script_content += ")\n\n"
    
    # Merge profiles and layout into config file
    script_content += "# Check if [profiles] section exists\n"
    script_content += "if ! grep -q '^\\[profiles\\]' \"$TERMINATOR_CONFIG_FILE\" 2>/dev/null; then\n"
    script_content += "  # Add [profiles] section if it doesn't exist\n"
    script_content += "  echo '' >> \"$TERMINATOR_CONFIG_FILE\"\n"
    script_content += "  echo '[profiles]' >> \"$TERMINATOR_CONFIG_FILE\"\n"
    script_content += "fi\n"
    script_content += "# Append profiles to config file\n"
    script_content += "echo \"$PROFILES_SECTION\" >> \"$TERMINATOR_CONFIG_FILE\"\n"
    script_content += "echo \"Profiles added to config\"\n\n"
    
    script_content += "# Check if [layouts] section exists\n"
    script_content += "if ! grep -q '^\\[layouts\\]' \"$TERMINATOR_CONFIG_FILE\" 2>/dev/null; then\n"
    script_content += "  # Add [layouts] section if it doesn't exist\n"
    script_content += "  echo '' >> \"$TERMINATOR_CONFIG_FILE\"\n"
    script_content += "  echo '[layouts]' >> \"$TERMINATOR_CONFIG_FILE\"\n"
    script_content += "fi\n"
    script_content += "# Append layout to config file\n"
    script_content += "echo \"$LAYOUT_SECTION\" >> \"$TERMINATOR_CONFIG_FILE\"\n"
    script_content += "echo \"Layout added to config: $TERMINATOR_CONFIG_FILE\"\n\n"
    
    # Launch terminator with the layout
    # Terminator uses --layout (or -l) with the layout name
    script_content += "TERMINATOR_PID=\"\"\n"
    script_content += "terminator --layout=\"$LAYOUT_NAME\" &\n"
    script_content += "TERMINATOR_PID=$!\n"
    script_content += "sleep 2\n"
    script_content += "echo \"Terminator launched with PID: $TERMINATOR_PID\"\n\n"
    
    script_content += "cleanup_and_exit() {\n"
    script_content += "  if [ -n \"$TERMINATOR_PID\" ] && kill -0 \"$TERMINATOR_PID\" 2>/dev/null; then\n"
    script_content += "    kill -TERM \"$TERMINATOR_PID\" 2>/dev/null || true\n"
    script_content += "    sleep 1\n"
    script_content += "    kill -KILL \"$TERMINATOR_PID\" 2>/dev/null || true\n"
    script_content += "  fi\n"
    script_content += "  # Restore config backup if it exists\n"
    script_content += "  if [ -f \"$TERMINATOR_CONFIG_FILE.bak.$$\" ]; then\n"
    script_content += "    mv \"$TERMINATOR_CONFIG_FILE.bak.$$\" \"$TERMINATOR_CONFIG_FILE\"\n"
    script_content += "    echo \"Config restored from backup\"\n"
    script_content += "  fi\n"
    script_content += "  exit 0\n"
    script_content += "}\n"
    script_content += "trap cleanup_and_exit SIGINT SIGTERM\n"
    script_content += "while kill -0 \"$TERMINATOR_PID\" 2>/dev/null; do sleep 1; done\n"
    script_content += "cleanup_and_exit\n"
    
    # Write script to temporary file
    fd, script_path = tempfile.mkstemp(suffix='.sh', prefix='terminator_launcher_')
    with os.fdopen(fd, 'w') as f:
        f.write(script_content)
    os.chmod(script_path, 0o755)
    
    return script_path


def launch_in_terminator(context, launch_arguments_names: list[str], launcher_paths: list[str],
                         launcher_pkg_install_dir: str,
                         launcher_pkg_name: str = 'tier4_perception_launch',
                         layout_name: str = 'ros2_launchers',
                         launch_pointcloud_container: bool = False) -> list[ExecuteProcess]:
    """Launch multiple launchers in terminator with split panes using a layout file."""
    launch_args_cmd = format_launch_args_for_command(context, launch_arguments_names)
    script_path = create_terminator_launcher_script(
        launcher_paths, launch_args_cmd, launcher_pkg_install_dir,
        launcher_pkg_name, layout_name, launch_pointcloud_container
    )
    return [ExecuteProcess(cmd=['bash', script_path], output='screen')]


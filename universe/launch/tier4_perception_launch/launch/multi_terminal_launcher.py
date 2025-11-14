import os
import shutil
import shlex
import tempfile
import logging
import uuid

from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

logger = logging.getLogger(__name__)


def detect_terminal_method() -> str:
    """Detect available terminal method for launching separate terminals."""
    return 'terminator' if shutil.which('terminator') else 'tmux' if shutil.which('tmux') else 'none'


def detect_gui_terminal() -> str:
    """Detect available GUI terminal emulator."""
    if os.environ.get('DISPLAY') is None:
        return ''
    terminals = ['gnome-terminal', 'xterm', 'konsole', 'xfce4-terminal', 'mate-terminal', 'terminator']
    return next((t for t in terminals if shutil.which(t)), '')


def format_launch_args_for_command(context, launch_arguments_names: list[str]) -> list[str]:
    """Convert launch arguments to command-line format."""
    return [f"{name}:={LaunchConfiguration(name).perform(context)}" 
            for name in launch_arguments_names 
            if LaunchConfiguration(name).perform(context) is not None]


def escape_dollar_signs_for_bash(cmd_str: str) -> str:
    """Escape dollar signs to preserve ROS2 launch substitutions."""
    return cmd_str.replace("$", "\\$")


def _get_workspace_setup_path(launcher_pkg_install_dir: str) -> str:
    """Get workspace setup.bash path."""
    workspace_install_dir = os.path.dirname(os.path.dirname(os.path.dirname(launcher_pkg_install_dir)))
    return os.path.join(workspace_install_dir, 'setup.bash')


def _build_command_string(cmd_parts: list[str], escape_dollars: bool = True) -> str:
    """Build quoted command string from parts."""
    cmd = ' '.join(shlex.quote(p) for p in cmd_parts)
    return escape_dollar_signs_for_bash(cmd) if escape_dollars else cmd


def _write_temp_script(content: str, prefix: str, suffix: str = '.sh') -> str:
    """Write script content to temporary file and return path."""
    fd, script_path = tempfile.mkstemp(suffix=suffix, prefix=prefix)
    with os.fdopen(fd, 'w') as f:
        f.write(content)
    os.chmod(script_path, 0o755)
    return script_path


def _generate_terminator_layout(commands: list[tuple[str, str]], layout_name: str, 
                                script_paths: list[str] = None) -> str:
    """Generate terminator layout config for given commands."""
    config_lines = [
        "[global_config]",
        "  suppress_multiple_term_dialog = True", "",
        "[keybindings]", "",
        "[profiles]",
        "  [[default]]", ""
    ]
    
    # Add profiles
    for i, _ in enumerate(commands, 1):
        profile_name = f"{layout_name}_profile_{i}"
        script_path = script_paths[i-1] if script_paths and i <= len(script_paths) else f'placeholder_{i}'
        config_lines.extend([
            f"  [[{profile_name}]]",
            "    use_custom_command = True",
            f"    custom_command = {shlex.quote(script_path)}", ""
        ])
    
    # Add layout
    config_lines.extend([
        "[layouts]",
        f"  [[{layout_name}]]",
        "    [[[window0]]]",
        "      type = Window",
        '      parent = ""'
    ])
    
    num_terminals = len(commands)
    if num_terminals == 0:
        config_lines.extend([
            "    [[[child1]]]",
            "      type = Terminal",
            "      parent = window0",
            "      profile = default"
        ])
    elif num_terminals == 1:
        title, _ = commands[0]
        config_lines.extend([
            "    [[[child1]]]",
            "      type = Terminal",
            "      parent = window0",
            f"      profile = {layout_name}_profile_1",
            f"      title = {title}"
        ])
    elif num_terminals == 2:
        config_lines.append("    [[[child1]]]")
        config_lines.append("      type = VPaned")
        config_lines.append("      parent = window0")
        for i, (title, _) in enumerate(commands):
            config_lines.extend([
                f"    [[[[child{i+2}]]]]",
                "      type = Terminal",
                "      parent = child1",
                f"      profile = {layout_name}_profile_{i+1}",
                f"      title = {title}",
                f"      order = {i}"
            ])
    else:
        # General case: nested VPaned structure
        child_counter = 1
        parent_pane = None
        
        for i, (title, _) in enumerate(commands):
            profile_name = f"{layout_name}_profile_{i+1}"
            is_first = (i == 0)
            is_last = (i == num_terminals - 1)
            
            if is_first:
                config_lines.extend([
                    f"    [[[child{child_counter}]]]",
                    "      type = VPaned",
                    "      parent = window0",
                    "      order = 0"
                ])
                parent_pane = f"child{child_counter}"
                child_counter += 1
                config_lines.extend([
                    f"    [[[child{child_counter}]]]",
                    "      type = Terminal",
                    f"      parent = {parent_pane}",
                    f"      profile = {profile_name}",
                    f"      title = {title}",
                    "      order = 0"
                ])
                child_counter += 1
            elif is_last:
                config_lines.extend([
                    f"    [[[child{child_counter}]]]",
                    "      type = Terminal",
                    f"      parent = {parent_pane}",
                    f"      profile = {profile_name}",
                    f"      title = {title}",
                    "      order = 1"
                ])
                child_counter += 1
            else:
                config_lines.extend([
                    f"    [[[child{child_counter}]]]",
                    "      type = VPaned",
                    f"      parent = {parent_pane}",
                    "      order = 1"
                ])
                parent_pane = f"child{child_counter}"
                child_counter += 1
                config_lines.extend([
                    f"    [[[child{child_counter}]]]",
                    "      type = Terminal",
                    f"      parent = {parent_pane}",
                    f"      profile = {profile_name}",
                    f"      title = {title}",
                    "      order = 0"
                ])
                child_counter += 1
    
    return '\n'.join(config_lines)


def create_tmux_launcher_script(launcher_paths: list[str], launch_args_cmd: list[str], 
                                launcher_pkg_install_dir: str, 
                                launcher_pkg_name: str = 'tier4_perception_launch',
                                session_name: str = 'ros2_launchers',
                                window_name: str = 'launchers',
                                launch_pointcloud_container: bool = False) -> str:
    """Create a bash script that launches tmux with split panes and runs launchers."""
    setup_bash_path = _get_workspace_setup_path(launcher_pkg_install_dir)
    total_panes = len(launcher_paths) + (1 if launch_pointcloud_container else 0)
    window_height, window_width = max(80, total_panes * 5), 200
    gui_terminal = detect_gui_terminal()
    
    lines = [
        "#!/bin/bash", "set -e", "",
        f"SESSION_NAME={shlex.quote(session_name)}",
        f"WINDOW_NAME={shlex.quote(window_name)}",
        f"WORKSPACE_SETUP={shlex.quote(setup_bash_path)}", "",
        "tmux has-session -t $SESSION_NAME 2>/dev/null && tmux kill-session -t $SESSION_NAME || true",
        "command -v tmux &> /dev/null || { echo 'ERROR: tmux not found' >&2; exit 1; }", "",
        f"tmux new-session -d -s $SESSION_NAME -n $WINDOW_NAME -x {window_width} -y {window_height} 'bash' || {{ echo 'ERROR: Failed to create tmux session' >&2; exit 1; }}",
        f"tmux resize-window -t $SESSION_NAME:0 -x {window_width} -y {window_height} 2>/dev/null || true",
        "sleep 0.3",
    ]
    
    if gui_terminal:
        term_commands = {
            'gnome-terminal': f"gnome-terminal --geometry {window_width}x{window_height}+10+10 -- bash -c \"tmux attach-session -t $SESSION_NAME || bash\"",
            'xterm': f"xterm -geometry {window_width}x{window_height} -e bash -c \"tmux attach-session -t $SESSION_NAME || bash\"",
            'konsole': f"konsole --geometry {window_width}x{window_height} -e bash -c \"tmux attach-session -t $SESSION_NAME || bash\"",
            'xfce4-terminal': f"xfce4-terminal --geometry {window_width}x{window_height} -e \"bash -c \\\"tmux attach-session -t $SESSION_NAME || bash\\\"\"",
            'mate-terminal': f"mate-terminal --geometry {window_width}x{window_height} -e \"bash -c \\\"tmux attach-session -t $SESSION_NAME || bash\\\"\"",
            'terminator': f"terminator -e \"bash -c \\\"tmux attach-session -t $SESSION_NAME || bash\\\"\""
        }
        lines.extend([f"{term_commands.get(gui_terminal, '')} &", "sleep 1"])
    
    lines.append("LAUNCHERS_SKIPPED=false")
    
    if launch_pointcloud_container:
        cmd_part = _build_command_string(['ros2', 'run', 'rclcpp_components', 'component_container',
                                         '--ros-args', '-r', '__node:=pointcloud_container', '--log-level', 'info'])
        lines.extend([
            f"CONTAINER_CMD_PART={shlex.quote(cmd_part)}",
            "CONTAINER_CMD=\"source $WORKSPACE_SETUP && $CONTAINER_CMD_PART\"",
            "echo \"Launching pointcloud container\"",
            "tmux send-keys -t $SESSION_NAME:0.0 \"$CONTAINER_CMD\" Enter",
            "sleep 1"
        ])
    
    if launcher_paths:
        first_path = launcher_paths[0]
        first_name = os.path.basename(first_path)
        full_path = os.path.join(launcher_pkg_install_dir, first_path)
        cmd_part = _build_command_string(['ros2', 'launch', full_path] + launch_args_cmd)
        
        if not launch_pointcloud_container:
            lines.extend([
                f"LAUNCH_CMD_PART_0={shlex.quote(cmd_part)}",
                "LAUNCH_CMD_0=\"source $WORKSPACE_SETUP && $LAUNCH_CMD_PART_0\"",
                f"echo \"Launching: {first_name}\"",
                "tmux send-keys -t $SESSION_NAME:0.0 \"$LAUNCH_CMD_0\" Enter",
                "sleep 1"
            ])
        else:
            lines.extend([
                "tmux split-window -v -t $SESSION_NAME:0 'bash' || { LAUNCHERS_SKIPPED=true; }",
                "[ \"$LAUNCHERS_SKIPPED\" = \"false\" ] && {",
                f"  LAUNCH_CMD_PART_0={shlex.quote(cmd_part)}",
                "  LAUNCH_CMD_0=\"source $WORKSPACE_SETUP && $LAUNCH_CMD_PART_0\"",
                f"  echo \"Launching: {first_name}\"",
                "  tmux send-keys -t $SESSION_NAME:0. \"$LAUNCH_CMD_0\" Enter",
                "  sleep 1",
                "}"
            ])
    
    initial_panes = 1 + (1 if launch_pointcloud_container and launcher_paths else 0)
    lines.append(f"SUCCESSFUL_PANES=$(({initial_panes}))")
    lines.append("[ \"$LAUNCHERS_SKIPPED\" != \"true\" ] && {")
    
    for i, launcher_path in enumerate(launcher_paths[1:], 1):
        launcher_name = os.path.basename(launcher_path)
        full_path = os.path.join(launcher_pkg_install_dir, launcher_path)
        cmd_part = _build_command_string(['ros2', 'launch', full_path] + launch_args_cmd)
        lines.extend([
            "  tmux split-window -v -t $SESSION_NAME:0 'bash' || break",
            f"  LAUNCH_CMD_PART_{i}={shlex.quote(cmd_part)}",
            f"  LAUNCH_CMD_{i}=\"source $WORKSPACE_SETUP && $LAUNCH_CMD_PART_{i}\"",
            f"  echo \"Launching: {launcher_name}\"",
            f"  tmux send-keys -t $SESSION_NAME:0. \"$LAUNCH_CMD_{i}\" Enter",
            "  SUCCESSFUL_PANES=$((SUCCESSFUL_PANES + 1))",
            "  sleep 1"
        ])
    lines.append("}")
    
    lines.extend([
        "tmux select-layout -t $SESSION_NAME:0 even-vertical",
        f"echo \"Launched $SUCCESSFUL_PANES/{total_panes} panes\"", ""
    ])
    
    if not gui_terminal:
        lines.append("[ -z \"$TMUX\" ] && [ -t 0 ] && [ -t 1 ] && exec tmux attach-session -t $SESSION_NAME 2>/dev/null || true")
    
    lines.extend([
        "cleanup_and_exit() {",
        "  if tmux has-session -t $SESSION_NAME 2>/dev/null; then",
        "    for pane in $(tmux list-panes -t $SESSION_NAME -F '#{window_index}.#{pane_index}' 2>/dev/null); do",
        "      tmux send-keys -t $SESSION_NAME:$pane C-c 2>/dev/null || true",
        "    done",
        "    sleep 2",
        "    tmux kill-session -t $SESSION_NAME 2>/dev/null || true",
        "  fi",
        "  exit 0",
        "}",
        "trap cleanup_and_exit SIGINT SIGTERM",
        "while tmux has-session -t $SESSION_NAME 2>/dev/null; do sleep 1; done"
    ])
    
    return _write_temp_script('\n'.join(lines), 'tmux_launcher_')


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
                                     launch_pointcloud_container: bool = False,
                                     titles: list[str] = None) -> str:
    """Create a bash script that launches terminator with a layout and runs launchers."""
    setup_bash_path = _get_workspace_setup_path(launcher_pkg_install_dir)
    session_group_id = f"autoware-multi-terminal-{uuid.uuid4()}"
    group_env = f"AUTOWARE_MULTI_TERMINAL_GROUP={session_group_id}"
    total_panes = len(launcher_paths) + (1 if launch_pointcloud_container else 0)
    
    # Build commands
    commands = []
    all_titles = []
    
    if launch_pointcloud_container:
        cmd = _build_command_string(['env', group_env, 'ros2', 'run', 'rclcpp_components', 
                                    'component_container', '--ros-args', '-r', '__node:=pointcloud_container', 
                                    '--log-level', 'info'], escape_dollars=False)
        commands.append(('pointcloud_container', cmd))
        all_titles.append('pointcloud_container')
    
    for launcher_path in launcher_paths:
        launcher_name = os.path.basename(launcher_path)
        full_path = os.path.join(launcher_pkg_install_dir, launcher_path)
        escaped_args = [escape_dollar_signs_for_bash(arg) for arg in launch_args_cmd]
        cmd = _build_command_string(['env', group_env, 'ros2', 'launch', full_path] + escaped_args, 
                                   escape_dollars=False)
        commands.append((launcher_name, cmd))
        all_titles.append(launcher_name)
    
    if titles:
        all_titles = titles
    
    # Create command scripts
    command_scripts = []
    for i, (title, cmd) in enumerate(commands, 1):
        pid_fd, pid_file_path = tempfile.mkstemp(suffix='.pid', prefix=f'ros2_cmd_{i}_')
        os.close(pid_fd)
        
        script_content = '\n'.join([
            "#!/bin/bash",
            "set -x",
            f"PID_FILE={shlex.quote(pid_file_path)}",
            "echo $$ > \"$PID_FILE\"",
            f"echo \"Starting launcher: {title} (PID $$)\"",
            f"source {shlex.quote(setup_bash_path)}",
            "",
            f"exec {cmd}"
        ])
        
        script_path = _write_temp_script(script_content, f'ros2_cmd_{i}_')
        command_scripts.append((script_path, pid_file_path))
    
    # Generate terminator config
    script_paths = [path for path, _ in command_scripts]
    config_content = _generate_terminator_layout(commands, layout_name, script_paths)
    
    config_fd, config_path = tempfile.mkstemp(suffix='_terminator_config', prefix='ros2_launcher_')
    with os.fdopen(config_fd, 'w') as f:
        f.write(config_content)
    
    # Build main script
    script_lines = [
        "#!/bin/bash", "set -e", "",
        f"WORKSPACE_SETUP={shlex.quote(setup_bash_path)}",
        f"LAYOUT_NAME={shlex.quote(layout_name)}",
        f"SESSION_GROUP_ID={shlex.quote(session_group_id)}", "",
        "command -v terminator &> /dev/null || { echo 'ERROR: terminator not found' >&2; exit 1; }", "",
        f"echo \"Launching terminator with {total_panes} panes\"", "",
        "echo \"Creating command scripts...\""
    ]
    
    for title, (script_path, pid_file) in zip(all_titles, command_scripts):
        script_lines.append(f"echo \"Created script for {title}: {script_path} (PID file: {pid_file})\"")
    
    script_lines.extend([
        f"TERMINATOR_CONFIG={shlex.quote(config_path)}", "",
        "echo \"About to launch terminator...\"",
        "TERMINATOR_PID=\"\"",
        "terminator --config=\"$TERMINATOR_CONFIG\" --layout=\"$LAYOUT_NAME\" &",
        "TERMINATOR_PID=$!",
        "echo \"Terminator launched with PID: $TERMINATOR_PID\"",
        "sleep 2",
        "if kill -0 \"$TERMINATOR_PID\" 2>/dev/null; then",
        "  echo \"Terminator is still running with PID: $TERMINATOR_PID\"",
        "else",
        "  echo \"ERROR: Terminator exited immediately (PID: $TERMINATOR_PID)\"",
        "  exit 1",
        "fi", "",
        f"COMMAND_SCRIPTS=\"{' '.join(shlex.quote(p) for p, _ in command_scripts)}\"",
        f"COMMAND_PID_FILES=\"{' '.join(shlex.quote(p) for _, p in command_scripts)}\"", "",
        "cleanup_and_exit() {",
        "  echo \"Shutting down all processes...\"",
        "  if [ -n \"$TERMINATOR_PID\" ] && kill -0 \"$TERMINATOR_PID\" 2>/dev/null; then",
        "    echo \"Terminating terminator (PID: $TERMINATOR_PID)...\"",
        "    kill -TERM \"$TERMINATOR_PID\" 2>/dev/null || true",
        "    kill -TERM -\"$TERMINATOR_PID\" 2>/dev/null || true",
        "    sleep 1",
        "    if kill -0 \"$TERMINATOR_PID\" 2>/dev/null; then",
        "      kill -KILL \"$TERMINATOR_PID\" 2>/dev/null || true",
        "      kill -KILL -\"$TERMINATOR_PID\" 2>/dev/null || true",
        "    fi",
        "  fi",
        "  if [ -n \"$COMMAND_PID_FILES\" ]; then",
        "    for pid_file in $COMMAND_PID_FILES; do",
        "      if [ -f \"$pid_file\" ]; then",
        "        child_pid=$(cat \"$pid_file\" 2>/dev/null)",
        "        if [ -n \"$child_pid\" ]; then",
        "          for sig in SIGINT SIGTERM SIGKILL; do",
        "            if kill -0 \"$child_pid\" 2>/dev/null; then",
        "              echo \"Sending $sig to PID $child_pid\"",
        "              kill -$sig \"$child_pid\" 2>/dev/null || true",
        "              [ \"$sig\" != \"SIGKILL\" ] && sleep 1",
        "            fi",
        "          done",
        "        fi",
        "        rm -f \"$pid_file\" 2>/dev/null || true",
        "      fi",
        "    done",
        "  fi",
        "  rm -f \"$TERMINATOR_CONFIG\" 2>/dev/null || true",
        "  for script in $COMMAND_SCRIPTS; do",
        "    rm -f \"$script\" 2>/dev/null || true",
        "  done",
        "  echo \"Cleanup complete.\"",
        "  exit 0",
        "}",
        "trap cleanup_and_exit SIGINT SIGTERM",
        "echo \"Launcher script is running. Press Ctrl+C to exit all sessions.\"",
        "while kill -0 \"$TERMINATOR_PID\" 2>/dev/null; do",
        "  sleep 2",
        "done",
        "echo \"Terminator exited. Cleaning up...\"",
        "cleanup_and_exit"
    ])
    
    return _write_temp_script('\n'.join(script_lines), 'terminator_launcher_')


def launch_in_terminator(context, launch_arguments_names: list[str], launcher_paths: list[str],
                         launcher_pkg_install_dir: str,
                         launcher_pkg_name: str = 'tier4_perception_launch',
                         layout_name: str = 'ros2_launchers',
                         launch_pointcloud_container: bool = False,
                         titles: list[str] = None) -> list[ExecuteProcess]:
    """Launch multiple launchers in terminator with split panes using a layout file."""
    launch_args_cmd = format_launch_args_for_command(context, launch_arguments_names)
    script_path = create_terminator_launcher_script(
        launcher_paths, launch_args_cmd, launcher_pkg_install_dir,
        launcher_pkg_name, layout_name, launch_pointcloud_container, titles
    )
    return [ExecuteProcess(cmd=['bash', script_path], output='screen')]


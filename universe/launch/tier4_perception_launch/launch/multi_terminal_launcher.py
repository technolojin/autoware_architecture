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
    
    def build_cmd_part_for_terminator(cmd_parts):
        """Build command part for terminator (no dollar escaping needed with single quotes)."""
        cmd_quoted = ' '.join(shlex.quote(p) for p in cmd_parts)
        return cmd_quoted  # No escape_dollar_signs_for_bash since we'll use single quotes
    
    if launch_pointcloud_container:
        container_cmd_part = build_cmd_part_for_terminator(['ros2', 'run', 'rclcpp_components', 'component_container',
                                            '--ros-args', '-r', '__node:=pointcloud_container', '--log-level', 'info'])
        commands.append(('pointcloud_container', container_cmd_part))
    
    for launcher_path in launcher_paths:
        launcher_name = os.path.basename(launcher_path)
        full_launcher_path = os.path.join(launcher_pkg_install_dir, launcher_path)
        launch_cmd_part = build_cmd_part_for_terminator(['ros2', 'launch', full_launcher_path] + launch_args_cmd)
        commands.append((launcher_name, launch_cmd_part))
    
    # Create a standalone terminator config file for this session
    # This avoids conflicts with the user's existing config
    config_fd, config_path = tempfile.mkstemp(suffix='_terminator_config', prefix='ros2_launcher_')
    
    # Build terminator config content manually (terminator uses custom INI format)
    config_content = "[global_config]\n"
    config_content += "  suppress_multiple_term_dialog = True\n\n"
    
    config_content += "[keybindings]\n\n"
    
    # Add profiles section with our custom profiles
    config_content += "[profiles]\n"
    config_content += "  [[default]]\n\n"
    
    for i, (title, cmd) in enumerate(commands, 1):
        profile_name = f"{layout_name}_profile_{i}"
        # Escape single quotes in the command
        cmd_for_single_quote = cmd.replace("'", "'\\''")
        # Build the bash command that sources setup and runs the ros2 command
        setup_bash_quoted = shlex.quote(setup_bash_path)
        full_cmd = f"source {setup_bash_quoted} && {cmd}"
        bash_cmd = f"bash -c {shlex.quote(full_cmd)}; exec bash"

        config_content += f"  [[{profile_name}]]\n"
        config_content += f"    use_custom_command = True\n"
        config_content += f"    custom_command = {shlex.quote(bash_cmd)}\n\n"
    
    # Add layouts section
    config_content += "[layouts]\n"
    config_content += f"  [[{layout_name}]]\n"
    config_content += "    [[[window0]]]\n"
    config_content += "      type = Window\n"
    config_content += '      parent = ""\n'
    
    if len(commands) == 0:
        # Empty layout
        config_content += "    [[[child1]]]\n"
        config_content += "      type = Terminal\n"
        config_content += "      parent = window0\n"
        config_content += "      profile = default\n"
    elif len(commands) == 1:
        # Single terminal
        title, _ = commands[0]
        profile_name = f"{layout_name}_profile_1"
        config_content += "    [[[child1]]]\n"
        config_content += "      type = Terminal\n"
        config_content += "      parent = window0\n"
        config_content += f"      profile = {profile_name}\n"
        config_content += f"      title = {title}\n"
    elif len(commands) == 2:
        # Two terminals - simple VPaned split
        config_content += "    [[[child1]]]\n"
        config_content += "      type = VPaned\n"
        config_content += "      parent = window0\n"
        
        for i, (title, _) in enumerate(commands):
            profile_name = f"{layout_name}_profile_{i+1}"
            config_content += f"    [[[[child{i+2}]]]]\n"
            config_content += "      type = Terminal\n"
            config_content += "      parent = child1\n"
            config_content += f"      profile = {profile_name}\n"
            config_content += f"      title = {title}\n"
            config_content += f"      order = {i}\n"
    else:
        # Multiple terminals - create nested VPaned structures
        # Structure: VPaned(Terminal1, VPaned(Terminal2, VPaned(Terminal3, ...)))
        child_counter = 1

        for i, (title, _) in enumerate(commands):
            profile_name = f"{layout_name}_profile_{i+1}"
            is_first = (i == 0)
            is_last = (i == len(commands) - 1)

            if is_first:
                # Root VPaned
                config_content += f"    [[[child{child_counter}]]]\n"
                config_content += "      type = VPaned\n"
                config_content += "      parent = window0\n"
                config_content += "      order = 0\n"
                parent_pane = f"child{child_counter}"
                child_counter += 1

                # First terminal
                config_content += f"    [[[child{child_counter}]]]\n"
                config_content += "      type = Terminal\n"
                config_content += f"      parent = {parent_pane}\n"
                config_content += f"      profile = {profile_name}\n"
                config_content += f"      title = {title}\n"
                config_content += "      order = 0\n"
                child_counter += 1
            elif is_last:
                # Last terminal (order=1 child of previous VPaned)
                config_content += f"    [[[child{child_counter}]]]\n"
                config_content += "      type = Terminal\n"
                config_content += f"      parent = {parent_pane}\n"
                config_content += f"      profile = {profile_name}\n"
                config_content += f"      title = {title}\n"
                config_content += "      order = 1\n"
                child_counter += 1
            else:
                # Nested VPaned (order=1 child of previous VPaned)
                config_content += f"    [[[child{child_counter}]]]\n"
                config_content += "      type = VPaned\n"
                config_content += f"      parent = {parent_pane}\n"
                config_content += "      order = 1\n"
                parent_pane = f"child{child_counter}"
                child_counter += 1

                # Terminal in this VPaned
                config_content += f"    [[[child{child_counter}]]]\n"
                config_content += "      type = Terminal\n"
                config_content += f"      parent = {parent_pane}\n"
                config_content += f"      profile = {profile_name}\n"
                config_content += f"      title = {title}\n"
                config_content += "      order = 0\n"
                child_counter += 1
    
    # Write config to temp file
    with os.fdopen(config_fd, 'w') as f:
        f.write(config_content)
    
    config_path_quoted = shlex.quote(config_path)
    
    # Add commands to use this config file
    script_content += f"TERMINATOR_CONFIG={config_path_quoted}\n"
    script_content += "echo \"Using terminator config: $TERMINATOR_CONFIG\"\n\n"
    
    # Launch terminator with the custom config and layout
    script_content += "TERMINATOR_PID=\"\"\n"
    script_content += "terminator --config=\"$TERMINATOR_CONFIG\" --layout=\"$LAYOUT_NAME\" &\n"
    script_content += "TERMINATOR_PID=$!\n"
    script_content += "sleep 2\n"
    script_content += "echo \"Terminator launched with PID: $TERMINATOR_PID\"\n\n"

    script_content += "cleanup_and_exit() {\n"
    script_content += "  if [ -n \"$TERMINATOR_PID\" ] && kill -0 \"$TERMINATOR_PID\" 2>/dev/null; then\n"
    script_content += "    kill -TERM \"$TERMINATOR_PID\" 2>/dev/null || true\n"
    script_content += "    sleep 1\n"
    script_content += "    kill -KILL \"$TERMINATOR_PID\" 2>/dev/null || true\n"
    script_content += "  fi\n"
    script_content += "  # Clean up temp files\n"
    script_content += "  rm -f \"$TERMINATOR_CONFIG\" 2>/dev/null || true\n"
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


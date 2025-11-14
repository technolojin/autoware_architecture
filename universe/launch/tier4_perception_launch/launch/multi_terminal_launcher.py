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
    return 'tmux' if shutil.which('tmux') is not None else 'none'


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


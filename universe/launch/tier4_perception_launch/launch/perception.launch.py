from launch import LaunchDescription
from launch.actions import ExecuteProcess

from autoware_architect.deployment import Deployment
from autoware_architect.config import ArchitectureConfig


def generate_autoware_architecture():
    """Generate Autoware Architecture deployment."""
    # Load architecture configuration from YAML file


    architecture_config = ArchitectureConfig()
    architecture_config.debug_mode = True
    architecture_config.log_level = "INFO"

    architecture_config.deployment_file = "deployment/deployment.yaml"
    architecture_config.architecture_yaml_list_file = "config/architecture/architecture.yaml"
    architecture_config.output_root_dir = "build/autoware_architect/output"

    logger = architecture_config.set_logging()

    # load and build the deployment
    logger.info("autoware architect: Building deployment...")
    deployment = Deployment(architecture_config)

    # generate the system visualization
    logger.info("autoware architect: Generating visualization...")
    deployment.visualize()

    # generate the launch files
    logger.info("autoware architect: Generating launch files...")
    deployment.generate_launcher()

    return

def launch_generated_launch_file():
    """Launch the generated Autoware Architecture launch file."""
    # Here we would normally load and execute the generated launch file.
    # For simplicity, we will just print a message.
    print("Launching generated Autoware Architecture launch file...")
    return ExecuteProcess(
        cmd=['ros2', 'launch', 'autoware_generated_launch', 'generated_launch.launch.py'],
        output='screen',
    )

def generate_launch_description():
    """Launch file that says hello world."""
    
    return LaunchDescription(
        [generate_autoware_architecture()]
        + [launch_generated_launch_file()]
    )

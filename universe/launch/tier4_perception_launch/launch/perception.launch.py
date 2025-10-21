import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from autoware_architect.deployment import Deployment
from autoware_architect.config import ArchitectureConfig

def generate_autoware_architecture():
    """Generate Autoware Architecture deployment."""
    # Load architecture configuration from YAML file


    architecture_config = ArchitectureConfig()
    architecture_config.debug_mode = True
    architecture_config.log_level = "INFO"

    workspace_root = os.path.dirname(get_package_prefix('autoware_architect')).rsplit('/', 1)[0]
    architect_pkg_build_dir = os.path.join(workspace_root, 'build', 'autoware_architect')
    launcher_pkg_install_dir = get_package_share_directory('tier4_perception_launch')

    architecture_config.architecture_yaml_list_file = os.path.join(architect_pkg_build_dir, "autoware_architect_config_file_list.txt")
    architecture_config.deployment_file = os.path.join(launcher_pkg_install_dir, "deployment/universe_perception.deployment.yaml")
    architecture_config.output_root_dir = os.path.join(launcher_pkg_install_dir)

    print("Architecture YAML List File:", architecture_config.architecture_yaml_list_file)
    print("Deployment File:", architecture_config.deployment_file)
    print("Output Root Dir:", architecture_config.output_root_dir)

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

    launcher_pkg_install_dir = get_package_share_directory('tier4_perception_launch')
    launcher_file = os.path.join(launcher_pkg_install_dir, "exports/universe_perception.deployment/launcher/default/main_ecu/perception/perception.launch.xml")

    print("Including launcher file:", launcher_file)

    return IncludeLaunchDescription(
        XMLLaunchDescriptionSource(launcher_file)
    )

def generate_launch_description():
    """Launch file that says hello world."""
    # First generate the architecture (this is just for setup, returns None)
    generate_autoware_architecture()
    
    # Then launch the generated launch file
    return LaunchDescription(
        [launch_generated_launch_file()]
    )

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from autoware_architect.deployment import Deployment
from autoware_architect.config import ArchitectureConfig









def generate_autoware_architecture(deployment_file: str):
    """Generate Autoware Architecture deployment."""
    # Load architecture configuration from YAML file

    architecture_config = ArchitectureConfig()
    architecture_config.debug_mode = True
    architecture_config.log_level = "INFO"

    logger = architecture_config.set_logging()

    workspace_root = os.path.dirname(get_package_prefix('autoware_architect')).rsplit('/', 1)[0]
    architect_pkg_build_dir = os.path.join(workspace_root, 'build', 'autoware_architect')
    launcher_pkg_install_dir = get_package_share_directory('tier4_perception_launch')

    architecture_config.architecture_yaml_list_file = os.path.join(architect_pkg_build_dir, "autoware_architect_config_file_list.txt")
    architecture_config.deployment_file = os.path.join(launcher_pkg_install_dir, deployment_file)
    architecture_config.output_root_dir = os.path.join(launcher_pkg_install_dir)

    logger.info("Architecture YAML List File: %s", architecture_config.architecture_yaml_list_file)
    logger.info("Deployment File: %s", architecture_config.deployment_file)
    logger.info("Output Root Dir: %s", architecture_config.output_root_dir)

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

def create_pointcloud_container():
    """Create the pointcloud_container composable node container."""
    return ComposableNodeContainer(
        name="pointcloud_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
    )

def launch_generated_launch_file(launch_arguments_names, launcher_file: str):
    """Launch the generated Autoware Architecture launch file."""
    # Here we would normally load and execute the generated launch file.
    # For simplicity, we will just print a message.
    print("Launching generated Autoware Architecture launch file...")

    # Build launch arguments dictionary using LaunchConfiguration for each argument
    launch_args_dict = {name: LaunchConfiguration(name) for name in launch_arguments_names}

    return IncludeLaunchDescription(
        XMLLaunchDescriptionSource(launcher_file),
        launch_arguments=launch_args_dict.items()
    )

def generate_launch_description():
    """ Generate autoware architecture and launch the generated launch file. """

    # set launch arguments 
    launch_arguments = []
    launch_argument_names = []
    def add_launch_arg(name: str, default_value=None, **kwargs):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value, **kwargs))
        launch_argument_names.append(name)

    # for perception ecu
    add_launch_arg("camera_2d_detector/model_path", default_value="$(var data_path)/tensorrt_yolox/yolox-sPlus-opt-pseudoV2-T4-960x960-T4-seg16cls.onnx")
    add_launch_arg("camera_2d_detector/label_path", default_value="$(var data_path)/tensorrt_yolox/label.txt")
    add_launch_arg("camera_2d_detector/color_map_path", default_value="$(var data_path)/tensorrt_yolox/semseg_color_map.csv")

    # deployment configuration
    add_launch_arg("data_path", default_value="$(env HOME)/autoware_data") # config
    add_launch_arg("config_path", default_value="install/autoware_configs/share/autoware_configs/config/default") # config
    add_launch_arg("vehicle_param_file", default_value="$(find-pkg-share $(var vehicle_model)_description)/config/vehicle_info.param.yaml") # config
    
    add_launch_arg("pointcloud_container_name", default_value="pointcloud_container")



    # 1. pipeline junctions: switches to change SW architecture 

    # Simulation / Evaluation
    add_launch_arg("use_empty_dynamic_object_publisher", default_value="false")
    add_launch_arg("use_perception_online_evaluator", default_value="false")
    add_launch_arg("use_perception_analytics_publisher", default_value="true")

    # Common
    add_launch_arg("mode", default_value="camera_lidar_fusion")

    # Object recognition
    add_launch_arg("use_multi_channel_tracker_merger", default_value="true") # merger and tracker
    add_launch_arg("use_irregular_object_detector", default_value="true") # detector on/off, also for merger
    add_launch_arg("use_detection_by_tracker", default_value="true") # detector on/off, also for merger
    add_launch_arg("lidar_detection_model", default_value="centerpoint/centerpoint_tiny") 
    add_launch_arg("lidar_detection_model_type", default_value="centerpoint") # ml model (mode) node difference, also for merger
    add_launch_arg("lidar_detection_model_name", default_value="centerpoint_tiny") # ml model, same node

    # Object recognition / detection / detector
    add_launch_arg("use_low_height_cropbox", default_value="false") # filter on/off of euclidean clustering

    # Object recognition / detection / detector / camera_lidar
    add_launch_arg("use_low_intensity_cluster_filter", default_value="true") # filter on/off
    add_launch_arg("use_image_segmentation_based_filter", default_value="false") # filter on/off
    add_launch_arg("segmentation_pointcloud_fusion_camera_ids", default_value="[0,2,4]") # sensor set, only if use_image_segmentation_based_filter

    # Object recognition / detection / filter
    add_launch_arg("use_object_filter", default_value="true") # filter and merger, almost true
    add_launch_arg("objects_filter_method", default_value="lanelet_filter") # objects filter mode, lanelet_filter or position_filter
    add_launch_arg("objects_validation_method", default_value="obstacle_pointcloud") # mode, obstacle_pointcloud or occupancy_grid
    add_launch_arg("use_pointcloud_map", default_value="true") # pointcloud map filter mode

    # Object recognition / detection / merger
    add_launch_arg("ml_camera_lidar_merger_priority_mode", default_value="0") # dynamic configuration, only if !use_multi_channel_tracker_merger

    # Object recognition / tracking
    add_launch_arg("use_radar_tracking_fusion", default_value="true") # merge topology change, only if !use_multi_channel_tracker_merger and radar is used
    add_launch_arg("tracker_publish_merged_objects", default_value="false") # dynamic configuration, only if use_multi_channel_tracker_merger


    # Object recognition / prediction
    add_launch_arg("use_vector_map", default_value="true") # always true

    # Obstacle segmentation
    add_launch_arg("use_obstacle_segmentation_single_frame_filter", default_value="false") # ground segmentation filter mode
    add_launch_arg("use_obstacle_segmentation_time_series_filter", default_value="true") # ground segmentation filter mode

    # Occupancy grid map
    add_launch_arg("occupancy_grid_map_method", default_value="pointcloud_based") # occupancy grid map mode
    add_launch_arg("occupancy_grid_map_updater", default_value="binary_bayes_filter") # always binary_bayes_filter

    # traffic light
    add_launch_arg("use_traffic_light_recognition", default_value="true") # module switch. need for simulation
    add_launch_arg("traffic_light_recognition/fusion_only", default_value="false") # linked with ecu
    add_launch_arg("traffic_light_recognition/camera_namespaces", default_value="[camera6, camera7]") # sensor set
    add_launch_arg("traffic_light_recognition/use_high_accuracy_detection", default_value="true") # filter on/off
    add_launch_arg("traffic_light_recognition/high_accuracy_detection_type", default_value="fine_detection") # filter mode, whole_image_detection or fine_detection


    # 2. parameter set arguments: get parameter file directories




    # determine the architecture to generate
    # if the combination of pipeline junctions does not support any architecture, raise an error



    deployment_file = "deployment/universe_perception.deployment.yaml"

    # First generate the architecture (this is just for setup, returns None)
    generate_autoware_architecture(deployment_file)
    launcher_pkg_install_dir = get_package_share_directory('tier4_perception_launch')
    launcher_path = "exports/universe_perception.deployment/launcher/default/main_ecu/perception/perception.launch.xml"
    launcher_file = os.path.join(launcher_pkg_install_dir, launcher_path)

    # Create the pointcloud container
    pointcloud_container = create_pointcloud_container()

    # Then launch the generated launch file
    return LaunchDescription(
        launch_arguments + [pointcloud_container, launch_generated_launch_file(launch_argument_names, launcher_file)]
    )

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from autoware_architect.deployment import Deployment
from autoware_architect.config import ArchitectureConfig
import pprint


def config_to_bool(config_name: str, context) -> bool:
    """Convert LaunchConfiguration to boolean."""
    value = LaunchConfiguration(config_name).perform(context)
    return value == "True" or value == "true"

def config_to_str(config_name: str, context) -> str:
    """Convert LaunchConfiguration to string."""
    value = LaunchConfiguration(config_name).perform(context)
    return str(value)

def determine_mode(context) -> dict:
    """Determine mode based on launch configurations."""
    modes = {}
    parameters = {}
    
    # common
    if config_to_bool("downsample_perception_common_pointcloud", context):
        modes["downsample_perception_common_pointcloud"] = "on"
    else:
        modes["downsample_perception_common_pointcloud"] = "off"

    if config_to_bool("use_perception_online_evaluator", context):
        modes["perception_online_evaluator"] = "on"
    else:
        modes["perception_online_evaluator"] = "off"

    if config_to_bool("use_perception_analytics_publisher", context):
        modes["perception_analytics_publisher"] = "on"
    else:
        modes["perception_analytics_publisher"] = "off"

    # obstacle segmentation
    if config_to_bool("use_obstacle_segmentation_single_frame_filter", context):
        if config_to_bool("use_obstacle_segmentation_time_series_filter", context):
            modes["obstacle_segmentation_filter"] = "combined"
        else:
            modes["obstacle_segmentation_filter"] = "single_frame"
    else:
        if config_to_bool("use_obstacle_segmentation_time_series_filter", context):
            modes["obstacle_segmentation_filter"] = "time_series"
        else:
            modes["obstacle_segmentation_filter"] = "none"

    # object recognition
    modality = config_to_str("mode", context)
    is_lidar_used = False
    is_lidar_camera_fusion = False

    ## multi modality
    if config_to_bool("use_empty_dynamic_object_publisher", context):
        modes["object_recognition"] = {"mode":"dummy"}
        modes["perception_online_evaluator"] = "off"
        modes["perception_analytics_publisher"] = "off"
    elif modality == "camera_lidar_radar_fusion":
        modes["object_recognition"] = {"mode":"camera_lidar_radar_fusion"}
        is_lidar_used = True
        is_lidar_camera_fusion = True
    elif modality == "camera_lidar_fusion":
        modes["object_recognition"] = {"mode":"camera_lidar_fusion"}
        modes["irregular_object_detector"] = "on" if config_to_bool("use_irregular_object_detector", context) else "off"
        is_lidar_used = True
        is_lidar_camera_fusion = True
    elif modality == "lidar_radar_fusion":
        modes["object_recognition"] = {"mode":"lidar_radar_fusion"}
        is_lidar_used = True
    ## single modality
    elif modality == "lidar":
        modes["object_recognition"] = {"mode":"lidar"}
        is_lidar_used = True
    elif modality == "radar":
        modes["object_recognition"] = {"mode":"radar"}
    elif modality == "camera":
        modes["object_recognition"] = {"mode":"camera"}
    else:
        return KeyError(f"Invalid mode: {modality}")
    
    ## detector additional modes
    if is_lidar_used:
        modes["object_recognition"]["detection_by_tracker"] = "on" if config_to_bool("use_detection_by_tracker", context) else "off"
        modes["object_recognition"]["filter__low_height_cropbox"] = "on" if config_to_bool("use_low_height_cropbox", context) else "off"

        # object filter
        if config_to_bool("use_object_filter", context):
            if config_to_str("objects_filter_method", context) == "lanelet_filter":
                modes["object_recognition"]["filter__object_filter"] = "lanelet_filter"
            elif config_to_str("objects_filter_method", context) == "position_filter":
                modes["object_recognition"]["filter__object_filter"] = "position_filter"
            else:
                return KeyError(f"Invalid objects filter method: {config_to_str('objects_filter_method', context)}")
        else:
            modes["object_recognition"]["filter__object_filter"] = "off"

        # object validator
        if config_to_bool("use_object_validator", context):
            if config_to_str("objects_validation_method", context) == "obstacle_pointcloud":
                if config_to_bool("use_pointcloud_map", context):
                    modes["object_recognition"]["filter__object_validator"] = "map_filtered_obstacle_pointcloud"
                else:
                    modes["object_recognition"]["filter__object_validator"] = "obstacle_pointcloud"
            elif config_to_str("objects_validation_method", context) == "occupancy_grid":
                modes["object_recognition"]["filter__object_validator"] = "occupancy_grid"
            else:
                return KeyError(f"Invalid objects validation method: {config_to_str('objects_validation_method', context)}")

        # lidar detection model
        lidar_detection_model_code = LaunchConfiguration("lidar_detection_model").perform(context)
        lidar_detection_model_type = lidar_detection_model_code.split('/')[0]
        lidar_detection_model_name = lidar_detection_model_code.split('/')[1] if '/' in lidar_detection_model_code else ""
        if lidar_detection_model_type == "centerpoint":
            modes["object_recognition"]["lidar_detection_model"] = {"type": "centerpoint"}
            if lidar_detection_model_name in ["centerpoint", "centerpoint_tiny", "centerpoint_sigma"]:
                modes["object_recognition"]["lidar_detection_model"]["name"] = lidar_detection_model_name
            elif lidar_detection_model_name == "":
                modes["object_recognition"]["lidar_detection_model"]["name"] = "centerpoint_tiny"
            else:
                return KeyError(f"Invalid centerpoint model name: {lidar_detection_model_name}")
        elif lidar_detection_model_type == "bevfusion":
            modes["object_recognition"]["lidar_detection_model"] = {"type": "bevfusion", "name": "bevfusion_lidar"}
        elif lidar_detection_model_type == "pointpainting":
            modes["object_recognition"]["lidar_detection_model"] = {"type": "pointpainting", "name": "pointpainting"}
        elif lidar_detection_model_type == "transfusion":
            modes["object_recognition"]["lidar_detection_model"] = {"type": "transfusion", "name": "transfusion"}
        elif lidar_detection_model_type == "apollo":
            modes["object_recognition"]["lidar_detection_model"] = {"type": "apollo"}
        elif lidar_detection_model_type == "clustering":
            modes["object_recognition"]["lidar_detection_model"] = {"type": "clustering"}
        else:
            return KeyError(f"Invalid lidar detection model type: {lidar_detection_model_type}")

    if is_lidar_camera_fusion:
        modes["object_recognition"]["irregular_object_detector"] = "on" if config_to_bool("use_irregular_object_detector", context) else "off"
        modes["object_recognition"]["image_segmentation_based_filter"] = "on" if config_to_bool("use_image_segmentation_based_filter", context) else "off"
        modes["object_recognition"]["low_intensity_cluster_filter"] = "on" if config_to_bool("use_low_intensity_cluster_filter", context) else "off"
        parameters["segmentation_pointcloud_fusion_camera_ids"] = config_to_str("segmentation_pointcloud_fusion_camera_ids", context)
        parameters["ml_camera_lidar_merger_priority_mode"] = int(LaunchConfiguration("ml_camera_lidar_merger_priority_mode").perform(context))


    ## detector

    ## tracker merger
    if config_to_bool("use_multi_channel_tracker_merger", context):
        modes["tracker"] = "multi_channel_tracker_merger"
    else:
        modes["tracker"] = "single_channel_tracker"
        if modality == "camera_lidar_radar_fusion":
            if config_to_bool("use_radar_tracking_fusion", context):
                modes["object_recognition"]["merger"] = {"type": "camera_lidar_merger"}
                modes["tracker"] = "tracker_merger_radar_fusion"
            else:
                modes["object_recognition"]["merger"] = {"type": "camera_lidar_radar_merger"}
        elif modality == "camera_lidar_fusion":
            modes["object_recognition"]["merger"] = {"type": "camera_lidar_merger"}
        elif modality == "lidar_radar_fusion":
            modes["object_recognition"]["merger"] = {"type": "lidar_merger"}
            modes["object_recognition"]["merger"]["radar_object_fusion"] = "on"
        elif modality == "lidar":
            modes["object_recognition"]["merger"] = {"type": "lidar_merger"}
        else:
            pass

    if modes["object_recognition"]["merger"]:
        if modes["object_recognition"]["merger"]["type"] in ["camera_lidar_radar_merger", "camera_lidar_merger", "lidar_merger"]:
            if config_to_bool("use_detection_by_tracker", context):
                if config_to_bool("use_object_filter", context):
                    if config_to_str("objects_filter_method", context) == "lanelet_filter":
                        modes["object_recognition"]["merger"]["object_association_mergers"] = "detection_by_tracker_and_lanelet_filter"
                    elif config_to_str("objects_filter_method", context) == "position_filter":
                        modes["object_recognition"]["merger"]["object_association_mergers"] = "detection_by_tracker_and_position_filter"
                    else:
                        return KeyError(f"Invalid objects filter method: {config_to_str('objects_filter_method', context)}")
                else:
                    modes["object_recognition"]["merger"]["object_association_mergers"] = "detection_by_tracker"
            else:
                if config_to_bool("use_object_filter", context):
                    if config_to_str("objects_filter_method", context) == "lanelet_filter":
                        modes["object_recognition"]["merger"]["object_association_mergers"] = "lanelet_filter"
                    elif config_to_str("objects_filter_method", context) == "position_filter":
                        modes["object_recognition"]["merger"]["object_association_mergers"] = "position_filter"
                    else:
                        return KeyError(f"Invalid objects filter method: {config_to_str('objects_filter_method', context)}")
                else:
                    modes["object_recognition"]["merger"]["object_association_mergers"] = "single_merger"

        if modes["object_recognition"]["merger"]["type"] in ["camera_lidar_radar_merger", "camera_lidar_merger"]:
            modes["object_recognition"]["merger"]["filter__roi_detected_object_merger"] = "on"
            modes["object_recognition"]["merger"]["irregular_detector_merger"] = "on" if config_to_bool("use_irregular_object_detector", context) else "off"
            
        if modes["object_recognition"]["merger"]["type"] == "camera_lidar_radar_merger":
            modes["object_recognition"]["merger"]["radar_object_association_merger"] = "on" if not config_to_bool("use_radar_tracking_fusion", context) else "off"
            

    ## object prediction
    if config_to_bool("use_vector_map", context) == True:
        if config_to_str("prediction_model_type", context) == "map_based":
            modes["object_prediction"] = "map_based"
        else:
            modes["object_prediction"] = "simpl_model"
    else:
        modes["object_prediction"] = "off"

    # occupancy grid map
    if config_to_str("occupancy_grid_map_method", context) == "pointcloud_based_occupancy_grid_map":
        modes["occupancy_grid_map"] = "pointcloud_based"
    elif config_to_str("occupancy_grid_map_method", context) == "laserscan_based_occupancy_grid_map":
        modes["occupancy_grid_map"] = "laserscan_based"
    elif config_to_str("occupancy_grid_map_method", context) == "multi_lidar_pointcloud_based_occupancy_grid_map":
        modes["occupancy_grid_map"] = "multi_lidar_pointcloud_based"
    else:
        modes["occupancy_grid_map"] = "off"

    # traffic light recognition
    if config_to_bool("use_traffic_light_recognition", context) == False:
        modes["traffic_light_recognition"] = "off"
    elif config_to_bool("traffic_light_recognition/fusion_only", context) == True:
        if config_to_str("traffic_light_recognition/high_accuracy_detection_type", context) == "fine_detection":
            modes["traffic_light_recognition"] = "fusion_only_fine_detection"
        else:
            modes["traffic_light_recognition"] = "fusion_only"
    elif config_to_bool("traffic_light_recognition/use_high_accuracy_detection", context) == True:
        if config_to_str("traffic_light_recognition/high_accuracy_detection_type", context) == "fine_detection":
            modes["traffic_light_recognition"] = "high_accuracy_fine_detection"
        elif config_to_str("traffic_light_recognition/high_accuracy_detection_type", context) == "whole_image_detection":
            modes["traffic_light_recognition"] = "high_accuracy_whole_image_detection"
        else:
            modes["traffic_light_recognition"] = "standard_detection"
    else:
        modes["traffic_light_recognition"] = "standard_detection"

    return modes, parameters

def generate_autoware_architecture(deployment_file: str, mode: dict, parameters: dict):
    """Generate Autoware Architecture deployment."""
    # Load architecture configuration from YAML file

    architecture_config = ArchitectureConfig()
    architecture_config.debug_mode = True
    architecture_config.log_level = "INFO"

    logger = architecture_config.set_logging()
    
    logger.info("=== Determined mode: %s ===", pprint.pformat(mode, indent=2))
    logger.info("=== Parameters: %s ===", pprint.pformat(parameters, indent=2))

    workspace_root = os.path.dirname(get_package_prefix('autoware_architect')).rsplit('/', 1)[0]
    architect_pkg_build_dir = os.path.join(workspace_root, 'build', 'autoware_architect')
    launcher_pkg_install_dir = get_package_share_directory('tier4_perception_launch')

    architecture_config.architecture_manifest_dir = os.path.join(architect_pkg_build_dir, "resource")
    architecture_config.deployment_file = os.path.join(launcher_pkg_install_dir, deployment_file)
    architecture_config.output_root_dir = os.path.join(launcher_pkg_install_dir)
    architecture_config.domains = ['shared', 'dummy_modules']

    logger.info("Architecture YAML List File: %s", architecture_config.architecture_manifest_dir)
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

def opaque_generate_autoware_architecture(context, deployment_file: str):
    """OpaqueFunction wrapper to generate Autoware Architecture at launch time."""
    mode, parameters = determine_mode(context)
    
    deployment_file = "deployment/universe_perception_clr_centerpoint_serial.deployment.yaml"
    generate_autoware_architecture(deployment_file, mode, parameters)

    return []

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
    add_launch_arg("downsample_perception_common_pointcloud", default_value="false")

    # Object recognition
    add_launch_arg("mode", default_value="camera_lidar_radar_fusion")
    add_launch_arg("use_multi_channel_tracker_merger", default_value="false") # merger and tracker
    add_launch_arg("use_irregular_object_detector", default_value="true") # detector on/off, also for merger
    add_launch_arg("use_detection_by_tracker", default_value="true") # detector on/off, also for merger
    add_launch_arg("use_object_validator", default_value="true") # always true
    add_launch_arg("lidar_detection_model", default_value="centerpoint/centerpoint_tiny") 

    # Object recognition / detection / detector
    add_launch_arg("use_low_height_cropbox", default_value="false") # filter on/off of euclidean clustering

    # Object recognition / detection / detector / camera_lidar
    add_launch_arg("use_low_intensity_cluster_filter", default_value="true") # additional detector on/off, also for merger
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
    add_launch_arg("prediction_model_type", default_value="map_based") # prediction mode, map_based or SIMPL model

    # Obstacle segmentation
    add_launch_arg("use_obstacle_segmentation_single_frame_filter", default_value="false") # ground segmentation filter mode
    add_launch_arg("use_obstacle_segmentation_time_series_filter", default_value="true") # ground segmentation filter mode

    # Occupancy grid map
    add_launch_arg("occupancy_grid_map_method", default_value="pointcloud_based_occupancy_grid_map") # occupancy grid map mode
    # add_launch_arg("occupancy_grid_map_updater", default_value="binary_bayes_filter") # always binary_bayes_filter

    # traffic light
    add_launch_arg("use_traffic_light_recognition", default_value="true") # module switch. need for simulation
    add_launch_arg("traffic_light_recognition/fusion_only", default_value="false") # linked with ecu
    add_launch_arg("traffic_light_recognition/camera_namespaces", default_value="[camera6, camera7]") # sensor set
    add_launch_arg("traffic_light_recognition/use_high_accuracy_detection", default_value="true") # filter on/off
    add_launch_arg("traffic_light_recognition/high_accuracy_detection_type", default_value="fine_detection") # filter mode, whole_image_detection or fine_detection



    # determine mode
    deployment_file = "deployment/universe_perception_clr_centerpoint_serial.deployment.yaml"

    # Setup launcher file path
    launcher_pkg_install_dir = get_package_share_directory('tier4_perception_launch')
    launcher_path = "exports/universe_perception_clr_centerpoint_serial.deployment/launcher/Runtime/main_ecu/perception/perception.launch.xml"
    launcher_file = os.path.join(launcher_pkg_install_dir, launcher_path)

    # Create the pointcloud container
    pointcloud_container = create_pointcloud_container()

    # Then launch the generated launch file
    return LaunchDescription(
        launch_arguments + [
            OpaqueFunction(function=lambda context: opaque_generate_autoware_architecture(context, deployment_file)),
            pointcloud_container, 
            launch_generated_launch_file(launch_argument_names, launcher_file)
        ]
    )

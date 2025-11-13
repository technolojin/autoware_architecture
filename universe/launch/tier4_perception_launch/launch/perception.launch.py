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
from autoware_architect.config import SystemConfig
import pprint

from enum import Enum


class PerceptionModality(str, Enum):
    NONE = "none"
    CAMERA_LIDAR_RADAR_FUSION = "camera_lidar_radar_fusion"
    CAMERA_LIDAR_FUSION = "camera_lidar_fusion"
    LIDAR__RADAR_FUSION = "lidar_radar_fusion"
    LIDAR = "lidar"
    CAMERA = "camera"
    RADAR = "radar"

class ObstacleSegmentationFilterType(Enum):
    NoFilter = 0
    SingleFrame = 1
    TimeSeries = 2
    Combined = 3

class ObjectRecognitionFusionType(Enum):
    NONE = 0
    Serial = 1
    Parallel = 2

class ObjectRecognitionLidarModelType(Enum):
    NONE = 0
    Centerpoint = 1
    Pointpainting = 2
    Apollo = 3
    Transfusion = 4

class ObjectRecognitionPredictionType(Enum):
    NONE = 0
    MapBased = 1
    SimplModel = 2

class OccupancyGridMapType(Enum):
    NONE = 0
    PointcloudBased = 1
    LaserscanBased = 2
    MultiLidarPointcloudBased = 3

class TrafficLightRecognitionType(Enum):
    NONE = 0
    FusionOnly = 1
    FusionOnlyFineDetection = 2
    FineDetection = 3
    WholeImageDetection = 4
    StandardDetection = 5

def config_to_bool(config_name: str, context) -> bool:
    """Convert LaunchConfiguration to boolean."""
    value = LaunchConfiguration(config_name).perform(context)
    return value == "True" or value == "true"

def config_to_str(config_name: str, context) -> str:
    """Convert LaunchConfiguration to string."""
    value = LaunchConfiguration(config_name).perform(context)
    return str(value)

def determine_modes(context) -> tuple[bool, dict]:
    """Determine mode based on launch configurations."""
    modes = {
        "perception_modality": PerceptionModality.NONE,
        "object_recognition_fusion_type": ObjectRecognitionFusionType.NONE,
        "object_recognition_lidar_model_type": ObjectRecognitionLidarModelType.NONE,
        "object_recognition_prediction_type": ObjectRecognitionPredictionType.NONE,
        "occupancy_grid_map_type": OccupancyGridMapType.NONE,
        "traffic_light_recognition_type": TrafficLightRecognitionType.NONE,
    }
    is_supported = True
    
    # common
    if config_to_bool("downsample_perception_common_pointcloud", context) == True:
        is_supported = False
        return is_supported, modes

    if config_to_bool("use_perception_online_evaluator", context):
        modes["perception_online_evaluator"] = True
    else:
        modes["perception_online_evaluator"] = False

    if config_to_bool("use_perception_analytics_publisher", context):
        modes["perception_analytics_publisher"] = True
    else:
        modes["perception_analytics_publisher"] = False

    # obstacle segmentation
    if config_to_bool("use_obstacle_segmentation_time_series_filter", context):
        if config_to_bool("use_obstacle_segmentation_time_series_filter", context):
            modes["obstacle_segmentation_filter"] = ObstacleSegmentationFilterType.TimeSeries
        else:
            modes["obstacle_segmentation_filter"] = ObstacleSegmentationFilterType.SingleFrame
    else:
        if config_to_bool("use_obstacle_segmentation_time_series_filter", context):
            modes["obstacle_segmentation_filter"] = ObstacleSegmentationFilterType.TimeSeries
        else:
            modes["obstacle_segmentation_filter"] = ObstacleSegmentationFilterType.NoFilter

    # object recognition
    lidar_mode, lidar_model = config_to_str("lidar_detection_model", context).split('/')
    modality = config_to_str("mode", context)
    if modality == PerceptionModality.CAMERA_LIDAR_RADAR_FUSION:
        modes["perception_modality"] = PerceptionModality.CAMERA_LIDAR_RADAR_FUSION
    elif modality == PerceptionModality.CAMERA_LIDAR_FUSION:
        modes["perception_modality"] = PerceptionModality.CAMERA_LIDAR_FUSION
    elif modality == PerceptionModality.LIDAR__RADAR_FUSION:
        modes["perception_modality"] = PerceptionModality.LIDAR__RADAR_FUSION
    elif modality == PerceptionModality.LIDAR:
        modes["perception_modality"] = PerceptionModality.LIDAR
    elif modality == PerceptionModality.CAMERA:
        modes["perception_modality"] = PerceptionModality.CAMERA
    elif modality == PerceptionModality.RADAR:
        modes["perception_modality"] = PerceptionModality.RADAR
    else:
        is_supported = False
        return is_supported, modes

    # lidar detection model
    if lidar_mode == "centerpoint":
        modes["object_recognition_lidar_model_type"] = ObjectRecognitionLidarModelType.Centerpoint
    elif lidar_mode == "pointpainting":
        modes["object_recognition_lidar_model_type"] = ObjectRecognitionLidarModelType.Pointpainting
    elif lidar_mode == "apollo":
        modes["object_recognition_lidar_model_type"] = ObjectRecognitionLidarModelType.Apollo
    elif lidar_mode == "transfusion":
        modes["object_recognition_lidar_model_type"] = ObjectRecognitionLidarModelType.Transfusion
    else:
        is_supported = False
        return is_supported, modes

    # use_multi_channel_tracker_merger only supported for camera_lidar_radar_fusion
    if config_to_bool("use_multi_channel_tracker_merger", context) == True:
        if modes["perception_modality"] == PerceptionModality.CAMERA_LIDAR_RADAR_FUSION:
            modes["object_recognition_fusion_type"] = ObjectRecognitionFusionType.Parallel
        else:
            modes["object_recognition_fusion_type"] = ObjectRecognitionFusionType.Serial
    elif config_to_bool("use_multi_channel_tracker_merger", context) == True:
        # the other modalities do not support multi-channel tracker merger
        is_supported = False
        return is_supported, modes
    else:
        # the other modalities with serial fusion
        modes["object_recognition_fusion_type"] = ObjectRecognitionFusionType.Serial

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

    return is_supported, modes

def determine_launcher_paths(modes: dict) -> list[str]:
    """
    Determine launcher paths based on modes.
    
    Returns a list of launcher file paths (relative to package share directory).
    These are placeholders that should be filled based on the actual mode combinations.
    """
    launcher_paths = []
    
    # Placeholder: Determine launcher paths based on modes
    # This should be implemented based on the actual mode combinations
    
    # Example structure (to be replaced with actual logic):
    modality = modes.get("perception_modality")
    fusion_type = modes.get("object_recognition_fusion_type")
    lidar_model = modes.get("object_recognition_lidar_model_type")
    
    # Placeholder paths - these should be determined based on actual mode combinations
    if modality == PerceptionModality.CAMERA_LIDAR_RADAR_FUSION:
        if fusion_type == ObjectRecognitionFusionType.Parallel:
            if lidar_model == ObjectRecognitionLidarModelType.Centerpoint:
                launcher_paths.append("exports/CameraLidarRadarCenterpointParallel.system/launcher/Runtime/main_ecu/object_recognition/object_recognition.launch.xml")
        elif fusion_type == ObjectRecognitionFusionType.Serial:
            if lidar_model == ObjectRecognitionLidarModelType.Centerpoint:
                launcher_paths.append("exports/CameraLidarRadarCenterpointSerial.system/launcher/Runtime/main_ecu/object_recognition/object_recognition.launch.xml")
    elif modality == PerceptionModality.CAMERA_LIDAR_FUSION:
        # Placeholder for camera_lidar_fusion launchers
        launcher_paths.append("exports/CameraLidarCenterpoint.system/launcher/Runtime/main_ecu/object_recognition/object_recognition.launch.xml")
    elif modality == PerceptionModality.LIDAR__RADAR_FUSION:
        # Placeholder for lidar_radar_fusion launchers
        launcher_paths.append("exports/LidarRadarCenterpoint.system/launcher/Runtime/main_ecu/object_recognition/object_recognition.launch.xml")
    elif modality == PerceptionModality.LIDAR:
        # Placeholder for lidar-only launchers
        launcher_paths.append("exports/LidarCenterpoint.system/launcher/Runtime/main_ecu/object_recognition/object_recognition.launch.xml")
    elif modality == PerceptionModality.CAMERA:
        # Placeholder for camera-only launchers
        launcher_paths.append("exports/Camera.system/launcher/Runtime/main_ecu/object_recognition/object_recognition.launch.xml")
    elif modality == PerceptionModality.RADAR:
        # Placeholder for radar-only launchers
        launcher_paths.append("exports/Radar.system/launcher/Runtime/main_ecu/object_recognition/object_recognition.launch.xml")
    
    # Add other launchers based on other modes (obstacle_segmentation, traffic_light, etc.)
    # Placeholder: Add obstacle segmentation launcher if needed
    # Placeholder: Add traffic light recognition launcher if needed
    # Placeholder: Add occupancy grid map launcher if needed
    
    return launcher_paths

def determine_parameters(context, modes: dict) -> dict:
    """
    Determine parameters dictionary based on launch arguments and modes.
    
    This is a placeholder function. Parameters should be extracted from launch arguments
    and organized into a dictionary structure for passing to launchers.
    """
    parameters = {}
    
    # Placeholder: Extract and organize parameters from context
    # This should be implemented to extract all relevant launch arguments
    # and organize them into the appropriate structure
    
    # Example (to be replaced with actual logic):
    # parameters["data_path"] = config_to_str("data_path", context)
    # parameters["config_path"] = config_to_str("config_path", context)
    # parameters["vehicle_param_file"] = config_to_str("vehicle_param_file", context)
    # etc.
    
    return parameters

def create_pointcloud_container():
    """Create the pointcloud_container composable node container."""
    return ComposableNodeContainer(
        name="pointcloud_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
    )

def launch_xml_fallback(launch_arguments_names, xml_launcher_file: str):
    """Launch XML launcher file with all given arguments (fallback for unsupported modes)."""
    # Build launch arguments dictionary using LaunchConfiguration for each argument
    launch_args_dict = {name: LaunchConfiguration(name) for name in launch_arguments_names}

    return IncludeLaunchDescription(
        XMLLaunchDescriptionSource(xml_launcher_file),
        launch_arguments=launch_args_dict.items()
    )

def launch_prebuilt_launchers(launch_arguments_names, launcher_paths: list[str]):
    """Launch pre-built launcher files with all given arguments."""
    launcher_pkg_install_dir = get_package_share_directory('tier4_perception_launch')
    
    # Build launch arguments dictionary using LaunchConfiguration for each argument
    launch_args_dict = {name: LaunchConfiguration(name) for name in launch_arguments_names}
    
    launch_actions = []
    for launcher_path in launcher_paths:
        launcher_file = os.path.join(launcher_pkg_install_dir, launcher_path)
        launch_actions.append(
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(launcher_file),
                launch_arguments=launch_args_dict.items()
            )
        )
    
    return launch_actions

def opaque_launch_perception_system(context, launch_argument_names: list[str], xml_fallback_file: str):
    """
    OpaqueFunction wrapper to launch perception system at launch time.
    
    Flow:
    1. Parse arguments and determine modes and if the combination is supported
    2. If not supported, launch with XML, handing over all the given arguments
    3. If supported, determine set of launchers (pre-built) and launch them
    """
    import logging
    logger = logging.getLogger(__name__)
    
    # Parse arguments and determine modes
    is_supported, modes = determine_modes(context)
    
    # Determine parameters (placeholder - to be implemented)
    parameters = determine_parameters(context, modes)
    
    launch_actions = []
    
    if not is_supported:
        # If not supported, launch with XML fallback, handing over all given arguments
        logger.warning("Mode combination not supported. Using XML fallback. Modes: %s", pprint.pformat(modes, indent=2))
        launch_actions.append(launch_xml_fallback(launch_argument_names, xml_fallback_file))
    else:
        # If supported, determine set of launchers (pre-built) and launch them
        logger.info("Mode combination supported. Using pre-built launchers. Modes: %s", pprint.pformat(modes, indent=2))
        launcher_paths = determine_launcher_paths(modes)
        
        if not launcher_paths:
            logger.warning("No launcher paths determined for modes. Using XML fallback. Modes: %s", pprint.pformat(modes, indent=2))
            launch_actions.append(launch_xml_fallback(launch_argument_names, xml_fallback_file))
        else:
            logger.info("Launching pre-built launchers: %s", launcher_paths)
            launch_actions.extend(launch_prebuilt_launchers(launch_argument_names, launcher_paths))
    
    return launch_actions

def generate_launch_description():
    """ Generate autoware system and launch the generated launch file. """

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



    # 1. pipeline junctions: switches to change SW system 

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

    # Setup XML fallback launcher file path (for unsupported modes)
    launcher_pkg_install_dir = get_package_share_directory('tier4_perception_launch')
    xml_fallback_path = "launch/perception.launch.xml"
    xml_fallback_file = os.path.join(launcher_pkg_install_dir, xml_fallback_path)

    # Create the pointcloud container
    pointcloud_container = create_pointcloud_container()

    # Launch the perception system (will determine if supported and launch accordingly)
    return LaunchDescription(
        launch_arguments + [
            pointcloud_container,
            OpaqueFunction(function=lambda context: opaque_launch_perception_system(context, launch_argument_names, xml_fallback_file))
        ]
    )

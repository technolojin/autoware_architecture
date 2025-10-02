#!/usr/bin/env python3
# Copyright 2025 TIER IV, inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Refactored build script using the new architecture."""

import sys
import argparse
import logging
from pathlib import Path

# Add the autoware_architect package to the Python path
script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir))

from autoware_architect.config import config
from autoware_architect.builders.deployment_builder import DeploymentBuilder
from autoware_architect.exceptions import DeploymentError


def setup_argument_parser() -> argparse.ArgumentParser:
    """Setup command line argument parser."""
    parser = argparse.ArgumentParser(
        description="Build autoware perception architecture deployment",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s deployment.yaml elements.txt ./output
  %(prog)s deployment.yaml elements.txt ./output --executable my_executable --debug
        """
    )
    
    parser.add_argument(
        "deployment_file",
        type=Path,
        help="Path to deployment configuration YAML file"
    )
    
    parser.add_argument(
        "architecture_yaml_list_file",
        type=Path,
        help="Path to file containing list of architecture YAML directories"
    )
    
    parser.add_argument(
        "output_root_dir",
        type=Path,
        help="Root directory for generated output files"
    )
    
    parser.add_argument(
        "--executable",
        type=str,
        default="component_container",
        help="Executable name for launch files (default: component_container)"
    )
    
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug mode with verbose logging"
    )
    
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="Only validate configuration files without generating outputs"
    )
    
    return parser


def setup_logging(debug: bool = False) -> None:
    """Setup logging configuration."""
    level = logging.DEBUG if debug else logging.INFO
    format_str = '%(asctime)s - %(name)s - %(levelname)s - %(message)s' if debug else '%(levelname)s: %(message)s'
    
    logging.basicConfig(
        level=level,
        format=format_str,
        handlers=[logging.StreamHandler(sys.stdout)]
    )


def validate_input_files(args: argparse.Namespace) -> None:
    """Validate that input files exist and are accessible."""
    if not args.deployment_file.exists():
        raise DeploymentError(f"Deployment file not found: {args.deployment_file}")
    
    if not args.architecture_yaml_list_file.exists():
        raise DeploymentError(f"Architecture YAML list file not found: {args.architecture_yaml_list_file}")
    
    if not args.deployment_file.is_file():
        raise DeploymentError(f"Deployment file is not a regular file: {args.deployment_file}")
    
    if not args.architecture_yaml_list_file.is_file():
        raise DeploymentError(f"Architecture YAML list file is not a regular file: {args.architecture_yaml_list_file}")
    
    # Check if architecture yaml list file has content
    try:
        with open(args.architecture_yaml_list_file, 'r') as f:
            content = f.read().strip()
            if not content:
                raise DeploymentError(f"Architecture YAML list file is empty: {args.architecture_yaml_list_file}")
            
            # Validate that listed files exist
            lines = content.split('\n')
            missing_files = []
            for line in lines:
                line = line.strip()
                if line and not Path(line).exists():
                    missing_files.append(line)
            
            if missing_files:
                logger = logging.getLogger(__name__)
                logger.warning(f"Some YAML files listed in {args.architecture_yaml_list_file} do not exist:")
                for missing_file in missing_files[:5]:  # Show only first 5 missing files
                    logger.warning(f"  - {missing_file}")
                if len(missing_files) > 5:
                    logger.warning(f"  ... and {len(missing_files) - 5} more files")
                
    except Exception as e:
        raise DeploymentError(f"Error reading architecture YAML list file: {e}")


def validate_python_environment() -> None:
    """Validate that the Python environment has required packages."""
    logger = logging.getLogger(__name__)
    
    try:
        # Check for required modules
        import yaml
        import pathlib
        logger.debug("Python environment validation passed")
    except ImportError as e:
        raise DeploymentError(f"Required Python package not found: {e}")


def build(deployment_file: Path, architecture_yaml_list_file: Path, output_root_dir: Path, 
          executable: str = "component_container", validate_only: bool = False) -> None:
    """
    Build the deployment with improved error handling and validation.
    
    Args:
        deployment_file: Path to deployment configuration YAML file
        architecture_yaml_list_file: Path to file containing list of architecture YAML directories
        output_root_dir: Root directory for generated output files
        executable: Executable name for launch files
        validate_only: If True, only validate without generating outputs
    """
    logger = logging.getLogger(__name__)
    logger.info("Starting autoware architect deployment build...")
    
    try:
        # Validate Python environment
        validate_python_environment()
        
        # Create output directory if it doesn't exist
        output_root_dir.mkdir(parents=True, exist_ok=True)
        
        # Configure the system with user-provided executable
        if executable != "component_container":
            config.default_executable = executable
        
        # Build the deployment using the new architecture
        builder = DeploymentBuilder()
        deployment = builder.build_deployment(
            deployment_file=deployment_file,
            architecture_yaml_list_file=architecture_yaml_list_file,
            output_root_dir=output_root_dir
        )
        
        if validate_only:
            logger.info("Configuration validation completed successfully")
            return
        
        # Generate outputs
        logger.info("Generating system visualization...")
        deployment.visualize()
        
        logger.info("Generating launch files...")
        deployment.generate_launcher()
        
        logger.info("Generating system monitor configuration...")
        deployment.generate_system_monitor()
        
        logger.info("Deployment build completed successfully!")
        
    except Exception as e:
        logger.error(f"Build failed: {e}")
        if logger.getEffectiveLevel() == logging.DEBUG:
            logger.exception("Detailed error information:")
        raise


def main() -> int:
    """Main entry point."""
    # Check Python version
    if sys.version_info < (3, 6):
        print("ERROR: Python 3.6 or higher is required", file=sys.stderr)
        return 3
    
    parser = setup_argument_parser()
    
    # Handle legacy command line format for backward compatibility
    if len(sys.argv) == 4 and not any(arg.startswith('-') for arg in sys.argv[1:]):
        # Legacy format: build.py deployment.yaml elements.txt ./output
        args = argparse.Namespace(
            deployment_file=Path(sys.argv[1]),
            architecture_yaml_list_file=Path(sys.argv[2]),
            output_root_dir=Path(sys.argv[3]),
            executable="component_container",
            debug=False,
            validate_only=False
        )
    else:
        args = parser.parse_args()
    
    # Setup logging
    setup_logging(args.debug)
    logger = logging.getLogger(__name__)
    
    logger.info(f"Python version: {sys.version}")
    logger.info(f"Build script path: {Path(__file__).resolve()}")
    logger.info(f"Working directory: {Path.cwd()}")
    
    try:
        # Show input parameters for debugging
        logger.debug(f"Deployment file: {args.deployment_file}")
        logger.debug(f"Architecture YAML list file: {args.architecture_yaml_list_file}")
        logger.debug(f"Output root directory: {args.output_root_dir}")
        logger.debug(f"Executable: {args.executable}")
        logger.debug(f"Validate only: {args.validate_only}")
        
        # Validate input files
        validate_input_files(args)
        
        # Build deployment
        build(
            deployment_file=args.deployment_file,
            architecture_yaml_list_file=args.architecture_yaml_list_file,
            output_root_dir=args.output_root_dir,
            executable=args.executable,
            validate_only=args.validate_only
        )
        
        logger.info("Build process completed successfully")
        return 0
        
    except DeploymentError as e:
        logger.error(f"Deployment error: {e}")
        logger.error("Please check your deployment configuration and try again")
        return 1
    except KeyboardInterrupt:
        logger.warning("Build process was interrupted by user")
        return 130
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        logger.error("This may be a bug in the build system")
        if logger.getEffectiveLevel() == logging.DEBUG:
            logger.exception("Detailed error information:")
        return 2


if __name__ == "__main__":
    sys.exit(main())

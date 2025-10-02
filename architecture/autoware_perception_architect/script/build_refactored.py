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

# Import from the installed autoware_architect package
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
        "element_list_file", 
        type=Path,
        help="Path to text file containing list of element configuration paths"
    )
    
    parser.add_argument(
        "output_dir",
        type=Path,
        help="Output directory for generated files"
    )
    
    parser.add_argument(
        "--executable",
        type=str,
        default="default_executable",
        help="Name of the executable for modules (default: %(default)s)"
    )
    
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug mode"
    )
    
    parser.add_argument(
        "--log-level",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        default="INFO",
        help="Set logging level (default: %(default)s)"
    )
    
    return parser


def main():
    """Main entry point."""
    parser = setup_argument_parser()
    args = parser.parse_args()
    
    # Update configuration from command line arguments
    if args.debug:
        config.debug_mode = True
        config.log_level = "DEBUG"
    else:
        config.log_level = args.log_level
    
    # Setup logging
    logger = config.setup_logging()
    
    try:
        logger.info("Starting autoware architecture build...")
        logger.info(f"Deployment file: {args.deployment_file}")
        logger.info(f"Element list file: {args.element_list_file}")
        logger.info(f"Output directory: {args.output_dir}")
        logger.info(f"Executable name: {args.executable}")
        
        # Validate input files
        if not args.deployment_file.exists():
            raise DeploymentError(f"Deployment file not found: {args.deployment_file}")
        
        if not args.element_list_file.exists():
            raise DeploymentError(f"Element list file not found: {args.element_list_file}")
        
        # Build deployment using builder pattern
        deployment = (DeploymentBuilder()
                     .with_deployment_config(args.deployment_file)
                     .with_element_list_file(args.element_list_file)
                     .with_output_dir(args.output_dir)
                     .build())
        
        # Generate all artifacts
        artifacts = deployment.build_all(args.executable)
        
        # Report results
        logger.info("Build completed successfully!")
        logger.info("Generated artifacts:")
        
        if artifacts["launchers"]:
            logger.info(f"  - {len(artifacts['launchers'])} launcher files")
            if config.debug_mode:
                for launcher_path in artifacts["launchers"]:
                    logger.debug(f"    {launcher_path}")
        
        if artifacts["visualization"]:
            logger.info(f"  - Visualization: {artifacts['visualization']}")
        
        if artifacts["system_monitor"]:
            logger.info(f"  - System monitor: {artifacts['system_monitor']}")
        
        logger.info(f"Output directory: {args.output_dir}")
        
    except DeploymentError as exc:
        logger.error(f"Deployment error: {exc}")
        sys.exit(1)
    except Exception as exc:
        logger.error(f"Unexpected error: {exc}")
        if config.debug_mode:
            logger.exception("Full traceback:")
        sys.exit(1)


if __name__ == "__main__":
    main()

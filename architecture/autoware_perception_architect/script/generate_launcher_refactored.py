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

"""Refactored launcher generation script using the new architecture."""

import sys
import argparse
import logging
from pathlib import Path

# Add the autoware_architect package to the Python path
script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir))

from autoware_architect.config import config
from autoware_architect.models.elements import ElementFactory
from autoware_architect.generators.launcher_generator import launcher_generator
from autoware_architect.exceptions import ValidationError, ModuleConfigurationError


def setup_argument_parser() -> argparse.ArgumentParser:
    """Setup command line argument parser."""
    parser = argparse.ArgumentParser(
        description="Generate ROS 2 launcher file for a module",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s module.yaml my_executable ./output
  %(prog)s /path/to/ObjectDetector.module.yaml perception_node ./launch --debug
        """
    )
    
    parser.add_argument(
        "module_config",
        type=Path,
        help="Path to module configuration YAML file"
    )
    
    parser.add_argument(
        "executable_name",
        type=str,
        help="Name of the executable for the module"
    )
    
    parser.add_argument(
        "output_dir",
        type=Path,
        help="Output directory for the launcher file"
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
    
    parser.add_argument(
        "--template-dir",
        type=Path,
        help="Directory containing custom Jinja2 templates"
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
    
    if args.template_dir:
        config.template_dir = args.template_dir
    
    # Setup logging
    logger = config.setup_logging()
    
    try:
        logger.info("Starting launcher generation...")
        logger.info(f"Module config: {args.module_config}")
        logger.info(f"Executable name: {args.executable_name}")
        logger.info(f"Output directory: {args.output_dir}")
        
        # Validate input file
        if not args.module_config.exists():
            raise ValidationError(f"Module configuration file not found: {args.module_config}")
        
        # Load module element
        logger.debug("Loading module configuration...")
        module = ElementFactory.create_element(args.module_config)
        
        # Verify it's a module
        if module.type != "module":
            raise ValidationError(f"Expected module configuration, got: {module.type}")
        
        logger.info(f"Loaded module: {module.full_name}")
        
        # Generate launcher file
        logger.debug("Generating launcher file...")
        launcher_path = launcher_generator.generate_module_launcher(
            module=module,
            executable_name=args.executable_name,
            output_dir=args.output_dir
        )
        
        logger.info("Launcher generation completed successfully!")
        logger.info(f"Generated file: {launcher_path}")
        
        # Display launcher content in debug mode
        if config.debug_mode:
            logger.debug("Generated launcher content:")
            with open(launcher_path, 'r') as f:
                content = f.read()
            for i, line in enumerate(content.split('\n'), 1):
                logger.debug(f"{i:3d}: {line}")
        
    except ModuleConfigurationError as exc:
        logger.error(f"Module configuration error: {exc}")
        sys.exit(1)
    except ValidationError as exc:
        logger.error(f"Validation error: {exc}")
        sys.exit(1)
    except Exception as exc:
        logger.error(f"Unexpected error: {exc}")
        if config.debug_mode:
            logger.exception("Full traceback:")
        sys.exit(1)


if __name__ == "__main__":
    main()

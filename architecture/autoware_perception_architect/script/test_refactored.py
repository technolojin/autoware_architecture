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

"""Test script to verify the refactored architecture works."""

import sys
import tempfile
import yaml
from pathlib import Path

# Add the autoware_architect package to the Python path
script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir))

# Import from the refactored package
try:
    from autoware_architect.config import config
    from autoware_architect.models.elements import ElementFactory
    from autoware_architect.generators.launcher_generator import launcher_generator
    from autoware_architect.builders.deployment_builder import DeploymentBuilder
except ImportError as e:
    print(f"Import error: {e}")
    print("Make sure the autoware_architect package is properly set up")
    sys.exit(1)


def create_test_module_config():
    """Create a test module configuration."""
    test_config = {
        "name": "TestModule.module",
        "launch": {
            "package": "test_package",
            "plugin": "test_package::TestNode",
            "node_output": "screen",
            "use_container": False
        },
        "inputs": [
            {"name": "image", "type": "sensor_msgs/msg/Image"},
            {"name": "pointcloud", "type": "sensor_msgs/msg/PointCloud2"}
        ],
        "outputs": [
            {"name": "objects", "type": "autoware_msgs/msg/DetectedObjects"}
        ],
        "parameters": [
            {"name": "config_file", "default": "$(find-pkg-share test_package)/config/test.yaml", "allow_substs": True}
        ],
        "configurations": [
            {"name": "use_gpu", "type": "bool", "default": True},
            {"name": "threshold", "type": "float", "default": 0.5}
        ],
        "processes": [
            {"name": "detection_process", "trigger": ["image"], "output": ["objects"]}
        ]
    }
    return test_config


def test_element_loading():
    """Test element loading functionality."""
    print("Testing element loading...")
    
    # Create temporary config file
    test_config = create_test_module_config()
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        yaml.dump(test_config, f)
        config_path = Path(f.name)
    
    try:
        # Load element
        element = ElementFactory.create_element(config_path)
        
        # Verify element properties
        assert element.name == "TestModule"
        assert element.type == "module"
        assert element.package_name == "test_package"
        assert len(element.inputs) == 2
        assert len(element.outputs) == 1
        
        print("✓ Element loading test passed")
        return element
        
    finally:
        config_path.unlink()


def test_launcher_generation(module):
    """Test launcher generation functionality."""
    print("Testing launcher generation...")
    
    with tempfile.TemporaryDirectory() as temp_dir:
        output_dir = Path(temp_dir)
        
        # Generate launcher
        launcher_path = launcher_generator.generate_module_launcher(
            module=module,
            executable_name="test_executable",
            output_dir=output_dir
        )
        
        # Verify file was created
        assert launcher_path.exists()
        assert launcher_path.suffix == ".xml"
        
        # Check content contains expected elements
        content = launcher_path.read_text()
        assert "TestModule.module.launch.xml" in str(launcher_path)
        assert "test_package" in content
        assert "test_executable" in content
        assert "input/image" in content
        assert "output/objects" in content
        
        print("✓ Launcher generation test passed")
        print(f"  Generated file: {launcher_path.name}")


def test_deployment_builder():
    """Test deployment builder functionality."""
    print("Testing deployment builder...")
    
    # Create test deployment config
    deployment_config = {
        "name": "TestDeployment.deployment",
        "components": [
            {"name": "TestModule", "type": "module"}
        ],
        "connections": []
    }
    
    # Create test element list
    test_module_config = create_test_module_config()
    
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_path = Path(temp_dir)
        
        # Create deployment config file
        deployment_file = temp_path / "deployment.yaml"
        with open(deployment_file, 'w') as f:
            yaml.dump(deployment_config, f)
        
        # Create module config file
        module_file = temp_path / "module.yaml"
        with open(module_file, 'w') as f:
            yaml.dump(test_module_config, f)
        
        # Create element list file
        element_list_file = temp_path / "elements.txt"
        with open(element_list_file, 'w') as f:
            f.write(str(module_file))
        
        output_dir = temp_path / "output"
        
        try:
            # Build deployment
            deployment = (DeploymentBuilder()
                         .with_deployment_config(deployment_file)
                         .with_element_list_file(element_list_file)
                         .with_output_dir(output_dir)
                         .build())
            
            # Generate artifacts
            artifacts = deployment.build_all("test_executable")
            
            # Verify artifacts were created
            assert "launchers" in artifacts
            assert len(artifacts["launchers"]) > 0
            
            print("✓ Deployment builder test passed")
            print(f"  Generated {len(artifacts['launchers'])} launcher files")
            
        except Exception as e:
            print(f"✗ Deployment builder test failed: {e}")
            raise


def main():
    """Run all tests."""
    print("Running refactored architecture tests...\n")
    
    # Setup configuration for testing
    config.debug_mode = True
    config.log_level = "WARNING"  # Reduce log noise during tests
    logger = config.setup_logging()
    
    try:
        # Test 1: Element loading
        module = test_element_loading()
        
        # Test 2: Launcher generation
        test_launcher_generation(module)
        
        # Test 3: Deployment builder
        test_deployment_builder()
        
        print("\n✓ All tests passed! The refactored architecture is working correctly.")
        
    except Exception as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

# Autoware Perception Architect - Refactored

This directory contains the refactored version of the Autoware Perception Architecture system, which generates ROS 2 launch files and manages component deployments.

## 🏗️ Architecture Overview

The refactored system follows modern software engineering principles:

```
autoware_architect/
├── deployment.py          # Main process
├── config.py              # Configuration management
├── exceptions.py           # Custom exception classes
├── models/                 # Data models
│   ├── elements.py         # Architecture elements (modules, pipelines)
│   ├── parameters.py       # Parameter management
│   ├── events.py          # Event system (placeholder)
│   ├── ports.py           # Port and connection models (placeholder)
│   └── instances.py       # Instance models (placeholder)
├── parsers/               # Configuration parsing
│   └── yaml_parser.py     # YAML configuration parser with caching
├── generators/            # Output generation
│   └── launcher_generator.py  # ROS 2 launch file generator
├── builders/              # Build orchestration
│   └── deployment_builder.py  # Deployment builder
└── utils/                 # Utilities
    └── naming.py          # Naming conventions (PascalCase ↔ snake_case)
```

## 🚀 Key Improvements

### 1. **Separation of Concerns**
- **Models**: Pure data structures without business logic
- **Parsers**: Configuration loading and caching
- **Generators**: Output file generation (launchers, visualizations)
- **Builders**: Orchestration and workflow management

### 2. **Error Handling**
- Custom exception hierarchy for better error messages
- Comprehensive validation at each step
- Graceful failure handling with detailed logging

### 3. **Configuration Management**
- Centralized configuration with environment variable support
- Configurable logging levels and caching
- Template directory customization

### 4. **Template System**
- Jinja2 template support for flexible launcher generation
- Fallback to built-in templates for simple cases
- Easy customization of output formats

### 5. **Builder Pattern**
- Fluent API for creating deployments
- Method chaining for readable configuration
- Validation before build execution

## 📋 Usage Examples

### Basic Launcher Generation

```python
from autoware_architect.models.elements import ConfigFactory
from autoware_architect.generators.launcher_generator import launcher_generator

# Load module configuration
module = ConfigFactory.create_element("path/to/module.yaml")

# Generate launcher
launcher_path = launcher_generator.generate_module_launcher(
    module=module,
    executable_name="my_executable", 
    output_dir="./launchers"
)
```

### Complete Deployment Build

```python
from autoware_architect.builders.deployment_builder import DeploymentBuilder

# Build deployment using fluent API
deployment = (DeploymentBuilder()
             .with_deployment_config("deployment.yaml")
             .with_element_list_file("elements.txt")
             .with_output_dir("./output")
             .build())

# Generate all artifacts
artifacts = deployment.build_all("my_executable")
```

### Command Line Scripts

```bash
# Generate single launcher
./generate_launcher_refactored.py module.yaml my_executable ./output

# Build complete deployment
./build_refactored.py deployment.yaml elements.txt ./output --executable my_executable --debug
```

## 🔧 Configuration

### Environment Variables

```bash
export AUTOWARE_ARCHITECT_DEBUG=true              # Enable debug mode
export AUTOWARE_ARCHITECT_LOG_LEVEL=DEBUG         # Set logging level
export AUTOWARE_ARCHITECT_CACHE_ENABLED=true     # Enable caching
export AUTOWARE_ARCHITECT_TEMPLATE_DIR=/path/to/templates  # Custom templates
```

### Template Customization

Create custom Jinja2 templates in your template directory:

```xml
<!-- module_launcher.xml.j2 -->
<?xml version="1.0"?>
<launch>
  <arg name="node_name" default="{{ node_name }}"/>
  
  {% for input_item in inputs %}
  <arg name="input/{{ input_item.name }}" default="{{ input_item.name }}"/>
  {% endfor %}
  
  <!-- ... rest of template ... -->
</launch>
```

## 🧪 Testing

Run the test suite to verify functionality:

```bash
# Run simple tests (no complex dependencies)
./test_simple.py

# Run comprehensive tests (requires full setup)
./test_refactored.py
```

## 📊 Benefits of Refactoring

### Before (Original Code)
- ❌ Mixed concerns in single files
- ❌ Global variables and debug flags
- ❌ String concatenation for XML generation
- ❌ Limited error handling
- ❌ Hardcoded templates

### After (Refactored Code)
- ✅ Clean separation of concerns
- ✅ Centralized configuration management
- ✅ Template-based generation with Jinja2
- ✅ Comprehensive error handling and validation
- ✅ Extensible architecture with clear interfaces
- ✅ Type hints and proper documentation
- ✅ Caching for improved performance
- ✅ Builder pattern for complex workflows

## 🔮 Future Enhancements

The refactored architecture provides a solid foundation for:

1. **Complete Event System**: Full implementation of event chains and processes
2. **Advanced Visualization**: Generate PlantUML diagrams and interactive dashboards
3. **System Monitoring**: Real-time monitoring configuration generation
4. **Plugin Architecture**: Support for custom generators and validators
5. **Performance Optimization**: Parallel processing and advanced caching
6. **Testing Framework**: Automated testing of generated configurations

## 🔄 Migration Guide

### For Existing Scripts

The original scripts still work, but new scripts should use the refactored API:

```python
# Old way
from generate_launcher import generate_launcher
generate_launcher(module_yaml_dir, executable_name, launch_file_dir)

# New way
from autoware_architect.builders.deployment_builder import DeploymentBuilder
deployment = DeploymentBuilder().with_deployment_config(...).build()
artifacts = deployment.build_all(executable_name)
```

### For CMake Integration

Update your CMakeLists.txt to use the new scripts:

```cmake
# Use refactored build script
autoware_architect_build_deploy_refactored(${PROJECT_NAME} deployment.yaml)
```

## 📚 API Reference

See individual module documentation for detailed API reference:

- `autoware_architect.models.elements` - Architecture element models
- `autoware_architect.generators.launcher_generator` - Launch file generation
- `autoware_architect.builders.deployment_builder` - Deployment orchestration
- `autoware_architect.config` - Configuration management

---

*This refactored architecture maintains backward compatibility while providing a modern, extensible foundation for future development.*




## install
```sh
# clone
git clone git@github.com:technolojin/autoware_architecture.git autoware_architecture/src
cd autoware_architecture
vcs import src < build_depends.repos 
```

# build
```sh
colcon build
```

* build log
overall build log: `log/latest/autoware_perception_deployment/stdout.log`
each deployment `build/autoware_perception_deployment/*.deployment.log`
ex. `build/autoware_perception_deployment/build_vehicle_beta2_01.deployment.log`



clean build
```sh
rm build -r ; rm install -r ; colcon build
```


# launch

```sh
ros2 launch tier4_perception_launch perception.launch.py
```


* launch component container
```sh
ros2 run rclcpp_components component_container --ros-args -r __node:=pointcloud_container --log-level info
```

## visualize

### graphviz dot

* install vscode plugin `graphviz-interactive-preview` https://marketplace.visualstudio.com/items?itemName=tintinweb.graphviz-interactive-preview

### Web view

> Access after build

[Visualization](../install/architecture_visualization.html)


# Glossary


new definitions

* entity definition
  * system
  * module
  * node
  * parameter set

* member of entity
  * parameter
  * port
  * process
  * connection

* instance
  * deployment
  * component
  * topic
  * parameter

* abstraction (invisible from user)
  * event
  * link


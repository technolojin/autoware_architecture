


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

```
ros2 launch tier4_perception_launch perception.launch.py node_name:=nodename data_path:=/datapath
```


* launch component container
```sh
ros2 run rclcpp_components component_container --ros-args -r __node:=pointcloud_container --log-level info
```

## visualize

generated files: install/autoware_perception_deployment/share/autoware_perception_deployment/visualization/*.puml

* install vscode PlantUML plugin: https://marketplace.visualstudio.com/items?itemName=jebbs.plantuml
  - press `alt + D` for preview

* manual install and launch
1. 
```sh
sudo apt install plantuml
plantuml -v
```
to get latest version, please visit https://plantuml.com/download
then replace existing 
```sh
sudo mv plantuml.jar /usr/share/plantuml/plantuml.jar
```
2. 
```sh
java -Xmx16384m -DPLANTUML_LIMIT_SIZE=65536 -jar /usr/share/plantuml/plantuml.jar -verbose -Playout=smetana -tpng 'install/autoware_perception_deployment/share/autoware_perception_deployment/exports/vehicle_beta2_01.deployment/visualization/vehicle_beta2_01.deployment_node_graph.puml'
java -Xmx16384m -DPLANTUML_LIMIT_SIZE=65536 -jar /usr/share/plantuml/plantuml.jar -verbose -Playout=smetana -tpng 'install/autoware_perception_deployment/share/autoware_perception_deployment/exports/vehicle_beta2_01.deployment/visualization/vehicle_beta2_01.deployment_logic_graph.puml'
java -Xmx16384m -DPLANTUML_LIMIT_SIZE=65536 -jar /usr/share/plantuml/plantuml.jar -verbose -Playout=smetana -tpng 'install/autoware_perception_deployment/share/autoware_perception_deployment/exports/vehicle_beta2_01.deployment/visualization/vehicle_beta2_01.deployment_sequence_graph.puml'
```




# words

* element
  * architecture
  * pipeline
  * module

* instance
  * deployment
  * component
  * connection

* port
* link, connection
* event, process
* parameter set





## install
```sh
# clone
git clone git@github.com:technolojin/autoware_architecture.git autoware_architecture/src
cd autoware_architecture
vcs import src < build_depends.repos 

# build
colcon build
```


## visualize

generated files: install/autoware_perception_deployment/share/autoware_perception_deployment/visualization/*.puml

* install vscode PlantUML plugin: https://marketplace.visualstudio.com/items?itemName=jebbs.plantuml
  - press `alt + D` for preview

* manual install
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
java -Xmx16384m -DPLANTUML_LIMIT_SIZE=65536 -jar /usr/share/plantuml/plantuml.jar -verbose -Playout=smetana -tpng 'install/autoware_perception_deployment/share/autoware_perception_deployment/visualization/x2_gen2.deployment_sequence_graph.puml'
```
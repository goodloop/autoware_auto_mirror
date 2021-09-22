Launching scenario simulator with AutowareAuto {#scenario-simulator-howto}
=======================================

@tableofcontents

# Setting up

It is assumed that you are running AutowareAuto in ADE environment with default paths. See [Autoware.Auto documentation](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation-ade.html) for instructions how to correctly set up Autoware.Auto and ADE.

* update, run and enter ADE
```
ade --rc .aderc-amd64-foxy-lgsvl start --update --enter
```
* go to `AutowareAuto` directory
```
cd AutowareAuto
```
* import required repos
```
vcs import < autoware.auto.foxy.repos
./get_scenario_simulator_v2.sh
```
* install dependencies for `scenario_test_runner` that are not present in ADE
```
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y
```
* build Autoware, `scenario_test_runner` and `kashiwanoha_map` (needed for this demo only) 
 ```
colcon build --packages-up-to scenario_test_runner scenario_simulator_launch kashiwanoha_map --cmake-args -DCMAKE_BUILD_TYPE=Release
 ```

## Build troubleshooting

In case of build problems, please refer to @ref building.

In case where `ros-foxy-autoware-auto-msgs` is installed on the system, colcon uses it instead of
the one in the `AutowareAuto/src/external/` folder. This may cause errors.
To prevent this, please remove the package by:

```{bash}
sudo apt purge -y ros-foxy-autoware-auto-msgs
```

In case an error occurs related to `acado` package, please run:

```{bash}
sudo apt purge -y ros-foxy-acado-vendor
sudo apt install -y ros-foxy-acado-vendor
```

# Usage

### Assumption

Autoware launch file needs to be ran from the repo root directory (inside `AutowareAuto` directory). This is a temporary tradeoff to simplify usage.

### Steps

In `ade` in `AutowareAuto` do the following:

* launch rviz2 with Autoware.Auto avp config
```
source install/setup.bash
rviz2 -d $(ros2 pkg prefix autoware_auto_launch)/share/autoware_auto_launch/config/avp.rviz
```
* launch scenario test runner
```
source install/setup.bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py sensor_model:=aip_xx1 vehicle_model:=lexus scenario:=src/external/scenario_simulator/test_runner/scenario_test_runner/test/scenario/AutowareAutoDemo.yaml architecture_type:=awf/auto
```

> _NOTE:_ You can ignore Rviz2 errors about missing models for 3D vehicle representation.

### Output

* Kashiwanoha map is placed far from the origin of `map` coordinate system (`x: 73805, y: 73805`) so it won't be visible on Rviz2 start, 
* view should correctly center on `base_link` just after `scenario_test_runner` launch (if not, just change current view to orbit around `base_link`),
* since the vehicle is not being rendered, it is best to change the `TF` marker scale to around `10` for better visibility,
* green bounding boxes representing trajectory should appear along scenario path,
* vehicle should start moving after about 30 seconds of warming up,
* vehicle should reach the goal
* success logs should appear in terminal:
```
[openscenario_interpreter_node-3] [INFO] [simulation.openscenario_interpreter]: Deactivate myself.
[openscenario_interpreter_node-3] [INFO] [simulation.openscenario_interpreter]: Deactivating.
[openscenario_interpreter_node-3] [INFO] [simulation.concealer]: Shutting down Autoware: (1/2) Stop publlishing/subscribing.
[openscenario_interpreter_node-3] [INFO] [simulation.concealer]: Shutting down Autoware: (1/2) Stoped publlishing/subscribing.
[openscenario_interpreter_node-3] [INFO] [simulation.concealer]: Shutting down Autoware: (2/2) Send SIGINT to Autoware launch process.
[openscenario_interpreter_node-3] [WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[openscenario_interpreter_node-3] [INFO] [simulation.openscenario_interpreter]: Success
```
* the scenario should run three times
* after third run, the program should shut down

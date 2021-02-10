IDE Specific Configuration {#ide-specific-configuration}
===========

[TOC]

# Supported IDEs

You can use any IDE or text editor to use or develop Autoware Auto. Here you can find how to 
configure some of them.

- @ref configuration-clion "CLion"
- @ref configuration-vscode "VS Code"

## CLion {#configuration-clion}

The key functionality that will make CLion able to index is the use of [Compilation Database](https://www.jetbrains.com/help/clion/compilation-database.html).

colcon tool lets users generate the `compile_commands.json` file required for loading them into CLion
with following argument:

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

You can find more information on it from [colcon documentation](https://colcon.readthedocs.io/en/released/user/how-to.html#cmake-packages-generating-compile-commands-json).

### Building

You can use the Terminal within the Clion to do the building.

Make sure you went through the @ref building "Building" instructions.
Because in order to be able to debug your code with Clion, you should compile with either `Debug` or
`RelWithDebInfo` flags. You can use `RelWithDebInfo` most of the times without problems.

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --packages-up-to <package_name>
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --packages-select <package_name>
```

And you can make some `.bash_aliases` aliases to simplify the building process.
```bash
alias colcon_build_reldeb="colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1"
alias colcon_build_reldeb_upto="colcon_build_reldeb --packages-up-to "
alias colcon_build_reldeb_sel="colcon_build_reldeb --packages-select "
```

### Opening up the project in CLion

Once you have created a compiled the Autoware with the commands above, you can load it into CLion.

Navigate to File | Open on the main menu, choose the compile_commands.json 
(it will be located in `AutowareAuto/build/` folder) file and click Open as Project.

By default, the project root is set to the directory containing the compilation database file, 
in our case this is the `AutowareAuto/build/ folder`. 

To change the project root, select Tools | Compilation Database | Change Project Root from the main menu, 
and select `AutowareAuto` directory from there.

Now CLion's code insight, refactoring, analysis, and navigation are fully available for your project.

To finish things up,
- Project Pane -> right click `install` folder and select `Mark Directory as -> Excluded`
- Project Pane -> right click `src` folder and select `Mark Directory as -> Project Sources and Headers`

And it should look like this:

@image html images/ide-configuration-clion-first-run.png "CLion First Run"

### Running and debugging the nodes

For now you can debug anything that you can run with `ros2 run package_name executable_name <param1> <param2>...`

And you cannot directly debug the things that you run through the `ros2 launch` although it should be
fairly simple to [have Clion Attach to Process](https://www.jetbrains.com/help/clion/attaching-to-local-process.html).

#### Example for running the "point_cloud_filter_transform_nodes" from the perception/filters in ROS2 Foxy:**

Normally you can run this node with following commands:
```bash
# In my pc, the AutowareAuto is located in /home/mfc/projects/
# Please update this path with your configuration

source /home/mfc/projects/AutowareAuto/install/setup.bash
ros2 run point_cloud_filter_transform_nodes point_cloud_filter_transform_node_exe --ros-args -r __ns:=/lidar_front --params-file /home/mfc/projects/AutowareAuto/src/perception/filters/point_cloud_filter_transform_nodes/param/vlp16_sim_lexus_filter_transform.param.yaml -r __node:=filter_transform_vlp16_front -r points_in:=/lidar_front/points_raw
```
To be able to run and/or debug this file with Clion, do the following:

1. Call Run | Edit Configurations from the menu or Click `Add Configuration...` from top right near build buttons.
   @image html images/ide-configuration-clion-run-configuration-empty.png "Empty Run/Debug Configurations"

2. Click on the plus button on top-left and pick `Custom Build Application`
   @image html images/ide-configuration-clion-run-configuration-custom-build-application.png "Custom Build Application"

3. Click `Configure Custom Build Targets`
   @image html images/ide-configuration-clion-custom-build-targets.png "Custom Build Targets"
   
4. Click the plus button from top-left to add a custom build target. 
   Leave the rest as they are, as this is a dummy target. And click OK.
   
5. In the `Custom Build Application` screen select the `Custom Build Target` for `Target:` that you have generated.

6. Executable: `/home/mfc/projects/AutowareAuto/install/point_cloud_filter_transform_nodes/lib/point_cloud_filter_transform_nodes/point_cloud_filter_transform_node_exe`

7. Program Arguments: `--ros-args -r __ns:=/lidar_front --params-file /home/mfc/projects/AutowareAuto/src/perception/filters/point_cloud_filter_transform_nodes/param/vlp16_sim_lexus_filter_transform.param.yaml -r __node:=filter_transform_vlp16_front -r points_in:=/lidar_front/points_raw`

8. (optional) Working Directory: `/home/mfc/projects/AutowareAuto/install/`
   
9. Environment Variables: `source /home/mfc/projects/AutowareAuto/install/setup.bash`

10. In the `Before Launch` list, press the minus button to remove the `Build` from the list.

In the end it should look like this:

@image html images/ide-configuration-clion-run-configuration-done.png "Run/Debug Configurations Done"

After you click OK, you now should be able to click the Triangle or the Bug button and run or debug your application :)

#### Configuring for other nodes

You can just change the `Executable` and `Program Arguments` then it should work with other nodes too.

## VS Code {#configuration-vscode}

Lorem ipsum.
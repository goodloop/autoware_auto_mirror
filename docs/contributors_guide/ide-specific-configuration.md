IDE Specific Configuration {#ide-specific-configuration}
===========

@tableofcontents

Any IDE or text editor can be used to develop Autoware.Auto. Here, the configuration directives 
for some can be found.

# CLion {#configuration-clion}

Here, Autoware.Auto was installed @ref installation-no-ade "without ADE" in the "~/projects/AutowareAuto" path.

The key functionality that will make CLion able to index is the use of a [Compilation Database](https://www.jetbrains.com/help/clion/compilation-database.html).

## Building

Terminal within CLion can be used to build the Autoware.Auto.

@ref building "Building" has the fundamental instructions for this section.

@ref building-compilation-optimization-flags "Building with compilation database" should be followed to generate the Compilation Database.

In order to be able to debug the code with CLion, the code should be compiled with either `Debug` or
`RelWithDebInfo` flags. `RelWithDebInfo` flag can be used most of the time without problems.

```{bash}
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --packages-up-to <package_name>
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --packages-select <package_name>
```

Also some `bash` aliases can be set in the `~/.bash_aliases` file to simplify the building process.
```{bash}
alias colcon_build_reldeb="colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1"
alias colcon_build_reldeb_upto="colcon_build_reldeb --packages-up-to "
alias colcon_build_reldeb_sel="colcon_build_reldeb --packages-select "
```

## Opening up the project in CLion

Once Autoware.Auto is compiled with the commands above, it can be loaded into CLion.

`Navigate to File | Open` in the main menu, then choose `compile_commands.json`
(it will be located in `AutowareAuto/build/` folder) file and click Open as Project.

By default, the project root is set to the directory containing the compilation database file which,
in our case, is the `AutowareAuto/build/` folder. 

To change the project root, select Tools | Compilation Database | Change Project Root from the main menu, 
and select the `AutowareAuto` directory from there.

Now CLion's code insight, refactoring, analysis, and navigation are fully available for the project.

To finish things up,
- Project Pane -> right click `install` folder and select `Mark Directory as -> Excluded`
- Project Pane -> right click `src` folder and select `Mark Directory as -> Project Sources and Headers`

And it should look like this:

@image html images/ide-configuration-clion-first-run.png "CLion First Run" width=90%

## Running and debugging the nodes

Now anything that can be run with `ros2 run package_name executable_name <param1> <param2>...` can be debugged.

The nodes that are run with `ros2 launch` command cannot be debugged with following method although it should be
fairly simple to [have Clion Attach to a Process](https://www.jetbrains.com/help/clion/attaching-to-local-process.html).

### Example for running the "point_cloud_filter_transform_nodes" from the perception/filters in ROS2 Foxy:**

Normally this node can be run with following commands:
```{bash}
# In my pc, the AutowareAuto is located in ~/projects/
# Please update this path with your configuration

source ~/projects/AutowareAuto/install/setup.bash
ros2 run point_cloud_filter_transform_nodes point_cloud_filter_transform_node_exe --ros-args -r __ns:=/lidar_front --params-file ~/projects/AutowareAuto/src/perception/filters/point_cloud_filter_transform_nodes/param/vlp16_sim_lexus_filter_transform.param.yaml -r __node:=filter_transform_vlp16_front -r points_in:=/lidar_front/points_raw
```

There are some [Path Variables](https://www.jetbrains.com/help/clion/absolute-path-variables.html) 
in CLion that make it easy to shorten some paths in certain places.
Here `$ContentRoot$` will be used to point to the project root which is `~/projects/AutowareAuto`.

To be able to run and/or debug this file with CLion, do the following:

1. Call Run | Edit Configurations from the menu or Click `Add Configuration...` from top right near build buttons.
   @image html images/ide-configuration-clion-run-configuration-empty.png "Empty Run/Debug Configurations" width=90%

2. Click on the plus button on top-left and pick `Custom Build Application`
   @image html images/ide-configuration-clion-run-configuration-custom-build-application.png "Custom Build Application" width=90%

3. Click `Configure Custom Build Targets`
   @image html images/ide-configuration-clion-custom-build-targets.png "Custom Build Targets" width=90%
   
4. Click the plus button from top-left to add a custom build target. 
   Leave the rest as they are, as this is a dummy target. And click OK.
   
5. In the `Custom Build Application` screen select the `Custom Build Target` for `Target:` that you have generated.

6. Executable: `$ContentRoot$/install/point_cloud_filter_transform_nodes/lib/point_cloud_filter_transform_nodes/point_cloud_filter_transform_node_exe`

7. Program Arguments: `--ros-args -r __ns:=/lidar_front --params-file $ContentRoot$/src/perception/filters/point_cloud_filter_transform_nodes/param/vlp16_sim_lexus_filter_transform.param.yaml -r __node:=filter_transform_vlp16_front -r points_in:=/lidar_front/points_raw`

8. (optional) Working Directory: `$ContentRoot$/install/`
   
9. Environment Variables: `source /home/mfc/projects/AutowareAuto/install/setup.bash` (Absolute path is required here)

10. In the `Before Launch` list, press the minus button to remove the `Build` from the list.

In the end it should look like this:

@image html images/ide-configuration-clion-run-configuration-done.png "Run/Debug Configurations Done" width=90%

After clicking OK, it should now be possible to click the Triangle or the Bug button to run or debug the application :)

### Configuring for other nodes

The `Executable` and `Program Arguments` can be modified to make it work with any other node.

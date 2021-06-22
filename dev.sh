#!/bin/bash
PKG=trajectory_follower_nodes
#PKG=trajectory_follower

case $1 in
	start)
		ade start --update --enter
	;;
	test)
		colcon test --packages-select "$PKG" --event-handlers console_direct+
	;;
	debug)
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select "$PKG"
	;;
	format)
		ament_uncrustify $(find src/ -type d -name $PKG | head -n 1) --reformat
	;;
	build)
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select "$PKG"
	;;
	*)
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select "$PKG"
	;;
esac


# Codebase for Lucia Winter School 2013 (Meta-CSP/ROS, T-Rex/Europa/ROS) #

This project contains sources for the Meta-CSP and Context Recognition exercises at the 2013 Lucia Winter School on AI and Robotics.

Four packages are provided:

  * `go_turtle`: simple node for sending commands to Turtlebot2, and receiving feedback.

  * `constraints-playground`: simple Java project with test class for general constraint reasoning exercises.  This depends on the  Meta-CSP Framework (see [metacsp.org](http://metacsp.org)), which is downloaded automatically by the build process (from the Maven Central repository).

  * `lucia`: provides a `contextrecognition` node (to be used with pre-recorded rosbags, also provided).  This also builds on the Meta-CSP Framework (downloaded automatically from Maven Central).

  * `myjava_messages`: allows to use custom message for dispatching actions to `go_turtle` node from `contextrecognition` node.

A fifth set of tools, namely T-Rex and the Europa planner binaries, is required for the latter half of the tutorial.  Installation instructions can be found [here](http://code.google.com/p/trex2-agent/wiki/LuciaInstall).  The installation instructions below should be completed before installing Europa/T-Rex.

_Note: in the following, let `<CATKIN_WS>` and `<ROSJAVA_WS>` indicate your catkin and rosjava workspaces, respectively.  If you followed the ROS tutorials, these are probably `~/catkin_ws` and `~/rosjava`.  Remember that both these workspaces must be included in the `ROS_PACKAGE_PATH` environment variable.  It is a good idea to add this to your `~/.bashrc`:_

```
export ROS_PACKAGE_PATH=<CATKIN_WS>:<ROSJAVA_WS>:$ROS_PACKAGE_PATH
```

## Pre-requisites ##

You should have already followed the ROS installation guides provided on the school website.  In short, this means that you have:

  * installed ROS Hydro ([tutorial](http://wiki.ros.org/ROS/Tutorials)) and ROSJava ([tutorial](http://wiki.ros.org/rosjava))

  * installed Turtlebot simulation support ([tutorial](http://wiki.ros.org/turtlebot_simulator))

  * installed and tried the pre-installed virtual-box image for the tutorial on KRR ([instructions](https://blogs.oracle.com/oswald/entry/importing_a_vdi_in_virtualbox), [links to image](http://www.dis.uniroma1.it/~gemignani/krr.html))


## Installing `go_turtle` ##

Enter catkin workspace source directory:

```
$> cd <CATKIN_WS>/src
```

Download the code:

```
$> svn co http://lucia-winter-school-2013.googlecode.com/svn/trunk/go_turtle
```

Build the package:

```
$> cd ..
$> catkin_make
```

Don't forget to source `setup.sh`:

```
$> source devel/setup.sh
```

## Installing `constraints-playground` ##

Enter directory of your choice (this is not a ROS package):

```
$> cd ~/my_workspace
```

Download the code:

```
$> svn co http://lucia-winter-school-2013.googlecode.com/svn/trunk/constraints-playground
```

Build and test the code:

```
$> cd constraints-playground
$> ./gradlew install
$> ./gradlew run
```

Optionally, make an Eclipse project:

```
$> ./gradlew eclipse
```

This project uses the Meta-CSP Framework API version 1.0.217.  The documentation (Javadoc) can be found [here](http://meta-csp-framework.googlecode.com/svn/javadoc/index.html) (or in Eclipse tooltips).

## Installing `myjava_messages` ##

Enter rosjava workspace source directory:

```
$> cd <ROSJAVA_WS>/src
```

Download the code:

```
$> svn co http://lucia-winter-school-2013.googlecode.com/svn/trunk/myjava_messages
```

Build the package:

```
$> cd ..
$> catkin_make
```

You may get the following error, which you should just ignore:

```
FAILURE: Build failed with an exception.

* What went wrong:
Could not copy MANIFEST.MF to '<ROSJAVA_WS>/src/myjava_messages/go_turtle/build/tmp/jar/MANIFEST.MF'.

* Try:
Run with --stacktrace option to get the stack trace. Run with --info or --debug option to get more log output.

BUILD FAILED

Total time: 6.744 secs
make[2]: *** [myjava_messages/CMakeFiles/gradle-myjava_messages] Error 1
make[1]: *** [myjava_messages/CMakeFiles/gradle-myjava_messages.dir/all] Error 2
make: *** [all] Error 2
Invoking "make" failed
```

Finally, build the Java resources:

```
$> cd src/myjava_messages
$> ./gradlew install
```

## Installing `lucia` ##

Enter rosjava workspace source directory:

```
$> cd <ROSJAVA_WS>/src
```

Download the code:

```
$> svn co http://lucia-winter-school-2013.googlecode.com/svn/trunk/lucia
```

Build the package:

```
$> cd ..
$> catkin_make
```

You may get an error as above - again, just ignore it!

Finally, build the Java resources:

```
$> cd src/lucia
$> ./gradlew installApp
```

This package also provides a launch file (located in `src/lucia/launch`) to run the ROS node you just compiled:

```
$> roslaunch lucia contextrecognition.launch
```

Optionally, make an Eclipse project (note that you will not need to modify code here, but you may want to see it):

```
$> ./gradlew eclipse
```
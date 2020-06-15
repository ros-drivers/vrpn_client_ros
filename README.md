# vrpn_client_ros
This fork aims at porting the original code from the kinetic-devel branch to ROS2.

## Requirements

The code requires VRPN to work. Unfortunately, currently the package is only
available in melodic and older ROS releases and can be installed with:
```
    apt-get install ros-melodic-vrpn
```

If you want to stick to ROS2 or do not have access to melodic packages you can
clone the following repository to the src folder of your ros_overlay_ws:
```
git clone --single-branch --branch debian/melodic/vrpn git@github.com:ros-drivers-gbp/vrpn-release.git
```

## What works?

I only use pose in my project, so I did not port anything else (TF, twist, accel). Also multiple sensors per tracker are not ported.
If there is anyone who would like to use the other features and is willing to test them, I'd be happy to help.

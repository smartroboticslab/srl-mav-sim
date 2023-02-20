# Simulators for SRL MAVs

Make sure to clone all submodules either by passing the `--recurse-submodules`
flag to `git clone` or by running `git submodule update --init --recursive`
after cloning.

## Dependencies

TODO

``` sh
pip3 install --user kconfiglib
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
# + ROS stuff
# TODO
```

## TODO

* Allow switching to offboard mode without a joystick (e.g. through
  QGroundControl).
* Add test scripts for the linear and non-linear MPCs.
* Modify the airframes according to our MAVs.
* Create an SDF file for our MAV?

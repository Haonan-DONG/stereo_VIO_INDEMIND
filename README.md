# stereo_VIO_INDEMIND

## for indemind sdk
```shell
# compile
## compile the demo sdk
cd demo
mkdir build && cd build
cmake ..   # warning: the sdk currently is only can be compiled with opencv331
make -j8

## compule the ros wrapper
cd ros
catkin_make -j8


# RUN THE DEMO
## ros as sudo
sudo su
source devel/setup.bash
roslaunch imsee_ros_wrapper start.launch/display.launch
```



## TODO
- [X] add adaptived indemind sdk
- [ ] suit OpenVINS.
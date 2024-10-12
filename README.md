# stereo_VIO_INDEMIND

## Indemind sdk
```shell
# compile
## compile the demo sdk
cd demo
mkdir build && cd build
cmake ..   # warning: the sdk currently is only can be compiled with opencv331
make -j8

## compile the ros wrapper
cd ros
catkin_make -j8


# RUN THE DEMO
## ros as sudo
sudo su
source devel/setup.bash
roslaunch imsee_ros_wrapper start.launch/display.launch
```

## Compile the openvins
```
cd openvins_ws
# ros compile, note that the basic environment is different. Therefore, please follow https://docs.openvins.com/gs-installing.html.
catkin_make -j8

# RUN the demo for the EUROC dataset.
## term 0
roscore
## term 1
source devel/setup.bash
roslaunch ov_msckf subscribe.launch config:=euroc_mav
## term 2
rosbag play V1_01_easy.bag
```

## Dataset
For convenience, we upload the test data for china mainland users in Baidu Disk.
### EUROC Dataset 
[V1_01_easy.bag](https://pan.baidu.com/s/1ng2MAm9Na1cZ7kutDOM8FQ?pwd=67yi). Code: 67yi 


## TODO
- [X] add adapted indemind sdk
- [X] suit OpenVINS for indemind sdk.
    - [X] Upload original OpenVINS code
    - [X] Upload datasets.
- [ ] refactor openvins: ros wrapper
    - [ ] ros wrapper.
- [ ] refactor openvins: algorithm core
    - [ ] image feature.
- [ ] refactor openvins: visualization
    - [ ] OpenGL suit
- [ ] refactor openvins: mobile platform
    - [ ] RK platform suit.
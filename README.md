# sky_vision
Skyrats computer vision repository

## Build
Put this repo inside sky_ws src folder, then build:
```
cd sky_ws
catkin build
```

Currently working:

## Aruco pose estimator
The aruco detector and pose estimator is writen in the python script.
To run the detection:
```
roslaunch sky_vision aruco.launch
```

The image from the camera will be processed and the new image with the aruco will be published in the ros topic:
```
camera/colour/image_new
```

## TODO
Create a structure for the vision features, example, put all information about the aruco detected in different ros topics:

```
vision/aruco/image
vision/aruco/id
vision/aruco/pose/x
vision/aruco/pose/y
vision/aruco/pose/z
vision/aruco/pose/ang_x
vision/aruco/pose/ang_y
```

For other applications:
```
vision/qrcode/image
vision/qrcode/payload
... etc
```

```
vision/cbrbase/image
vision/cbrbase/type
... etc
```

```
vision/barcode/image
vision/barcode/payload
... etc
```

# WebRTC Teleoperation

This project is a WebRTC teleoperation application that allows a user to control a robot remotely using a web browser.

For more information about project structure and components refer to [docs](docs/index.md).

For more information about changes and new features see [CHANGELOG.md](CHANGELOG.md).

## Dependencies

- [nlohmann JSON](https://github.com/nlohmann/json)
- [libdatachannel](https://github.com/paullouisageneau/libdatachannel)

## Running on your PC

## Build

Build the PC client using CMake:

```
$ cd pc_client
$ cmake -B build
$ cd build
$ make -j2
```

## Run

First, start the signaling server for establishing the WebRTC connection between your PC client and the web app:

```
$ python signaling/signaling-server.py
```

In separate terminal, start the web app:

```
$ cd web_client
$ python3 -m http.server --bind 0.0.0.0 8080
```

Now, in seperate terminal, start the PC client:

```
$ cd pc_client/build
$ ./libdatachannel_app
``` 

The PC client and the web app should now be connected to the signaling server. To establish a peer connection between
them, go to http://localhost:8080/ and enter auth (by default set to 'auth0') and remote id (by default set to '
origin-1') and press 'Offer'.
The peer connection should be established and you should be able to see a video stream from your PC in the web app.

(Optional) If you want to use a video from gstreamer, change main.cpp file in pc_client/src and start the RTP h264 video
stream with payload type 96
on `localhost:6000`.

On Linux, use the following gstreamer demo pipeline to capture video from a V4L2 webcam and send it as RTP to port
6000 (You might need to
change `/dev/video0` to your actual device):

```
$ gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480 ! videoconvert ! queue ! x264enc tune=zerolatency bitrate=1000 key-int-max=30 ! video/x-h264, profile=constrained-baseline ! rtph264pay pt=96 mtu=1200 ! udpsink host=127.0.0.1 port=6000
```

On Mac, use the following GStreamer demo pipeline to capture video from an AVFoundation video source and send it as RTP
to port 6000:

```
$ gst-launch-1.0 avfvideosrc device-index=0 ! video/x-raw,width=640,height=480 ! videoconvert ! queue ! x264enc tune=zerolatency bitrate=1000 key-int-max=30 ! video/x-h264,profile=constrained-baseline ! rtph264pay pt=96 mtu=1200 ! udpsink host=127.0.0.1 port=6000
```

## Running on Origin

## Build

Place the origin_webrtc_teleop folder inside the src directory of the workspace that is located on the origin.

Navigate to the workspace directory and build the package using the following command (don't forget to source the
workspace
first):

```
$ colcon build --packages-select origin_webrtc_teleop
```

## Run

Start the signaling server for establishing the WebRTC connection between the origin and the web app:

```
$ python signaling/signaling-server.py
```

In separate terminal, start the web app:

```
$ cd web_client
$ python3 -m http.server --bind 0.0.0.0 8080
```

On the origin navigate to the workspace and set up the environment:

```
$ . install/setup.bash
```

Then, start the teleoperation node:

```
$ ros2 run origin_webrtc_teleop origin_webrtc_teleop
```

## To-Do

- [ ] place the teleop_client library outside the origin_webrtc_teleop package
- [ ] refactor the PeerConnection class to reduce the number of parameters required for construction. Consider splitting
  it into multiple classes.
- [ ] fix web app to show every kind of message received
- [ ] add ImageEncode for encoding ros image topics
- [ ] add video stream from the origin to the web app
- [ ] send control messages from the web app to the origin for controlling the robot
- [ ] configure project to run in separate docker container
- [ ] add a launch file for the origin_webrtc_teleop package
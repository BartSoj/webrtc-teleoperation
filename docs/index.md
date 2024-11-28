# WebRTC Teleop

## Compoenents:

### teleop client lib

inside `teleop_client` folder the classes for handling the WebRTC connection are provided.
PeerConnections represents a single webrtc connection with a client.
Teleoperation object that is responsible for handling the peer connections and sending the data and video stream to the
web client as well as receiving the control data.
VideoEncoder object that is responsible for encoding the video stream that is passed from the robot camera to the web
client.

### origin webrtc teleop

contains a ros node that allows for teleoperation of a robot using WebRTC. In the config folder the configuration file
for teleoperation and video encoder is located. The teleoperation node read the configuration file, sets up the
teleoperation and video encoder. It subscribes to the ros topic to get the camera images and other telemetry data and
sends them using the teleoperation object. It also, publishes the control data received from the teleoperation object.

### Signaling

inside `signaling` folder a signaling server is provided. It is a simple python websocket server that forwards messages
between clients. It's only responsible for exchanging the SDP offer/answer and ICE candidates between clients required
to establish a WebRTC connection.

### web client

inside `web_client` folder a web application for teleoperation is provided. It uses WebRTC to transfer data to
and from the other client. It allows the user to view the video stream form robot camera and
additional telemetry data as well as control the robot using a gamepad.

### PC client

provides a way to test the teleoperation on your PC. `teleop_actions` folder contains implementations of different
functionalities that can be used with teleoperation. SendCounterAction is the simplest, it sends a message with the
counter that increases every second to the web client. StreamVideoAction sends a stream from your pc camera to the web
client. AddClientAction allows for adding a new client inside the terminal. Note: the current implementation of PC
client is not working.

## Other resources:

Component diagrams can be found [here](media/component-diagram.png)

For more information about the project, see [README.md](../README.md)

For performance and testing, see [testing.md](testing.md)
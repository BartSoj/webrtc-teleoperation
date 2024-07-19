# WebRTC Teleop

## origin webrtc teleop

contains a ros node that allows for teleoperation of a robot using WebRTC. Inside `teleop_client` folder the
Teleoperation and PeerConnection class is located. The node uses Teleoperation object that is responsible for handling
the WebRTC connection and sending the data and video stream to the web client as well as receiving the control data.
PeerConnections represents a single webrtc connection with a client.

## PC client

provides a way to test the teleoperation on your PC. `teleop_actions` folder contains implementations of different
functionalities that can be used with teleoperation. SendCounterAction is the simplest, it sends a message with the
counter that increases every second to the web client. StreamVideoAction sends a stream from your pc camera to the web
client. AddClientAction allows for adding a new client inside the terminal.

## Signaling

inside `signaling` folder a signaling server is provided. It is a simple python websocket server that forwards messages
between clients. It's only responsible for exchanging the SDP offer/answer and ICE candidates between clients required
to establish a WebRTC connection.

## web client

inside `web_client` folder a minimal web application for teleoperation is provided. It uses WebRTC to transfer data to
and from the other client.
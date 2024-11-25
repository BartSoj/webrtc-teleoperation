# Teleop Web Client

This is the web client for the robot teleoperation. It allows the user to view the video stream form robot camera and
additional telemetry data as well as control the robot using a gamepad.

## Running the website

```bash
npm run dev -- --host
```

## Using the website

- Enter the authentication and id for the robot. It can be webrtc_teleop package configuration.
- Choose observe to view the video stream and telemetry data.
- Choose control to view the video stream, telemetry data and control the robot using a gamepad.
  ![img.png](img.png)
# WebRTC Teleop Latency Test for Robot Control

Test to measure the delay between the video capture and its display on the web app of the USB camera without the ROS
network.

## Testing Procedure

The smartphone displayed a running timer with millisecond precision.
The pc's camera captured the timer display.
The WebRTC stream was received and displayed on the receiving device.
15-second test video of both the original timer and the received video stream.
Five screenshots were taken at various points during the test.
Network speeds were measured for both the PC and the smartphone.

## Results

Latency Measurement
Average Latency: Approximately 90ms. The delay remained relatively consistent.
![latency_measurement](media/img.png)
![latency_measurement](media/img_1.png)
![latency_measurement](media/img_2.png)
![latency_measurement](media/img_3.png)
![latency_measurement](media/img_4.png)

## Network Speeds

### PC network speed: ![PC network speed](media/pc_network.png)

### Smartphone network speed: ![Smartphone network speed](media/phone_network.jpeg)

Test to measure the delay between the video capture and its display on the web app using the camera on the robot with
the ROS network.

## Testing Procedure

The PC displayed a running timer with millisecond precision.
The robot's camera captured the timer display.
The WebRTC stream was received and displayed on the PC.
15-second test video of both the original timer and the received video stream.
Five screenshots were taken at various points during the test.

## Results

Latency Measurement
Average Latency: Approximately 150ms. The delay remained relatively consistent.
![img_5.png](media/img_5.png)
![img_6.png](media/img_6.png)
![img_7.png](media/img_7.png)
![img_8.png](media/img_8.png)
![img_9.png](media/img_9.png)

## Implications for Robot Control

A 150ms latency in the video feed allows for:

- Near real-time visual feedback
- Responsive robot control in most scenarios
- Effective teleoperation for tasks that don't require split-second timing

## Future Work

- Extended duration tests
- Various network condition testing
- Tests with video stram from Origin
- Integrated latency monitoring
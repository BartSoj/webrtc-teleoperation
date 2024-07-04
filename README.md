# Robot control app using libdatachannel

## Build

```git submodule update --init --recursive --depth 1```
```cmake -B build```
```cd build```
```make -j2``

## Run

```gst-launch-1.0 avfvideosrc device-index=0 ! video/x-raw,width=640,height=480 ! videoconvert ! queue ! x264enc tune=zerolatency bitrate=1000 key-int-max=30 ! video/x-h264,profile=constrained-baseline ! rtph264pay pt=96 mtu=1200 ! udpsink host=127.0.0.1 port=6000```

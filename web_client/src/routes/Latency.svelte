<script lang="ts">
    interface LatencyInfo {
        tR1: number;
        sender_local_clock: number;
    }

    let latency_list: number[] = [];
    const MAX_VALUES = 6;

    let latency: number = 0;

    export let sendMessage: (msg: string) => void;

    setInterval(() => {
        const tR1 = Math.trunc(performance.now());
        const message = JSON.stringify({
            type: "latency",
            latency: {tR1: tR1}
        });
        sendMessage(message);
    }, 2000); // Sends every 2 seconds

    export function updateLatencyInfo(newLatencyInfo: LatencyInfo, videoElement: HTMLVideoElement) {
        // Latency Calculation
        const tR2 = performance.now(); // Receiver's current time
        const {tR1, sender_local_clock} = newLatencyInfo;

        // Round Trip Time (RTT) and Network Latency
        const rtt = tR2 - tR1;
        const networkLatency = rtt / 2;
        let senderTime = sender_local_clock + networkLatency;

        videoElement.requestVideoFrameCallback((now: DOMHighResTimeStamp, framemeta: VideoFrameCallbackMetadata) => {
            if (framemeta.rtpTimestamp !== undefined) {
                const delaySinceVideoCallbackRequested = now - tR2;
                senderTime += delaySinceVideoCallbackRequested;

                // Calculate expected and actual video times
                const expectedVideoTimeMsec = senderTime;
                const actualVideoTimeMsec = Math.trunc(framemeta.rtpTimestamp / 90); // Convert RTP timestamp to milliseconds

                // Compute and update latency
                const latencyVal = expectedVideoTimeMsec - actualVideoTimeMsec;
                updateLatency(latencyVal);
            } else {
                console.log('RTP timestamp is not available for this frame.');
            }
        });
    }

    function updateLatency(newLatency: number) {
        latency_list.push(newLatency);

        if (latency_list.length > MAX_VALUES) {
            latency_list.shift();
        }

        latency = Math.max(...latency_list);
    }
</script>

<div class="latency-info">
    <h2>Latency</h2>
    <p>{latency.toFixed(0)} ms</p>
</div>
<script lang="ts">
    interface LatencyInfo {
        latency: number;
    }

    let latencies: number[] = [];
    const MAX_VALUES = 6;

    let latencyInfo: LatencyInfo = {
        latency: 0
    };

    export function updateLatencyInfo(newLatencyInfo: LatencyInfo) {
        latencies.push(newLatencyInfo.latency);

        if (latencies.length > MAX_VALUES) {
            latencies.shift();
        }

        const maxLatency = Math.max(...latencies);

        latencyInfo = {latency: maxLatency};
    }
</script>

<div class="latency-info">
    <h2>Latency</h2>
    <p>{latencyInfo.latency.toFixed(0)} ms</p>
</div>
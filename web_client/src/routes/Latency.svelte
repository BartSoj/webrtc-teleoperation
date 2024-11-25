<script lang="ts">
    interface LatencyInfo {
        latency: number;
    }

    let latencyInfo: LatencyInfo = {
        latency: 0
    };

    export function updateLatencyInfo(newLatencyInfo: LatencyInfo) {
        latencyInfo = newLatencyInfo;
    }
</script>

<div class="latency-info">
    <h2>Latency</h2>
    <p>{latencyInfo.latency.toFixed(0)} ms</p>
</div>
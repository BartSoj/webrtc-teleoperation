<script lang="ts">
    import {onMount} from 'svelte';
    import {messageStore} from "$lib/teleoperationStore";

    export let id: string;

    interface NetworkTelemetry {
        cellular_strength: number;
        wifi_strength: number;
    }

    let networkInfo: NetworkTelemetry = {
        cellular_strength: -1,
        wifi_strength: -1
    };

    export function updateNetworkInfo(newNetworkInfo: NetworkTelemetry) {
        networkInfo = newNetworkInfo;
    }

    onMount(() => {
        messageStore.subscribe(({id: messageId, message}) => {
            if (id !== messageId) return;
            const data = JSON.parse(message || "{}");
            if (data?.type === 'network') updateNetworkInfo(data.network);
        });
    });

    function getIcon(strength: number): string {
        switch (strength) {
            case 3:
                return "/icons/connection/connection-three.png"; // Excellent
            case 2:
                return "/icons/connection/connection-two.png"; // Fair
            case 1:
                return "/icons/connection/connection-one.png"; // Poor
            case 0:
                return "/icons/connection/connection-zero.png"; // Disconnected
            default:
                return "/icons/connection/connection-zero.png"; // Unknown
        }
    }
</script>

<div id="network-info">
    <h2>Network</h2>
    {#if networkInfo.wifi_strength === -1 && networkInfo.cellular_strength === -1}
        <p>Unknown</p>
    {:else}
        {#if networkInfo.wifi_strength !== -1}
            <div style="display: flex; align-items: center; gap: 8px;">
                <p>WiFi:</p>
                <img src="{getIcon(networkInfo.wifi_strength)}"
                     alt="wifi-icon"
                     style="width: 24px; height: 24px; filter: invert(1)">
            </div>
        {/if}
        {#if networkInfo.cellular_strength !== -1}
            <div style="display: flex; align-items: center; gap: 8px;">
                <p>Cellular:</p>
                <img src="{getIcon(networkInfo.cellular_strength)}"
                     alt="cellular-icon"
                     style="width: 24px; height: 24px; filter: invert(1)">
            </div>
        {/if}
    {/if}
</div>
<script lang="ts">
    import {messageStore} from "$lib/teleoperationStore";
    import {onMount} from "svelte";

    export let id: string;

    interface OdometryData {
        position: { x: number; y: number; z: number };
        orientation: { x: number; y: number; z: number; w: number };
        linear: { x: number; y: number; z: number };
        angular: { x: number; y: number; z: number };
    }

    let odometry: OdometryData = {
        position: {x: 0, y: 0, z: 0},
        orientation: {x: 0, y: 0, z: 0, w: 0},
        linear: {x: 0, y: 0, z: 0},
        angular: {x: 0, y: 0, z: 0},
    };

    export function updateOdometryData(newOdometry: OdometryData) {
        odometry = newOdometry;
    }

    onMount(() => {
        messageStore.subscribe(({id: messageId, message}) => {
            if (id !== messageId) return;
            const data = JSON.parse(message || "{}");
            if (data?.type === 'odometry') updateOdometryData(data.odometry);
        });
    });
</script>

<div id="odometry">
    <h2>Location</h2>
    <p>heading: {odometry.orientation.z.toFixed(2)}</p>
    <p>x: {odometry.position.x.toFixed(2)}</p>
    <p>y: {odometry.position.y.toFixed(2)}</p>
    <h2>Speed</h2>
    <p>Linear: {odometry.linear.x.toFixed(2)}</p>
    <p>Angular: {odometry.angular.z.toFixed(2)}</p>
</div>
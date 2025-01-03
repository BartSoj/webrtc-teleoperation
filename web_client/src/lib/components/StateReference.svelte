<script lang="ts">
    import {messageStore} from "$lib/teleoperationStore";
    import {onMount} from "svelte";

    export let id: string;

    interface StateReferenceData {
        pose: {
            position: { x: number; y: number; z: number };
            orientation: { x: number; y: number; z: number; w: number };
        };
        velocity: { x: number; y: number; z: number };
        acceleration: { x: number; y: number; z: number };
    }

    let stateReference: StateReferenceData = {
        pose: {
            position: {x: 0, y: 0, z: 0},
            orientation: {x: 0, y: 0, z: 0, w: 0},
        },
        velocity: {x: 0, y: 0, z: 0},
        acceleration: {x: 0, y: 0, z: 0},
    };

    export function updateStateReferenceData(newStateReference: StateReferenceData) {
        stateReference = newStateReference;
    }

    onMount(() => {
        messageStore.subscribe(({id: messageId, message}) => {
            if (id !== messageId) return;
            const data = JSON.parse(message || "{}");
            if (data?.type === "state_reference") updateStateReferenceData(data.state_reference);
        });
    });
</script>

<div id="state-reference">
    <h2>Pose</h2>
    <p>x: {stateReference.pose.position.x.toFixed(2)} </p>
    <p>y: {stateReference.pose.position.y.toFixed(2)} </p>
    <p>z: {stateReference.pose.position.z.toFixed(2)} </p>
    <p>Orientation (heading): {stateReference.pose.orientation.z.toFixed(2)} </p>

    <h2>Velocity</h2>
    <p>x: {stateReference.velocity.x.toFixed(2)}</p>
    <p>y: {stateReference.velocity.y.toFixed(2)}</p>
    <p>z: {stateReference.velocity.z.toFixed(2)}</p>

    <h2>Acceleration</h2>
    <p>x: {stateReference.acceleration.x.toFixed(2)}</p>
    <p>y: {stateReference.acceleration.y.toFixed(2)}</p>
    <p>z: {stateReference.acceleration.z.toFixed(2)}</p>
</div>
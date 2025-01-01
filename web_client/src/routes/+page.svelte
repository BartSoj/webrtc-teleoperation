<script lang="ts">
    import {onMount} from 'svelte';
    import Video from './Video.svelte';
    import Connection from './Connection.svelte';
    import Battery from './Battery.svelte';
    import Logs from './Logs.svelte';
    import Network from './Network.svelte';
    import Odometry from './Odometry.svelte';
    import Messaging from './Messaging.svelte';
    import Latency from './Latency.svelte';
    import Gamepad from './Gamepad.svelte';
    import {
        initializeTeleoperation,
        localId,
        peerConnectionMap
    } from '$lib/teleoperationStore';

    const DEV_MODE = false;

    $: activeId = (() => {
        const keys = Object.keys($peerConnectionMap);
        return keys.length > 0 ? keys[keys.length - 1] : "";
    })();

    let videoElement: HTMLVideoElement;

    onMount(() => {
        initializeTeleoperation();
    });
</script>

<main>
    <Video bind:videoElement videoStream={$peerConnectionMap[activeId]?.videoStream} templateVideoSrc="default.mp4"/>
    <div class="content">
        <div class="box" style="top: 5%; right: 5%;">
            <h2>Local ID</h2>
            <p>{$localId}</p>
            <h2>Active ID</h2>
            <p>{activeId}</p>
        </div>

        {#if DEV_MODE || $peerConnectionMap[activeId]?.state !== 'connected'}
            <h1>Origin Teleop</h1>
            <div class="box" style="top: 40%; left: 50%; transform: translateX(-50%);">
                <Connection/>
            </div>
        {/if}

        {#if DEV_MODE || $peerConnectionMap[activeId]?.state === 'connected'}
            <div class="box" style="top: 5%; left: 5%;">
                <Network id={activeId}/>
            </div>
            <div class="box" style="top: 30%; left: 5%;">
                <Odometry id={activeId}/>
            </div>
            <div class="box" style="bottom: 15%; left: 5%;">
                <Battery id={activeId}/>
            </div>

            {#if $peerConnectionMap[activeId]?.enableMessaging}
                <div class="box" style="bottom: 15%; left: 50%; transform: translateX(-50%)">
                    <Messaging id={activeId}/>
                </div>
                <div class="box" style="top: 30%; right: 5%;">
                    <Logs id={activeId}/>
                </div>
            {/if}
            <div class="box" style="bottom: 15%; right: 5%;">
                <Latency id={activeId} {videoElement}/>
            </div>
        {/if}

        {#if DEV_MODE || $peerConnectionMap[activeId]?.access === 'control'}
            <div class="box" style="top: 5%; left: 50%; transform: translateX(-50%)">
                <Gamepad id={activeId}/>
            </div>
        {/if}
    </div>
</main>

<style>
    :global(body) {
        margin: 0;
        padding: 0;
        overflow: hidden;
        background-color: black;
        user-select: none;
    }

    main {
        position: relative;
        width: 100vw;
        height: 100vh;
        overflow: hidden;
    }

    .content {
        position: relative;
        z-index: 1;
        width: 100%;
        height: 100%;
        font-family: arial, verdana, sans-serif;
        font-size: 12pt;
        color: #FFFFFF;
    }

    h1 {
        text-align: center;
        margin: 0;
        font-size: 3.5em;
        text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.7);
        position: relative;
        top: 15%;
    }

    .box {
        position: absolute;
        background-color: rgba(0, 0, 0, 0.7);
        border-radius: 10px;
        padding: 0.5em 1em 0.5em 1em;
        max-width: 300px;
        width: max-content;
    }

    :global(.box h2) {
        font-size: 1em;
        margin-top: 0.5em;
        margin-bottom: 0.5em;
    }

    :global(.box p) {
        margin: 0.3em 0;
        font-size: 0.8em;
    }

    :global(input, select) {
        margin-bottom: 0.5em;
        padding: 0.3em;
        width: 100%;
        box-sizing: border-box;
    }

    :global(.box label) {
        margin: 0.5em 0;
        font-size: 0.8em;
    }

</style>

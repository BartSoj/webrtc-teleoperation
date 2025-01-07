<script lang="ts">
    import {onMount} from 'svelte';
    import CreateConnection from "$lib/components/CreateConnection.svelte";
    import SetActiveConnection from "$lib/components/SetActiveConnection.svelte";
    import Battery from "$lib/components/Battery.svelte";
    import Plot from "$lib/components/Plot.svelte";
    import {
        initializeTeleoperation,
        localId,
        peerConnectionMap
    } from '$lib/teleoperationStore';
    import SetShapeExample from "$lib/components/SetShapeExample.svelte";
    import StateReference from "$lib/components/StateReference.svelte";

    const DEV_MODE = false;

    let showPlot = false;

    $: showCreateConnection = (() => {
        return activeId === "";
    })();

    let newActiveId = "";
    $: activeId = (() => {
        if (newActiveId) {
            const id = newActiveId;
            newActiveId = "";
            return id;
        }
        const keys = Object.keys($peerConnectionMap);
        return keys.length > 0 ? keys[keys.length - 1] : "";
    })();

    onMount(() => {
        initializeTeleoperation();
    });
</script>

<main>
    <div class="content">
        <div class="box" style="top: 5%; right: 5%;">
            <p>Local ID: {$localId}</p>
            <p>Active ID: {activeId}</p>
        </div>

        {#if DEV_MODE || showCreateConnection}
            <h1>Vertex Teleop</h1>
            <div class="box" style="top: 40%; left: 50%; transform: translateX(-50%);">
                <CreateConnection/>
            </div>
        {/if}

        {#if DEV_MODE || Object.keys($peerConnectionMap).length > 0}
            <div class="box" style="top: 5%; left: 5%;">
                <h2>Plot</h2>
                <button on:click={() => showPlot=!showPlot}>
                    {showPlot ? 'Hide' : 'Show'}
                </button>
            </div>
            <div class="box" style="bottom: 5%; right: 5%;">
                <h2>Connection</h2>
                <div class="button-container">
                    <SetActiveConnection bind:newActiveId/>
                    <div>
                        <button on:click={() => showCreateConnection=!showCreateConnection}>
                            {showCreateConnection ? 'Discard' : 'Add'}
                        </button>
                    </div>
                </div>
            </div>
            {#if showPlot}
                <div class="box" style="top: 50%; left: 50%; transform: translate(-50%, -50%);">
                    <Plot/>
                </div>
            {/if}
        {/if}

        {#if DEV_MODE || $peerConnectionMap[activeId]?.state !== 'connected' && !showCreateConnection}
            <div class="box" style="top: 40%; left: 50%; transform: translateX(-50%);">
                <h2>Connecting...</h2>
            </div>
        {/if}

        {#if DEV_MODE || $peerConnectionMap[activeId]?.state === 'connected' && !showCreateConnection}
            <div class="box" style="top: 30%; left: 5%;">
                <StateReference id={activeId}/>
            </div>
            <div class="box" style="bottom: 15%; left: 5%;">
                <Battery id={activeId}/>
            </div>
            {#if DEV_MODE || $peerConnectionMap[activeId]?.access === 'control'}
                <div class="box" style="bottom: 15%; left: 50%; transform: translateX(-50%);">
                    <h2>Shape</h2>
                    <SetShapeExample id={activeId}/>
                </div>
            {/if}
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
        padding: 1em 1em 1em 1em;
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

    :global(.box .button-container) {
        display: flex;
        justify-content: center;
        align-items: center;
        gap: 12px; /* Space between buttons */
    }

    :global(.box button) {
        width: 60px;
        height: 60px;
        border-radius: 50%;
        border: none;
        cursor: pointer;
        font-size: 0.8em;
        transition: background-color 0.3s ease-in-out, transform 0.2s ease-in-out; /* Adds hover and press effects */
        box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1); /* Optional shadow for a modern look */

    }

    :global(.box button:active) {
        transform: scale(0.95);
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

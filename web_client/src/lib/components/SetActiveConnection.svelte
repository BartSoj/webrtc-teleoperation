<script lang="ts">
    import {peerConnectionMap} from '$lib/teleoperationStore';

    export let newActiveId: string = "";

    function handleSendClick(id: string) {
        newActiveId = id;
    }

    let areButtonsVisible = false;

    function toggleButtons() {
        areButtonsVisible = !areButtonsVisible;
    }
</script>

<div id="setActiveConnection" class="button-container">
    <!-- Buttons group toggled by visibility -->
    <div style={areButtonsVisible ? "" : "display: none"}>
        {#each Object.keys($peerConnectionMap) as id}
            <button class="circle-button" on:click={() => handleSendClick(id)}>{id}</button> <!-- Add explicit space -->
        {/each}
    </div>

    <!-- Expand/Collapse Button -->
    <button class="circle-button" on:click={toggleButtons}>
        {areButtonsVisible ? "Collapse" : "Expand"}
    </button>
</div>
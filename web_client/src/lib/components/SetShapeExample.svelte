<script lang="ts">
    import {sendMessage} from '$lib/teleoperationStore';

    export let id: string;

    const flat_circle = {
        type: "shape_example",
        shape_json: {
            execution_activated: true,
            type: "flat_circle",
            parameters: {
                radius: 2,
                execution_time: 20
            }
        }
    };

    const vertical_circle = {
        type: "shape_example",
        shape_json: {
            execution_activated: true,
            type: "vertical_circle",
            parameters: {
                radius: 2,
                execution_time: 20
            }
        }
    };

    const stop = {
        type: "shape_example",
        shape_json: {
            execution_activated: false,
        }
    };

    function handleSendClick(service_message: object) {
        $sendMessage(id, JSON.stringify(service_message));
    }

    let areButtonsVisible = false;

    function toggleButtons() {
        areButtonsVisible = !areButtonsVisible;
    }
</script>

<div id="setShapeExample" class="button-container">
    <!-- Buttons group toggled by visibility -->
    <div style={areButtonsVisible ? "" : "display: none"}>
        <button class="circle-button" on:click={() => handleSendClick(flat_circle)}>Flat</button>
        <button class="circle-button" on:click={() => handleSendClick(vertical_circle)}>Vertical</button>
        <button class="circle-button" on:click={() => handleSendClick(stop)}>Stop</button>
    </div>

    <!-- Expand/Collapse Button -->
    <button class="circle-button" on:click={toggleButtons}>
        {areButtonsVisible ? "Collapse" : "Expand"}
    </button>
</div>
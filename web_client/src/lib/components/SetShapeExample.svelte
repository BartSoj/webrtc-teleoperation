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

<div id="setShapeExample">
    <!-- Buttons group toggled by visibility -->
    <div class={areButtonsVisible ? "" : "hidden"}>
        <button class="circle-button" on:click={() => handleSendClick(flat_circle)}>Flat</button>
        <button class="circle-button" on:click={() => handleSendClick(vertical_circle)}>Vertical</button>
        <button class="circle-button" on:click={() => handleSendClick(stop)}>Stop</button>
    </div>

    <!-- Expand/Collapse Button -->
    <button class="circle-button" on:click={toggleButtons}>
        {areButtonsVisible ? "Collapse" : "Expand"}
    </button>
</div>

<style>
    #setShapeExample {
        display: flex;
        justify-content: center;
        align-items: center;
        gap: 12px; /* Space between buttons */
    }

    .circle-button {
        width: 60px; /* Width of the circle button */
        height: 60px; /* Height of the circle button */
        border-radius: 50%; /* Makes the button a circle */
        border: none;
        cursor: pointer;
        transition: background-color 0.3s ease-in-out, transform 0.2s ease-in-out; /* Adds hover and press effects */
        box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1); /* Optional shadow for a modern look */
        margin: 0.3em 0;
        font-size: 0.8em;
    }

    .circle-button:active {
        transform: scale(0.95);
    }

    .hidden {
        display: none; /* Hide the element */
    }
</style>
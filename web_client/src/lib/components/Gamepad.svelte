<script lang="ts">
    import {onMount} from "svelte";
    import {sendControlMsg} from "$lib/teleoperationStore";

    export let id: string;

    let controlMsg: any = null;

    function handleGamepadConnected(event: GamepadEvent) {
        console.log("Gamepad connected:", event.gamepad.id);
    }

    function handleGamepadDisconnected(event: GamepadEvent) {
        console.log("Gamepad disconnected:", event.gamepad.id);
        if (controlMsg && controlMsg.id === event.gamepad.id) {
            controlMsg = null;
        }
    }

    function updateGamepadState() {
        const gamepads = navigator.getGamepads();
        for (let gamepad of gamepads) {
            if (gamepad &&
                (!controlMsg ||
                    controlMsg.id === gamepad.id ||
                    gamepad.buttons[3].pressed)) {
                controlMsg = {
                    type: "control",
                    id: gamepad.id,
                    request_control: gamepad.buttons[0].pressed,
                    reset_control: gamepad.buttons[1].pressed,
                    previous_control: gamepad.buttons[2].pressed,
                    forward: Math.abs(gamepad.axes[1]) >= 0.1 ? -gamepad.axes[1] : 0,
                    rotation: Math.abs(gamepad.axes[2]) >= 0.1 ? -gamepad.axes[2] : 0
                };
                $sendControlMsg(id, JSON.stringify(controlMsg));
            }
        }

        requestAnimationFrame(updateGamepadState);
    }

    onMount(() => {
        window.addEventListener("gamepadconnected", handleGamepadConnected);
        window.addEventListener("gamepaddisconnected", handleGamepadDisconnected);

        updateGamepadState();

        return () => {
            window.removeEventListener("gamepadconnected", handleGamepadConnected);
            window.removeEventListener("gamepaddisconnected", handleGamepadDisconnected);
        };
    });
</script>

<div id="gamepad">
    {#if controlMsg === null}
        <p>No gamepads connected</p>
    {:else}
        <p>Active {controlMsg.id}</p>
    {/if}
</div>
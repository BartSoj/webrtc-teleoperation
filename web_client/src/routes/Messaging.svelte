<script lang="ts">
    import {sendMessage} from '$lib/teleoperationStore';

    export let id: string;

    let message: string = '';

    function handleSendClick() {
        if (message === '') {
            return;
        }
        const log_message = {
            type: "log",
            log: {
                message: message,
                level: "0"
            }
        };
        $sendMessage(id, JSON.stringify(log_message));
        message = '';
    }
</script>

<div id="messaging">
    <h2>Send a message</h2>
    <input type="text" id="sendMsg" placeholder="message" bind:value={message}/>
    <input type="button" id="sendBtn" value="Send" on:click={handleSendClick}/>
</div>
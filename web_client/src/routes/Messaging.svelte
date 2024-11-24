<script lang="ts">
    let message: string = '';

    export let sendMessage: (msg: string) => void;

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
        sendMessage(JSON.stringify(log_message));
        message = '';
    }
</script>

<div id="messaging">
    <h2>Send a message</h2>
    <input type="text" id="sendMsg" placeholder="message" bind:value={message}/>
    <input type="button" id="sendBtn" value="Send" on:click={handleSendClick}/>
</div>
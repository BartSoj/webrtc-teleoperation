<script lang="ts">
    import {messageStore} from "$lib/teleoperationStore";
    import {onMount} from "svelte";

    export let id: string;

    interface LogMessage {
        message: string;
        level: string;
    }

    let logMessages: LogMessage[] = [];

    export function updateLogMessages(newLogMessage: LogMessage) {
        logMessages = [...logMessages, newLogMessage];
    }

    function autoScroll(node: HTMLElement) {
        const scroll = () => {
            node.scrollTop = node.scrollHeight;
        };

        const observer = new MutationObserver(scroll);
        observer.observe(node, {childList: true, subtree: true});

        return {
            destroy() {
                observer.disconnect();
            }
        };
    }

    onMount(() => {
        messageStore.subscribe(({id: messageId, message}) => {
            if (id !== messageId) return;
            const data = JSON.parse(message || "{}");
            if (data?.type === 'log') updateLogMessages(data.log);
        });
    });
</script>

<div id="logs">
    <h2>Messages</h2>
    <div id="log-messages" use:autoScroll>
        {#each logMessages as log}
            <p><strong>{log.level}</strong>: {log.message}</p>
        {/each}
    </div>
</div>

<style>
    #log-messages {
        max-height: 150px;
        overflow-y: auto;
        scrollbar-width: none;
        -ms-overflow-style: none;
    }

    #log-messages::-webkit-scrollbar {
        display: none;
    }
</style>

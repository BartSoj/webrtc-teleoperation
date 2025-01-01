<script lang="ts">
    export let videoStream: MediaStream | undefined;
    export let templateVideoSrc: string;
    export let videoElement: HTMLVideoElement;

    function setSrcObject(node: HTMLVideoElement, stream: MediaStream | undefined) {
        node.src = templateVideoSrc;
        return {
            update(newStream: MediaStream | undefined) {
                if (newStream === undefined) {
                    node.srcObject = null;
                    node.src = templateVideoSrc;
                } else if (node.srcObject !== newStream) {
                    node.srcObject = newStream;
                    node.src = "";
                }
            },
            destroy() {
                node.srcObject = null;
            }
        };
    }
</script>

<div class="video-container">
    <video
            id="video-element"
            muted
            autoplay
            playsinline
            loop
            bind:this={videoElement}
            use:setSrcObject={videoStream}
    >
    </video>
</div>

<style>
    .video-container {
        position: fixed;
        top: 0;
        left: 0;
        width: 100%;
        height: 100%;
        display: flex;
        justify-content: center;
        align-items: center;
        background-color: black;
        z-index: -1;
    }

    video {
        width: 100%;
        height: 100%;
        object-fit: contain;
    }
</style>

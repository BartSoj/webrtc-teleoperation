<script lang="ts">
    import {onMount} from 'svelte';
    import Connection from './Connection.svelte';
    import type {ConnectionInfo} from "$lib";
    import Battery from './Battery.svelte';
    import Logs from './Logs.svelte';
    import Network from './Network.svelte';
    import Odometry from './Odometry.svelte';
    import Messaging from './Messaging.svelte';
    import Latency from './Latency.svelte';
    import Gamepad from './Gamepad.svelte';

    const DEV_MODE: boolean = false;

    let connectionInfo: ConnectionInfo = {
        auth: '',
        access: 'observe',
        offerId: '',
        enableMessaging: false,
    };

    let localId: string = '';

    let battery: Battery | undefined;
    let logs: Logs | undefined;
    let network: Network | undefined;
    let odometry: Odometry | undefined;
    let latency: Latency | undefined;

    let peerConnectionState: string = 'new';
    const templateVideoSrc: string = 'default.mp4';

    let createOfferDisabled: boolean = true;

    let sendMessage: (msg: string) => void;
    let sendControlMsg: (msg: string) => void;
    let handleOfferClick: () => void;

    interface Message {
        id: string;
        type: string;
        description?: string;
        candidate?: string;
        mid?: string;
    }

    onMount(() => {
        const config: RTCConfiguration = {
            iceServers: [{urls: 'stun:stun.l.google.com:19302'}],
        };

        localId = randomId(4);

        const ip = location.hostname;
        const url = `ws://${ip}:8000/${localId}`;

        const peerConnectionMap: { [key: string]: RTCPeerConnection } = {};
        const dataChannelMap: { [key: string]: RTCDataChannel } = {};

        const videoElement = document.getElementById('video-element') as HTMLVideoElement;

        sendMessage = (msg) => {
            for (const dc of Object.values(dataChannelMap)) {
                dc.send(msg);
            }
        }

        sendControlMsg = (msg) => {
            if (connectionInfo.access === 'control') {
                for (const dc of Object.values(dataChannelMap)) {
                    dc.send(msg);
                }
            }
        }

        console.log('Connecting to signaling...');
        openSignaling(url)
            .then((ws) => {
                console.log('WebSocket connected, signaling ready');
                createOfferDisabled = false;
                handleOfferClick = () => offerPeerConnection(ws, connectionInfo.offerId);
            })
            .catch((err) => console.error(err));

        function openSignaling(url: string): Promise<WebSocket> {
            return new Promise((resolve, reject) => {
                const ws = new WebSocket(url);
                ws.onopen = () => resolve(ws);
                ws.onerror = () => reject(new Error('WebSocket error'));
                ws.onclose = () => console.error('WebSocket disconnected');
                ws.onmessage = (e) => {
                    if (typeof e.data != 'string') return;
                    const message: Message = JSON.parse(e.data);
                    console.log(message);
                    const {id, type} = message;

                    let pc = peerConnectionMap[id];
                    if (!pc) {
                        if (type != 'offer') return;

                        // Create PeerConnection for answer
                        console.log(`Answering to ${id}`);
                        pc = createPeerConnection(ws, id);
                    }

                    switch (type) {
                        case 'offer':
                        case 'answer':
                            pc.setRemoteDescription({
                                sdp: message.description!,
                                type: message.type as RTCSdpType,
                            }).then(() => {
                                if (type == 'offer') {
                                    // Send answer
                                    sendLocalDescription(ws, id, pc, 'answer');
                                }
                            });
                            break;

                        case 'candidate':
                            pc.addIceCandidate({
                                candidate: message.candidate!,
                                sdpMid: message.mid!,
                            });
                            break;
                    }
                };
            });
        }

        function offerPeerConnection(ws: WebSocket, id: string) {
            // Create PeerConnection
            console.log(`Offering to ${id}`);
            const pc = createPeerConnection(ws, id);

            // Create DataChannel
            const label = 'test';
            console.log(`Creating DataChannel with label "${label}"`);
            const dc = pc.createDataChannel(label);
            setupDataChannel(dc, id);

            // Send offer
            sendLocalDescription(ws, id, pc, 'offer');
        }

        // Create and set up a PeerConnection
        function createPeerConnection(ws: WebSocket, id: string): RTCPeerConnection {
            const pc = new RTCPeerConnection(config);
            pc.onconnectionstatechange = () => {
                console.log(`Connection state: ${pc.connectionState}`);
                peerConnectionState = pc.connectionState;
            };
            pc.onicegatheringstatechange = () =>
                console.log(`Gathering state: ${pc.iceGatheringState}`);
            pc.onicecandidate = (e) => {
                if (e.candidate && e.candidate.candidate) {
                    // Send candidate
                    sendLocalCandidate(ws, id, e.candidate);
                }
            };
            pc.ontrack = (evt) => {
                console.log(`Track from ${id} received`);
                videoElement.srcObject = evt.streams[0];
                videoElement.play();
            };
            pc.ondatachannel = (e) => {
                const dc = e.channel;
                console.log(`"DataChannel from ${id} received with label "${dc.label}"`);
                setupDataChannel(dc, id);

                dc.send(`Hello from ${localId}`);
            };

            peerConnectionMap[id] = pc;
            return pc;
        }

        // Setup a DataChannel
        function setupDataChannel(dc: RTCDataChannel, id: string): RTCDataChannel {
            dc.onopen = () => {
                console.log(`DataChannel from ${id} open`);
            };
            dc.onclose = () => {
                console.log(`DataChannel from ${id} closed`);
            };
            dc.onmessage = (e) => {
                if (typeof e.data != 'string') return;
                const data = JSON.parse(e.data);
                if (data.type === 'battery' && battery) {
                    battery.updateBatteryInfo(data.battery);
                } else if (data.type === 'network' && network) {
                    network.updateNetworkInfo(data.network);
                } else if (data.type === 'odometry' && odometry) {
                    odometry.updateOdometryData(data.odometry);
                } else if (data.type === 'latency' && latency) {
                    latency.updateLatencyInfo(data.latency, videoElement);
                } else if (data.type === 'log' && logs) {
                    logs.updateLogMessages(data.log);
                }
            };

            dataChannelMap[id] = dc;
            return dc;
        }

        function sendLocalDescription(ws: WebSocket, id: string, pc: RTCPeerConnection, type: 'offer' | 'answer') {
            (type == 'offer' ? pc.createOffer() : pc.createAnswer())
                .then((desc) => pc.setLocalDescription(desc))
                .then(() => {
                    const {sdp, type} = pc.localDescription!;
                    ws.send(
                        JSON.stringify({
                            id,
                            type,
                            auth: connectionInfo.auth,
                            access: connectionInfo.access,
                            description: sdp,
                        })
                    );
                });
        }

        function sendLocalCandidate(ws: WebSocket, id: string, cand: RTCIceCandidate) {
            const {candidate, sdpMid} = cand;
            ws.send(
                JSON.stringify({
                    id,
                    type: 'candidate',
                    auth: connectionInfo.auth,
                    access: connectionInfo.access,
                    candidate,
                    mid: sdpMid,
                })
            );
        }

        // Helper function to generate a random ID
        function randomId(length: number): string {
            const characters =
                '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz';
            const pickRandom = () =>
                characters.charAt(Math.floor(Math.random() * characters.length));
            return [...Array(length)].map(pickRandom).join('');
        }
    });

</script>

<main>
    <div class="video-container">
        <video id="video-element" muted autoplay playsinline loop
               src={peerConnectionState === 'connected' ? null : templateVideoSrc}>
        </video>
    </div>
    <div class="content">
        <div class="box" style="top: 5%; right: 5%;">
            <h2>Local ID</h2>
            <p>{localId}</p>
        </div>

        {#if DEV_MODE || peerConnectionState !== 'connected'}
            <h1>Origin Teleop</h1>
            <div class="box" style="top: 40%; left: 50%; transform: translateX(-50%);">
                <Connection bind:connectionInfo {handleOfferClick} {createOfferDisabled}/>
            </div>
        {/if}

        {#if DEV_MODE || peerConnectionState === 'connected'}
            <div class="box" style="top: 5%; left: 5%;">
                <Network bind:this={network}/>
            </div>
            <div class="box" style="top: 30%; left: 5%;">
                <Odometry bind:this={odometry}/>
            </div>
            <div class="box" style="bottom: 15%; left: 5%;">
                <Battery bind:this={battery}/>
            </div>

            {#if connectionInfo.enableMessaging}
                <div class="box" style="bottom: 15%; left: 50%; transform: translateX(-50%)">
                    <Messaging sendMessage={sendMessage}/>
                </div>
                <div class="box" style="top: 30%; right: 5%;">
                    <Logs bind:this={logs}/>
                </div>
            {/if}
            <div class="box" style="bottom: 15%; right: 5%;">
                <Latency bind:this={latency} sendMessage={sendMessage}/>
            </div>
        {/if}

        {#if DEV_MODE || connectionInfo.access === 'control'}
            <div class="box" style="top: 5%; left: 50%; transform: translateX(-50%)">
                <Gamepad sendControlMsg={sendControlMsg}/>
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

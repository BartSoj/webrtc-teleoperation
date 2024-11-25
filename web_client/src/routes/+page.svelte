<script lang="ts">
    import {onMount} from 'svelte';
    import Battery from './Battery.svelte';
    import Logs from './Logs.svelte';
    import Network from './Network.svelte';
    import Odometry from './Odometry.svelte';
    import Messaging from "./Messaging.svelte";
    import Latency from './Latency.svelte';

    const DEV_MODE: boolean = false;

    let localId: string = '';
    let auth: string = '';
    let access: string = 'observe';
    let offerId: string = '';

    let battery: Battery | undefined;
    let logs: Logs | undefined;
    let network: Network | undefined;
    let odometry: Odometry | undefined;
    let latency: Latency | undefined;

    let peerConnectionState: string = 'new';
    const templateVideoSrc: string = 'default.mp4';

    let createOfferDisabled: boolean = true;

    let controlMsg: any = null;

    let sendMessage: (msg: string) => void;
    let sendControlMsg: () => void;

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
                    forward: -gamepad.axes[1],
                    rotation: -gamepad.axes[2]
                };
                sendControlMsg();
            }
        }

        requestAnimationFrame(updateGamepadState);
    }

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

        const offerBtn = document.getElementById('offerBtn') as HTMLInputElement;
        const videoElement = document.getElementById('video-element') as HTMLVideoElement;

        setInterval(() => {
            const tR1 = Math.trunc(performance.now());
            for (const dc of Object.values(dataChannelMap)) {
                const message = JSON.stringify({
                    type: "latency",
                    latency: tR1
                });
                dc.send(message);
            }
        }, 2000); // Sends every 2 seconds

        sendMessage = (msg) => {
            for (const dc of Object.values(dataChannelMap)) {
                dc.send(msg);
            }
        }

        sendControlMsg = () => {
            if (access === 'control') {
                for (const dc of Object.values(dataChannelMap)) {
                    dc.send(JSON.stringify(controlMsg));
                }
            }
        }

        console.log('Connecting to signaling...');
        openSignaling(url)
            .then((ws) => {
                console.log('WebSocket connected, signaling ready');
                createOfferDisabled = false;
                offerBtn.onclick = () => offerPeerConnection(ws, offerId);
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
                    // Latency Calculation
                    const tR2 = performance.now(); // Receiver's current time
                    const {tR1, delay_since_received, sender_local_clock, track_times_msec} = data.latency;

                    // Round Trip Time (RTT) and Network Latency
                    const rtt = tR2 - delay_since_received - tR1;
                    const networkLatency = rtt / 2;
                    let senderTime = sender_local_clock + delay_since_received + networkLatency;

                    videoElement.requestVideoFrameCallback((now: DOMHighResTimeStamp, framemeta: VideoFrameCallbackMetadata) => {
                        if (framemeta.rtpTimestamp !== undefined) {
                            const delaySinceVideoCallbackRequested = now - tR2;
                            senderTime += delaySinceVideoCallbackRequested;
                            const [tSV1, tS1] = track_times_msec[0];

                            // Calculate expected and actual video times
                            const timeSinceLastKnownFrame = senderTime - tS1;
                            const expectedVideoTimeMsec = tSV1 + timeSinceLastKnownFrame;
                            const actualVideoTimeMsec = Math.trunc(framemeta.rtpTimestamp / 90); // Convert RTP timestamp to milliseconds

                            // Compute and update latency
                            const latencyVal = expectedVideoTimeMsec - actualVideoTimeMsec;
                            if (latency)
                                latency.updateLatencyInfo({latency: latencyVal});
                        } else {
                            console.log('RTP timestamp is not available for this frame.');
                        }
                    });
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
                            auth,
                            access,
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
                    auth,
                    access,
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

        window.addEventListener("gamepadconnected", handleGamepadConnected);
        window.addEventListener("gamepaddisconnected", handleGamepadDisconnected);

        updateGamepadState();

        return () => {
            window.removeEventListener("gamepadconnected", handleGamepadConnected);
            window.removeEventListener("gamepaddisconnected", handleGamepadDisconnected);
        };
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
                <h2>Send an offer through signaling</h2>
                <input type="text" id="auth" placeholder="auth" bind:value={auth} disabled={createOfferDisabled}/>
                <input type="text" id="offerId" placeholder="remote ID" bind:value={offerId}
                       disabled={createOfferDisabled}/>
                <select id="access" bind:value={access} disabled={createOfferDisabled}>
                    <option value="observe" selected>Observe</option>
                    <option value="control">Control</option>
                </select>
                <input type="button" id="offerBtn" value="Offer" disabled={createOfferDisabled}/>
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
            <div class="box" style="bottom: 15%; left: 50%; transform: translateX(-50%)">
                <Messaging sendMessage={sendMessage}/>
            </div>
            <div class="box" style="top: 30%; right: 5%;">
                <Logs bind:this={logs}/>
            </div>
            <div class="box" style="bottom: 15%; right: 5%;">
                <Latency bind:this={latency}/>
            </div>
        {/if}

        {#if DEV_MODE || access === 'control'}
            <div class="box" style="top: 5%; left: 50%; transform: translateX(-50%)">
                {#if controlMsg === null}
                    <p>No gamepads connected</p>
                {:else}
                    <p>Active {controlMsg.id}</p>
                {/if}
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

</style>

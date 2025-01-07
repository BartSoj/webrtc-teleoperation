import {get, writable} from 'svelte/store';
import type {ConnectionInfo, PeerConnectionEntry} from '$lib';

// Stores
export const localId = writable('');
export const peerConnectionMap = writable<{ [key: string]: PeerConnectionEntry }>({});
export const createOfferDisabled = writable(true);
export const messageStore = writable<{ id: string; message: string }>({id: '', message: ''});

// Reactive function stores
export const sendMessage = writable<(id: string, msg: string) => void>(() => {
});
export const sendControlMsg = writable<(id: string, msg: string) => void>(() => {
});
export const sendOffer = writable<(connectionInfo: ConnectionInfo) => void>(() => {
});

interface Message {
    id: string;
    type: string;
    description?: string;
    candidate?: string;
    mid?: string;
}

export function initializeTeleoperation(): void {
    const config: RTCConfiguration = {
        iceServers: [{urls: 'stun:stun.l.google.com:19302'}],
    };

    localId.set(randomId(4));
    const ip = location.hostname;
    const url = `ws://${ip}:8000/${get(localId)}`;

    sendMessage.set((id: string, msg: string) => {
        const entry = get(peerConnectionMap)[id];
        entry?.dataChannel?.send(msg);
    });

    sendControlMsg.set((id: string, msg: string) => {
        const entry = get(peerConnectionMap)[id];
        if (entry?.access === 'control') {
            entry.dataChannel?.send(msg);
        }
    });

    console.log('Connecting to signaling...');
    openSignaling(url)
        .then((ws) => {
            console.log('WebSocket connected, signaling ready');
            createOfferDisabled.set(false);
            sendOffer.set((connectionInfo: ConnectionInfo) => offerPeerConnection(ws, connectionInfo));
        })
        .catch((err) => console.error(err));

    function openSignaling(url: string): Promise<WebSocket> {
        return new Promise((resolve, reject) => {
            const ws = new WebSocket(url);

            ws.onopen = () => resolve(ws);
            ws.onerror = () => reject(new Error('WebSocket error'));
            ws.onclose = () => console.error('WebSocket disconnected');
            ws.onmessage = (e) => {
                if (typeof e.data !== 'string') return;
                const message: Message = JSON.parse(e.data);
                console.log(message);
                const {id, type} = message;

                let entry = get(peerConnectionMap)[id];
                if (!entry) {
                    if (type !== 'offer') return;

                    createPeerConnection(ws, id);
                    entry = get(peerConnectionMap)[id];
                }

                const pc = entry?.peerConnection;

                switch (type) {
                    case 'offer':
                    case 'answer':
                        pc?.setRemoteDescription({
                            sdp: message.description!,
                            type: message.type as RTCSdpType,
                        }).then(() => {
                            if (type === 'offer') {
                                sendLocalDescription(ws, id, 'answer');
                            }
                        });
                        break;
                    case 'candidate':
                        pc?.addIceCandidate({
                            candidate: message.candidate!,
                            sdpMid: message.mid!,
                        });
                        break;
                }
            };
        });
    }

    function offerPeerConnection(ws: WebSocket, connectionInfo: ConnectionInfo) {
        const id = connectionInfo.offerId;
        if (get(peerConnectionMap)[id]) return;

        console.log(`Offering to ${id}`);
        createPeerConnection(ws, id);

        const entry = get(peerConnectionMap)[id];
        entry!.auth = connectionInfo.auth;
        entry!.access = connectionInfo.access;
        entry!.enableMessaging = connectionInfo.enableMessaging;

        const label = 'test';
        const dc = entry!.peerConnection.createDataChannel(label);
        setupDataChannel(dc, id);

        sendLocalDescription(ws, id, 'offer');
    }

    function createPeerConnection(ws: WebSocket, id: string) {
        const pc = new RTCPeerConnection(config);

        peerConnectionMap.set({
            ...get(peerConnectionMap),
            [id]: {
                peerConnection: pc,
                state: 'new',
                enableMessaging: false,
            },
        });

        setTimeout(() => {
            if (pc.connectionState === 'new') {
                console.log(`${id} Closing connection`);
                pc.close();
                peerConnectionMap.update((map) => {
                    delete map[id];
                    return map;
                });
            }
        }, 3000)

        pc.onconnectionstatechange = () => {
            console.log(`${id} Connection state: ${pc.connectionState}`);
            peerConnectionMap.update((map) => {
                if (map[id]) {
                    map[id].state = pc.connectionState;

                    if (pc.connectionState === 'failed' || pc.connectionState === 'disconnected') {
                        console.log(`${id} Closing connection`);
                        pc.close();
                        delete map[id];
                    }
                }
                return map;
            });
        };

        pc.onicegatheringstatechange = () => console.log(`${id} Gathering state: ${pc.iceGatheringState}`);
        pc.onicecandidate = (e) => {
            if (e.candidate && e.candidate.candidate) {
                sendLocalCandidate(ws, id, e.candidate);
            }
        };

        pc.ontrack = (evt) => {
            console.log(`${id} Track received`);
            peerConnectionMap.update((map) => {
                if (map[id]) map[id].videoStream = evt.streams[0];
                return map;
            });
        };

        pc.ondatachannel = (e) => {
            const dc = e.channel;
            console.log(`${id} DataChannel received with label "${dc.label}"`);
            setupDataChannel(dc, id);
        };
    }

    function setupDataChannel(dc: RTCDataChannel, id: string) {
        dc.onopen = () => console.log(`DataChannel from ${id} open`);
        dc.onclose = () => console.log(`DataChannel from ${id} closed`);
        dc.onmessage = (e) => {
            if (typeof e.data === 'string') {
                messageStore.set({id, message: e.data});
            }
        };

        peerConnectionMap.update((map) => {
            if (map[id]) map[id].dataChannel = dc;
            return map;
        });
    }

    function sendLocalDescription(ws: WebSocket, id: string, type: 'offer' | 'answer') {
        const entry = get(peerConnectionMap)[id];
        const pc = entry?.peerConnection;

        (type === 'offer' ? pc?.createOffer() : pc?.createAnswer())
            ?.then((desc) => pc.setLocalDescription(desc!))
            .then(() => {
                const {sdp, type} = pc!.localDescription!;
                ws.send(
                    JSON.stringify({
                        id,
                        type,
                        auth: entry!.auth,
                        access: entry!.access,
                        description: sdp,
                    })
                );
            });
    }

    function sendLocalCandidate(ws: WebSocket, id: string, candidate: RTCIceCandidate) {
        const entry = get(peerConnectionMap)[id];
        ws.send(
            JSON.stringify({
                id,
                type: 'candidate',
                auth: entry?.auth,
                access: entry?.access,
                candidate: candidate.candidate,
                mid: candidate.sdpMid,
            })
        );
    }
}

function randomId(length: number): string {
    const chars = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz';
    return Array.from({length}, () => chars.charAt(Math.floor(Math.random() * chars.length))).join('');
}
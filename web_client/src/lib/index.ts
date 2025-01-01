// place files you want to import through the `$lib` alias in this folder.

export interface ConnectionInfo {
    offerId: string;
    enableMessaging: boolean;
    auth: string;
    access: 'observe' | 'control';
}

export interface PeerConnectionEntry {
    peerConnection: RTCPeerConnection;
    state: string;
    enableMessaging: boolean;
    videoStream?: MediaStream;
    dataChannel?: RTCDataChannel;
    auth?: string;
    access?: string;
}
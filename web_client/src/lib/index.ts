// place files you want to import through the `$lib` alias in this folder.

export interface ConnectionInfo {
    localId: string;
    auth: string;
    access: 'observe' | 'control';
    offerId: string;
    enableMessaging: boolean;
}
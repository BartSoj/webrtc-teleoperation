// place files you want to import through the `$lib` alias in this folder.

export interface ConnectionInfo {
    offerId: string;
    enableMessaging: boolean;
    auth: string;
    access: 'observe' | 'control';
}
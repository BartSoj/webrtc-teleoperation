<script lang="ts">
    import type {ConnectionInfo} from "$lib";
    import {createOfferDisabled, sendOffer} from "$lib/teleoperationStore";

    let connectionInfo: ConnectionInfo = {
        offerId: '',
        enableMessaging: false,
        auth: '',
        access: 'observe',
    };

    function handleOfferClick() {
        $sendOffer(connectionInfo)
    }

</script>

<div id="connection">
    <h2>Send an offer through signaling</h2>
    <input type="text" id="auth" placeholder="auth" bind:value={connectionInfo.auth}
           disabled={$createOfferDisabled}/>
    <input type="text" id="offerId" placeholder="remote ID" bind:value={connectionInfo.offerId}
           disabled={$createOfferDisabled}/>
    <select id="access" bind:value={connectionInfo.access} disabled={$createOfferDisabled}>
        <option value="observe" selected>Observe</option>
        <option value="control">Control</option>
    </select>
    <div style="display: inline-flex; align-items: center; gap: 0.5em; white-space: nowrap;">
        <label for="enableMessagingCheckbox">Enable Messaging</label>
        <input type="checkbox" bind:checked={connectionInfo.enableMessaging} id="enableMessagingCheckbox"
               disabled={$createOfferDisabled}/>
    </div>
    <input type="button" id="offerBtn" value="Offer" disabled={$createOfferDisabled} on:click={handleOfferClick}/>
</div>
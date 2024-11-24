<script lang="ts">
    interface BatteryInfo {
        voltage: number;
        state_of_charge: number;
    }

    let batteryInfo: BatteryInfo = {
        voltage: 0,
        state_of_charge: 0
    };

    export function updateBatteryInfo(newBatteryInfo: BatteryInfo) {
        batteryInfo = newBatteryInfo;
    }

    function getBatteryIcon(stateOfCharge: number): string {
        if (stateOfCharge >= 80) {
            return "/icons/battery/battery-full.png";
        } else if (stateOfCharge >= 60) {
            return "/icons/battery/battery-third.png";
        } else if (stateOfCharge >= 40) {
            return "/icons/battery/battery-half.png";
        } else if (stateOfCharge >= 20) {
            return "/icons/battery/battery-first.png";
        } else if (stateOfCharge > 0) {
            return "/icons/battery/battery-empty.png";
        } else {
            return "/icons/battery/battery-alert.png";
        }
    }
</script>

<div id="battery-info">
    <h2>Battery</h2>
    <div style="display: flex; align-items: center; gap: 8px;">
        <p>{batteryInfo.state_of_charge.toFixed(0)}%</p>
        <img src="{getBatteryIcon(batteryInfo.state_of_charge)}"
             alt="battery-icon"
             style="width: 24px; height: 24px; filter: invert(1)">
    </div>
</div>
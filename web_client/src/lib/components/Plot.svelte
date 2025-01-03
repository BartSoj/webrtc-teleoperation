<script lang="ts">
    import {messageStore} from "$lib/teleoperationStore";
    import {onMount} from "svelte";

    let Plotly: any;

    interface Position {
        x: number;
        y: number;
        z: number;
    }

    let positionData: Record<string, { x: number[], y: number[], z: number[] }> = {};

    let plotContainer: HTMLDivElement;

    function redrawPlot() {
        const layout: Partial<Plotly.Layout> = {
            autosize: true,
            uirevision: 'true',
            scene: {
                aspectmode: "manual",
                aspectratio: {
                    x: 1, y: 1, z: 1,
                },
                xaxis: {
                    nticks: 1,
                    range: [-10, 10],
                },
                yaxis: {
                    nticks: 1,
                    range: [-10, 10],
                },
                zaxis: {
                    nticks: 1,
                    range: [-10, 10],
                }
            },
        };

        let data: Array<Partial<Plotly.ScatterData>> = [];
        data = Object.keys(positionData).map((id) => ({
            x: positionData[id].x,
            y: positionData[id].y,
            z: positionData[id].z,
            type: 'scatter3d',
            mode: 'lines+markers',
            name: id,
            marker: {
                color: 'blue',
                size: 1.5,
                symbol: 'circle',
            },
            line: {
                color: 'blue',
                width: 2,
            },
        }));

        Plotly.newPlot(plotContainer, data, layout);
    }

    function updatePositionData(id: string, newPosition: Position) {
        if (!Plotly) return;
        if (!positionData[id]) {
            console.log("This should only be called when new peer connection is added");
            positionData[id] = {x: [], y: [], z: []};
            redrawPlot();
        }
        positionData[id].x.push(newPosition.x);
        positionData[id].y.push(newPosition.y);
        positionData[id].z.push(newPosition.z);

        Plotly.extendTraces(
            plotContainer,
            {
                x: [[newPosition.x]],
                y: [[newPosition.y]],
                z: [[newPosition.z]],
            },
            [Object.keys(positionData).indexOf(id)]
        );
    }

    onMount(async () => {
        const module = await import('plotly.js-dist');
        Plotly = module.default;
    });

    $: {
        const {id, message} = $messageStore;
        const data = JSON.parse(message || "{}");
        if (data?.type === "state_reference") updatePositionData(id, data.state_reference.pose.position);
    }
</script>

<div id="plot">
    <div class="plot-container" bind:this={plotContainer}><!-- Plotly chart will be drawn inside this DIV --></div>
</div>

<style>
    .plot-container {
        box-sizing: border-box;
        width: 75vw;
        height: 75vh;
    }
</style>
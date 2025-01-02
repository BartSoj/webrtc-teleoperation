<script lang="ts">
    import {onMount} from 'svelte';

    let plotContainer: HTMLDivElement;

    const generateCircularData = (numPoints: number, radius: number): { x: number[]; y: number[]; z: number[] } => {
        const theta = Array.from({length: numPoints}, (_, i) => (i / numPoints) * 2 * Math.PI);
        const x = theta.map(t => radius * Math.cos(t));
        const y = theta.map(t => radius * Math.sin(t));
        const z = Array(numPoints).fill(0); // Circle in the XY plane
        return {x, y, z};
    };

    // Generate data for a circle with 100 points and radius 5
    const {x, y, z}: { x: number[]; y: number[]; z: number[] } = generateCircularData(100, 5);

    let data: Array<Partial<Plotly.ScatterData>> = [
        {
            x,
            y,
            z,
            type: 'scatter3d',
            mode: 'lines+markers',
            marker: {
                color: 'blue',
                size: 1.5,
                symbol: 'circle',
            },
            line: {
                color: 'blue',
                width: 2,
            },
        },
    ]; // Default plot data

    const redrawPlot = async (): Promise<void> => {
        const module = await import('plotly.js-dist');
        let Plotly = module.default;

        const layout: Partial<Plotly.Layout> = {
            autosize: true,
            margin: {l: 0, r: 0, b: 0, t: 0},
        };

        if (plotContainer && typeof Plotly !== 'undefined') {
            await Plotly.newPlot(plotContainer, data, layout, {showSendToCloud: true});
        }
    };

    onMount(async (): Promise<void> => {
        await redrawPlot();
        window.addEventListener('resize', redrawPlot);
    });
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
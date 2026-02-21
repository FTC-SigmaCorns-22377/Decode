export function createChart(id, datasets) {
    const ctx = document.getElementById(id).getContext('2d');
    return new Chart(ctx, {
        type: 'line',
        data: {
            labels: [],
            datasets: datasets.map(ds => ({
                label: ds.label, borderColor: ds.color,
                data: [], borderWidth: 1, pointRadius: 0
            }))
        },
        options: {
            responsive: true, maintainAspectRatio: false, animation: false,
            scales: { x: { display: false } },
            plugins: { legend: { labels: { color: '#ccc', font: { size: 10 } } } }
        }
    });
}

export const chartWheels = createChart('chart-wheels', [
    { label: 'FL', color: '#ff4444' }, { label: 'FR', color: '#44ff44' },
    { label: 'BL', color: '#4444ff' }, { label: 'BR', color: '#ffff44' }
]);
export const chartMechs = createChart('chart-mechanisms', [
    { label: 'Flywheel', color: '#00ffff' }, { label: 'Turret', color: '#ff44ff' },
    { label: 'Spindexer Pwr', color: '#ff8800' }, { label: 'Spindexer Ang', color: '#88ff00' }
]);
export const chartPos = createChart('chart-pos', [
    { label: 'X', color: '#ff4444' }, { label: 'Y', color: '#44ff44' }, { label: 'Heading', color: '#4488ff' }
]);
export const chartError = createChart('chart-error', [
    { label: 'Err X', color: '#ff4444' }, { label: 'Err Y', color: '#44ff44' },
    { label: 'Err Heading', color: '#4488ff' }, { label: 'Err Vx', color: '#ff8800' },
    { label: 'Err Vy', color: '#aa44ff' }, { label: 'Err Omega', color: '#00ffff' }
]);
export const chartUncertainty = createChart('chart-uncertainty', [
    { label: 'Odo RMS σ (cm)', color: '#ff6600' },
    { label: 'Fusion RMS σ (cm)', color: '#00ff88' }
]);

export function syncCharts(history, currentIndex) {
    if (history.length === 0) return;
    const limit = 500;
    const start = Math.max(0, currentIndex - limit);
    const window = history.slice(start, currentIndex + 1);
    const labels = window.map(s => s.t);
    const update = (chart, dataExtractors) => {
        chart.data.labels = labels;
        dataExtractors.forEach((extractor, i) => {
            chart.data.datasets[i].data = window.map(extractor);
        });
        chart.update();
    };
    update(chartWheels, [s => s.telemetry.fl, s => s.telemetry.fr, s => s.telemetry.bl, s => s.telemetry.br]);
    update(chartMechs, [s => s.telemetry.flywheel, s => s.telemetry.turret, s => s.telemetry.spindexerPower, s => s.telemetry.spindexerAngle]);
    update(chartPos, [s => s.base.x, s => s.base.y, s => s.base.yaw]);
    update(chartError, [
        s => s.error ? s.error.x : null,
        s => s.error ? s.error.y : null,
        s => s.error ? s.error.yaw : null,
        s => s.error ? s.error.vx : null,
        s => s.error ? s.error.vy : null,
        s => s.error ? s.error.omega : null
    ]);
    update(chartUncertainty, [
        s => s.gtsam ? Math.sqrt((s.gtsam.odoCovXX || 0) + (s.gtsam.odoCovYY || 0)) * 100 : null,
        s => s.gtsam ? Math.sqrt((s.gtsam.covXX || 0) + (s.gtsam.covYY || 0)) * 100 : null
    ]);
}

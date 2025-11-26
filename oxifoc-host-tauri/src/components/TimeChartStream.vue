<script setup lang="ts">
/**
 * TimeChartStream.vue - WebGL-accelerated real-time streaming chart for ADC data
 *
 * CRITICAL REQUIREMENTS:
 * =======================
 * This component REQUIRES the GitHub version of TimeChart (1.0.0-beta or later),
 * NOT the npm package (0.5.2). The npm version lacks DataPointsBuffer mutation tracking.
 *
 * In package.json, use:
 *   "timechart": "github:huww98/TimeChart"
 *
 * DO NOT use:
 *   "timechart": "^0.5.2"  // This version breaks after ~40 seconds!
 *
 * DATA FORMAT:
 * ------------
 * AdcSample from oxifoc-protocol contains:
 *   - ia, ib, ic: Raw 12-bit ADC values for phase currents (0-4095, centered ~2048)
 *   - vbusMv: DC bus voltage in millivolts
 *   - fetTempCX10: FET temperature in 0.1°C units
 *   - seq: Monotonic sequence number
 *
 * The chart displays normalized phase currents (ia, ib, ic) centered around zero.
 */
import { computed, onMounted, onUnmounted, ref, watch } from 'vue'
import { storeToRefs } from 'pinia'
import TimeChart from 'timechart'
import type { AdcSample } from '../bindings'
import { error as logError } from '@tauri-apps/plugin-log'
import { useStreamStore } from '../stores/streamStore'

type DataPoint = { x: number; y: number }

const props = defineProps<{ windowMs: number }>()
const streamStore = useStreamStore()
const { latestSample, sampleCount } = storeToRefs(streamStore)
const TARGET_RATE_HZ = 60

// ADC configuration for normalization
const ADC_MIDPOINT = 2048 // 12-bit ADC center
const ADC_SCALE = 2048 // Scale factor for normalization

const chartRef = ref<HTMLDivElement | null>(null)
let chart: TimeChart | null = null
let themeObserver: MutationObserver | null = null
let resizeHandler: (() => void) | null = null
let updateScheduled = false

const windowMsComputed = computed(() => props.windowMs || 2_000)

/**
 * Initial data buffers - these get passed to TimeChart during initialization.
 * IMPORTANT: After buildChart(), TimeChart wraps these with DataPointsBuffer prototype.
 * From that point on, use getSeriesData() to access the tracked arrays, NOT these buffers.
 */
const dataBuffers: DataPoint[][] = [[], [], []]

const seriesConfig = [
  { name: 'Phase A (Ia)', colorVar: '--color-primary', fallback: '#22d3ee' },
  { name: 'Phase B (Ib)', colorVar: '--color-secondary', fallback: '#8b5cf6' },
  { name: 'Phase C (Ic)', colorVar: '--color-accent', fallback: '#f97316' },
]

// Track the latest x value for manual scrolling (not using realTime mode)
let latestX = 0

// Canvas-based color normalizer
let colorCtx: CanvasRenderingContext2D | null = null
const ensureColorCtx = () => {
  if (!colorCtx && typeof document !== 'undefined') {
    const canvas = document.createElement('canvas')
    canvas.width = 1
    canvas.height = 1
    colorCtx = canvas.getContext('2d')
  }
}

const getCssVar = (name: string, fallback: string): string => {
  if (typeof window === 'undefined') return fallback
  const value = getComputedStyle(document.documentElement).getPropertyValue(name).trim()
  return value || fallback
}

const normalizeColor = (value: string, fallback: string): string => {
  if (typeof window === 'undefined') return fallback
  ensureColorCtx()
  if (!colorCtx) return fallback
  try {
    colorCtx.fillStyle = value
    colorCtx.fillRect(0, 0, 1, 1)
    const pixel = colorCtx.getImageData(0, 0, 1, 1).data
    if (pixel[3] === 0) return fallback
    const [r, g, b, a] = pixel
    const alpha = (a / 255).toFixed(3).replace(/0+$/, '').replace(/\.$/, '')
    return alpha && alpha !== '1' ? `rgba(${r}, ${g}, ${b}, ${alpha})` : `rgb(${r}, ${g}, ${b})`
  } catch (e) {
    console.warn('Color normalization failed for', value, e)
    return fallback
  }
}

const resolveVarColor = (varName: string, fallback: string) =>
  normalizeColor(getCssVar(varName, fallback), fallback)

const getColors = () => ({
  text: resolveVarColor('--color-base-content', '#0f172a'),
  grid: resolveVarColor('--color-base-300', '#cbd5e1'),
  legendBg: resolveVarColor('--color-base-100', '#ffffff'),
  legendBorder: resolveVarColor('--color-base-300', '#cbd5e1'),
  series: seriesConfig.map((s) => resolveVarColor(s.colorVar, s.fallback)),
})

const applyAxisStyle = (colors: { text: string; grid: string }) => {
  if (!chartRef.value?.shadowRoot) return
  const root = chartRef.value.shadowRoot
  let styleEl = root.querySelector<HTMLStyleElement>('#theme-axis-style')
  if (!styleEl) {
    styleEl = document.createElement('style')
    styleEl.id = 'theme-axis-style'
    root.appendChild(styleEl)
  }
  styleEl.textContent = `
    text { fill: ${colors.text}; }
    .domain, .tick line { stroke: ${colors.grid}; }
  `
}

const applyLegendStyle = (colors: { text: string; legendBg: string; legendBorder: string }) => {
  const legendEl = chartRef.value?.shadowRoot?.querySelector('chart-legend') as HTMLElement | null
  const legendRoot = legendEl?.shadowRoot
  if (!legendRoot) return

  let styleEl = legendRoot.querySelector<HTMLStyleElement>('#theme-legend-style')
  if (!styleEl) {
    styleEl = document.createElement('style')
    styleEl.id = 'theme-legend-style'
    legendRoot.appendChild(styleEl)
  }

  styleEl.textContent = `
    :host {
      background: ${colors.legendBg};
      color: ${colors.text};
      border: 1px solid ${colors.legendBorder};
    }
    .item:not(.visible) {
      color: ${colors.text}CC;
    }
  `
}

const buildChart = () => {
  if (!chartRef.value) return
  const colors = getColors()

  chartRef.value.style.color = colors.text
  chartRef.value.style.backgroundColor = 'transparent'

  // Following TimeChart demo.js pattern - NO realTime mode, manual xRange management
  // Note: dataBuffers will be converted to DataPointsBuffer by TimeChart
  chart = new TimeChart(chartRef.value, {
    series: seriesConfig.map((s, idx) => ({
      name: s.name,
      // eslint-disable-next-line @typescript-eslint/no-explicit-any -- TimeChart internally converts arrays to DataPointsBuffer
      data: dataBuffers[idx] as any,
      color: colors.series[idx] ?? s.fallback,
      lineWidth: 2,
    })),
    paddingLeft: 52,
    paddingRight: 16,
    paddingTop: 12,
    paddingBottom: 28,
    xRange: { min: 0, max: windowMsComputed.value },
    yRange: { min: -1.2, max: 1.2 }, // Normalized range for phase currents
    realTime: false, // Manual scrolling - we control xRange based on actual data
    backgroundColor: 'transparent',
  })

  applyAxisStyle(colors)
  applyLegendStyle(colors)
}

const rebuildChart = () => {
  if (chart) {
    chart.dispose()
    chart = null
  }
  // Reset state for fresh start
  latestX = 0
  dataBuffers.forEach((buf) => (buf.length = 0))
  buildChart()
}

/**
 * Get the actual DataPointsBuffer arrays from TimeChart.
 *
 * CRITICAL: These are the arrays with mutation tracking (pushed_back, poped_front, etc.)
 * Always use this function to access data arrays for push/shift operations.
 * Never use the original dataBuffers directly after chart initialization.
 */
// eslint-disable-next-line @typescript-eslint/no-explicit-any
const getSeriesData = () => chart?.options.series.map((s: any) => s.data) ?? []

/**
 * Scheduled update handler - called via requestAnimationFrame for batching.
 */
const scheduledUpdate = () => {
  updateScheduled = false

  if (!chart) return

  try {
    const seriesData = getSeriesData()

    // Trim old data using shift() on the DataPointsBuffer arrays
    const cutoffX = latestX - windowMsComputed.value * 2
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    seriesData.forEach((data: any) => {
      while (data.length > 0 && data[0].x < cutoffX) {
        data.shift()
      }
    })

    // Manually scroll xRange to follow the latest data
    const windowMs = windowMsComputed.value
    chart.options.xRange = {
      min: latestX - windowMs,
      max: latestX,
    }

    // Sync all changes to WebGL and render
    chart.update()
  } catch (e) {
    void logError(`TimeChart ERROR during update: ${e}`)
  }
}

/**
 * Normalize raw ADC value to -1.0 to 1.0 range.
 * ADC values are 12-bit (0-4095), centered around 2048.
 */
const normalizeAdcValue = (raw: number): number => {
  return (raw - ADC_MIDPOINT) / ADC_SCALE
}

/**
 * Append a new ADC sample to all series.
 *
 * Extracts phase currents (ia, ib, ic) from AdcSample and normalizes them.
 */
const appendSample = (sample: AdcSample) => {
  if (!chart) return

  // Increment x by fixed interval for smooth, stable timing
  latestX += 1000 / TARGET_RATE_HZ // ~16.67ms per sample at 60Hz

  // Normalize phase current values and push to chart
  const normalizedValues = [
    normalizeAdcValue(sample.ia),
    normalizeAdcValue(sample.ib),
    normalizeAdcValue(sample.ic),
  ]

  // Push to the DataPointsBuffer arrays (mutation is tracked)
  const seriesData = getSeriesData()
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  seriesData.forEach((data: any, idx: number) => {
    data.push({ x: latestX, y: normalizedValues[idx] })
  })

  // Batch updates - only one requestAnimationFrame per frame
  if (!updateScheduled) {
    updateScheduled = true
    requestAnimationFrame(scheduledUpdate)
  }
}

const observeTheme = () => {
  if (typeof MutationObserver === 'undefined') return

  themeObserver = new MutationObserver((mutations) => {
    for (const mutation of mutations) {
      if (mutation.type === 'attributes' && mutation.attributeName === 'data-theme') {
        rebuildChart()
      }
    }
  })

  themeObserver.observe(document.documentElement, {
    attributes: true,
    attributeFilter: ['data-theme'],
  })
}

const cleanup = () => {
  updateScheduled = false

  if (themeObserver) {
    themeObserver.disconnect()
    themeObserver = null
  }

  if (typeof window !== 'undefined' && resizeHandler) {
    window.removeEventListener('resize', resizeHandler)
    resizeHandler = null
  }

  if (chart) {
    chart.dispose()
    chart = null
  }
}

onMounted(() => {
  void streamStore.ensureStream()
  buildChart()

  resizeHandler = () => chart?.onResize()
  window.addEventListener('resize', resizeHandler)
  observeTheme()
})

/**
 * Watch sampleCount for reliable sample detection.
 */
watch(sampleCount, () => {
  const sample = latestSample.value
  if (sample) appendSample(sample)
})

// Rebuilding the chart is the most reliable way to change the window size.
watch(windowMsComputed, () => {
  rebuildChart()
})

onUnmounted(() => cleanup())

if (import.meta.hot) {
  import.meta.hot.dispose(() => {
    cleanup()
  })
}
</script>

<template>
  <div class="card bg-base-100 shadow-xl">
    <div class="card-body">
      <h2 class="card-title">Phase Currents</h2>
      <div class="mt-4 h-72 w-full">
        <div ref="chartRef" class="h-full w-full"></div>
      </div>
    </div>
  </div>
</template>

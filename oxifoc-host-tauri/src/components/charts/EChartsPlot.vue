<script setup lang="ts">
/**
 * EChartsPlot.vue - Canvas-based real-time streaming chart for ADC data
 *
 * High-performance ECharts implementation with:
 * - Pre-allocated tuple pools to eliminate GC pressure
 * - Binary search for time-based windowing
 * - LTTB sampling for large datasets
 * - Zero-allocation updates via tuple mutation
 */
import { computed, onUnmounted, ref, watch } from 'vue'
import { storeToRefs } from 'pinia'
import { use } from 'echarts/core'
import { CanvasRenderer } from 'echarts/renderers'
import { LineChart } from 'echarts/charts'
import { GridComponent, LegendComponent } from 'echarts/components'
import VChart from 'vue-echarts'
import type { EChartsOption } from 'echarts'
import { useStreamStore } from '../../stores/streamStore'
import { useFrameTiming } from '../../composables/useFrameTiming'

// Register required ECharts components
use([CanvasRenderer, LineChart, GridComponent, LegendComponent])

type SeriesConfig = {
  name: string
  colorVar: string
  fallback: string
}

const props = defineProps<{ windowMs?: number }>()
const streamStore = useStreamStore()
const { latestSample, approxUpdateHz } = storeToRefs(streamStore)

const chartRef = ref<InstanceType<typeof VChart> | null>(null)
let rafId: number | null = null
let updatePending = false
const { measureFrame, avgFrameTimeMs, maxFps } = useFrameTiming()

/**
 * Pre-allocated tuple pools to eliminate GC pressure.
 * Each tuple [x, y] is created once at startup and mutated in place.
 */
const MAX_POINTS = 2000 // More than we'll ever need for a 20s window at 60Hz
const phaseAPool: [number, number][] = Array.from({ length: MAX_POINTS }, () => [0, 0])
const phaseBPool: [number, number][] = Array.from({ length: MAX_POINTS }, () => [0, 0])
const phaseCPool: [number, number][] = Array.from({ length: MAX_POINTS }, () => [0, 0])
let activePointCount = 0

/**
 * Flag to temporarily pause chart data updates during user interaction.
 * Prevents click events from being swallowed during rapid canvas redraws.
 */
let updatesPaused = false

const windowMs = computed(() => props.windowMs ?? 2000)
const windowSec = computed(() => windowMs.value / 1000)

const seriesDefs: SeriesConfig[] = [
  { name: 'Phase A', colorVar: '--color-primary', fallback: '#22d3ee' },
  { name: 'Phase B', colorVar: '--color-secondary', fallback: '#8b5cf6' },
  { name: 'Phase C', colorVar: '--color-accent', fallback: '#f97316' },
]

/**
 * Convert any CSS color (including OKLCH) to RGB hex format
 * Uses canvas to leverage browser's color parsing
 */
const colorToRgb = (cssColor: string): string => {
  if (typeof window === 'undefined') return cssColor

  const canvas = document.createElement('canvas')
  canvas.width = 1
  canvas.height = 1
  const ctx = canvas.getContext('2d')

  if (!ctx) return cssColor

  ctx.fillStyle = cssColor
  return ctx.fillStyle // Returns in rgb() or #hex format
}

const getCssVar = (name: string, fallback: string): string => {
  if (typeof window === 'undefined') return fallback
  const value = getComputedStyle(document.documentElement).getPropertyValue(name).trim()
  const color = value || fallback
  return colorToRgb(color)
}

const getColors = () => seriesDefs.map((s) => getCssVar(s.colorVar, s.fallback))

/**
 * Build chart data from samples using binary search and pre-allocated tuples.
 * Mutates tuple values in place - ZERO allocations per frame.
 */
const buildChartData = (windowMs: number) => {
  const samples = streamStore.samples
  if (!samples.length) {
    activePointCount = 0
    return
  }

  const latestTimestamp = samples[samples.length - 1].timestampMs
  const cutoff = latestTimestamp - windowMs

  // Binary search for the first sample within the window (samples are sorted by timestamp)
  let low = 0
  let high = samples.length
  while (low < high) {
    const mid = (low + high) >>> 1
    if (samples[mid].timestampMs < cutoff) {
      low = mid + 1
    } else {
      high = mid
    }
  }
  const startIdx = low

  // Calculate active point count (capped to pool size)
  activePointCount = Math.min(samples.length - startIdx, MAX_POINTS)

  // Mutate tuple values in place - no allocations!
  for (let i = 0; i < activePointCount; i++) {
    const sample = samples[startIdx + i]
    const x = (sample.timestampMs - latestTimestamp) / 1000
    const y0 = sample.values[0] ?? 0
    const y1 = sample.values[1] ?? 0
    const y2 = sample.values[2] ?? 0

    // Mutate existing tuples - these were pre-allocated at startup
    phaseAPool[i][0] = x
    phaseAPool[i][1] = y0
    phaseBPool[i][0] = x
    phaseBPool[i][1] = y1
    phaseCPool[i][0] = x
    phaseCPool[i][1] = y2
  }
}

/**
 * Reactive chart option
 */
const chartOption = ref<EChartsOption>({
  animation: false,
  grid: { left: 52, right: 16, top: 50, bottom: 36 },
  xAxis: {
    type: 'value',
    min: -windowSec.value,
    max: 0,
    axisLabel: {
      formatter: (val: number) => `${val.toFixed(1)}s`,
      color: getCssVar('--color-base-content', '#0f172a'),
    },
    axisLine: { lineStyle: { color: getCssVar('--color-base-300', '#cbd5e1') } },
    splitLine: { lineStyle: { color: getCssVar('--color-base-300', '#cbd5e1'), opacity: 0.28 } },
  },
  yAxis: {
    type: 'value',
    min: -1.6,
    max: 1.6,
    axisLabel: {
      color: getCssVar('--color-base-content', '#0f172a'),
    },
    axisLine: { show: false },
    splitLine: { lineStyle: { color: getCssVar('--color-base-300', '#cbd5e1'), opacity: 0.28 } },
  },
  legend: {
    top: 10,
    right: 10,
    orient: 'horizontal',
    itemWidth: 14,
    itemHeight: 10,
    itemGap: 14,
    icon: 'circle',
    selectedMode: 'multiple',
    textStyle: {
      color: getCssVar('--color-base-content', '#0f172a'),
      fontSize: 13,
    },
    inactiveColor: getCssVar('--color-base-300', '#cbd5e1'),
  },
  series: (() => {
    const colors = getColors()
    return seriesDefs.map((s, idx) => ({
      name: s.name,
      type: 'line',
      showSymbol: false,
      smooth: false, // Disabled for performance - no Bezier interpolation
      sampling: 'lttb',
      large: true, // Enable large data mode optimization
      largeThreshold: 500,
      lineStyle: { color: colors[idx] ?? s.fallback, width: 2 },
      emphasis: { disabled: true },
      data: [],
    }))
  })(),
})

/**
 * Perform the actual chart update.
 * Uses requestAnimationFrame for optimal frame alignment.
 */
const doChartUpdate = () => {
  rafId = null
  updatePending = false

  if (updatesPaused || !chartRef.value) return

  // Build data in place using binary search (zero allocations)
  buildChartData(windowMs.value)

  // Slice creates shallow copy with references to same tuples - minimal overhead
  // This is necessary because ECharts needs to know the array length
  const phaseASlice = phaseAPool.slice(0, activePointCount)
  const phaseBSlice = phaseBPool.slice(0, activePointCount)
  const phaseCSlice = phaseCPool.slice(0, activePointCount)

  // Use manual setOption to avoid disrupting legend state
  const chart = chartRef.value
  measureFrame(() => {
    chart.setOption({
      xAxis: {
        min: -windowSec.value,
        max: 0,
      },
      series: [
        { name: seriesDefs[0].name, data: phaseASlice },
        { name: seriesDefs[1].name, data: phaseBSlice },
        { name: seriesDefs[2].name, data: phaseCSlice },
      ],
    })
  })
}

// Update chart data on new samples - renders at display refresh rate
watch(latestSample, () => {
  if (updatePending || updatesPaused) return
  updatePending = true
  rafId = requestAnimationFrame(doChartUpdate)
})

/**
 * Mouse interaction handlers for pausing updates during clicks.
 */
const onZrMouseDown = () => {
  updatesPaused = true
}

const onZrMouseUp = () => {
  // Single-frame delay ensures the click event fires before updates resume
  requestAnimationFrame(() => {
    updatesPaused = false
  })
}

// Cleanup on unmount
onUnmounted(() => {
  if (rafId !== null) {
    cancelAnimationFrame(rafId)
    rafId = null
  }
})

// Initialize stream
streamStore.ensureStream()
</script>

<template>
  <div class="card bg-base-100 shadow-xl">
    <div class="card-body">
      <div class="flex flex-col items-start gap-2">
        <h2 class="card-title">Phase Currents</h2>
        <div class="flex items-center gap-2 flex-wrap">
          <div class="badge badge-outline">
            {{ approxUpdateHz ? `${approxUpdateHz.toFixed(1)} Hz` : '— Hz' }} incoming
          </div>
          <div class="badge badge-success badge-outline">
            {{ avgFrameTimeMs ? `${avgFrameTimeMs.toFixed(1)}ms` : '—' }} / frame
          </div>
          <div class="badge badge-info badge-outline">
            {{ maxFps ? `${maxFps.toFixed(0)} fps` : '—' }} capable
          </div>
        </div>
      </div>
      <div class="mt-4 h-72 w-full">
        <VChart
          ref="chartRef"
          class="h-full w-full"
          :option="chartOption"
          :autoresize="true"
          :manual-update="true"
          @zr:mousedown="onZrMouseDown"
          @zr:mouseup="onZrMouseUp" />
      </div>
    </div>
  </div>
</template>

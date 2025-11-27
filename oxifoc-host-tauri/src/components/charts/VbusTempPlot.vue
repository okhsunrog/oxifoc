<script setup lang="ts">
/**
 * VbusTempPlot.vue - Dual-axis chart for voltage and temperature
 *
 * Shows DC bus voltage (V) and FET temperature (°C) over time.
 * Uses same high-performance patterns as EChartsPlot.
 */
import { computed, onUnmounted, ref } from 'vue'
import { use } from 'echarts/core'
import { CanvasRenderer } from 'echarts/renderers'
import { LineChart } from 'echarts/charts'
import { GridComponent, LegendComponent } from 'echarts/components'
import VChart from 'vue-echarts'
import type { EChartsOption } from 'echarts'
import { useStreamStore } from '../../stores/streamStore'
import { useFrameTiming } from '../../composables/useFrameTiming'

use([CanvasRenderer, LineChart, GridComponent, LegendComponent])

const props = defineProps<{ windowMs?: number }>()
const streamStore = useStreamStore()

const chartRef = ref<InstanceType<typeof VChart> | null>(null)
let updateInterval: ReturnType<typeof setInterval> | null = null
const { measureFrame, avgFrameTimeMs, maxFps } = useFrameTiming()

// Update at 10Hz - voltage/temp change slowly, no need for 60Hz
const UPDATE_INTERVAL_MS = 100

// Pre-allocated tuple pools
const MAX_POINTS = 2000
const voltagePool: [number, number][] = Array.from({ length: MAX_POINTS }, () => [0, 0])
const tempPool: [number, number][] = Array.from({ length: MAX_POINTS }, () => [0, 0])
let activePointCount = 0

let updatesPaused = false

const windowMs = computed(() => props.windowMs ?? 2000)
const windowSec = computed(() => windowMs.value / 1000)

const colorToRgb = (cssColor: string): string => {
  if (typeof window === 'undefined') return cssColor
  const canvas = document.createElement('canvas')
  canvas.width = 1
  canvas.height = 1
  const ctx = canvas.getContext('2d')
  if (!ctx) return cssColor
  ctx.fillStyle = cssColor
  return ctx.fillStyle
}

const getCssVar = (name: string, fallback: string): string => {
  if (typeof window === 'undefined') return fallback
  const value = getComputedStyle(document.documentElement).getPropertyValue(name).trim()
  const color = value || fallback
  return colorToRgb(color)
}

const buildChartData = (windowMs: number) => {
  const samples = streamStore.samples
  if (!samples.length) {
    activePointCount = 0
    return
  }

  const latestTimestamp = samples[samples.length - 1].timestampMs
  const cutoff = latestTimestamp - windowMs

  // Binary search for first sample in window
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

  activePointCount = Math.min(samples.length - startIdx, MAX_POINTS)

  for (let i = 0; i < activePointCount; i++) {
    const sample = samples[startIdx + i]
    const x = (sample.timestampMs - latestTimestamp) / 1000
    // Convert mV to V
    const voltage = sample.raw.vbusMv / 1000
    // Convert 0.1°C to °C
    const temp = sample.raw.fetTempCX10 / 10

    voltagePool[i][0] = x
    voltagePool[i][1] = voltage
    tempPool[i][0] = x
    tempPool[i][1] = temp
  }
}

const chartOption = ref<EChartsOption>({
  animation: false,
  grid: { left: 60, right: 60, top: 50, bottom: 36 },
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
  yAxis: [
    {
      type: 'value',
      name: 'Voltage (V)',
      position: 'left',
      axisLabel: {
        formatter: '{value} V',
        color: getCssVar('--color-warning', '#eab308'),
      },
      axisLine: { show: true, lineStyle: { color: getCssVar('--color-warning', '#eab308') } },
      splitLine: { lineStyle: { color: getCssVar('--color-base-300', '#cbd5e1'), opacity: 0.28 } },
    },
    {
      type: 'value',
      name: 'Temp (°C)',
      position: 'right',
      axisLabel: {
        formatter: '{value}°C',
        color: getCssVar('--color-error', '#ef4444'),
      },
      axisLine: { show: true, lineStyle: { color: getCssVar('--color-error', '#ef4444') } },
      splitLine: { show: false },
    },
  ],
  legend: {
    top: 10,
    right: 70,
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
  series: [
    {
      name: 'Voltage',
      type: 'line',
      yAxisIndex: 0,
      showSymbol: false,
      smooth: false,
      sampling: 'lttb',
      lineStyle: { color: getCssVar('--color-warning', '#eab308'), width: 2 },
      emphasis: { disabled: true },
      data: [],
    },
    {
      name: 'Temperature',
      type: 'line',
      yAxisIndex: 1,
      showSymbol: false,
      smooth: false,
      sampling: 'lttb',
      lineStyle: { color: getCssVar('--color-error', '#ef4444'), width: 2 },
      emphasis: { disabled: true },
      data: [],
    },
  ],
})

const doChartUpdate = () => {
  if (updatesPaused || !chartRef.value) return
  if (!streamStore.samples.length) return

  buildChartData(windowMs.value)

  const voltageSlice = voltagePool.slice(0, activePointCount)
  const tempSlice = tempPool.slice(0, activePointCount)

  const chart = chartRef.value
  measureFrame(() => {
    chart.setOption({
      xAxis: {
        min: -windowSec.value,
        max: 0,
      },
      series: [
        { name: 'Voltage', data: voltageSlice },
        { name: 'Temperature', data: tempSlice },
      ],
    })
  })
}

// Start interval-based updates when component mounts
const startUpdates = () => {
  if (updateInterval) return
  updateInterval = setInterval(doChartUpdate, UPDATE_INTERVAL_MS)
}

const stopUpdates = () => {
  if (updateInterval) {
    clearInterval(updateInterval)
    updateInterval = null
  }
}

// Start updates immediately
startUpdates()

const onZrMouseDown = () => {
  updatesPaused = true
}

const onZrMouseUp = () => {
  requestAnimationFrame(() => {
    updatesPaused = false
  })
}

onUnmounted(() => {
  stopUpdates()
})

streamStore.ensureStream()
</script>

<template>
  <div class="card bg-base-100 shadow-xl">
    <div class="card-body">
      <div class="flex flex-col items-start gap-2">
        <h2 class="card-title">Voltage & Temperature</h2>
        <div class="flex items-center gap-2 flex-wrap">
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

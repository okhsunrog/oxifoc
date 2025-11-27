<script setup lang="ts">
import { computed, ref } from 'vue'
import { storeToRefs } from 'pinia'
import EChartsPlot from './charts/EChartsPlot.vue'
import VbusTempPlot from './charts/VbusTempPlot.vue'
import { useStreamStore } from '../stores/streamStore'

const windowMs = ref(2000)
const windowSeconds = computed(() => windowMs.value / 1000)

const streamStore = useStreamStore()
const { approxUpdateHz, latestSample } = storeToRefs(streamStore)

// Ensure stream is running
streamStore.ensureStream()
</script>

<template>
  <div class="w-full space-y-3">
    <div class="flex items-center justify-between">
      <h2 class="text-xl font-bold">Real-Time Telemetry</h2>
      <span class="badge badge-outline font-mono">
        {{ approxUpdateHz ? `${approxUpdateHz.toFixed(1)} Hz` : '— Hz' }}
      </span>
    </div>

    <!-- Phase Currents -->
    <div class="card bg-base-100 shadow-xl">
      <div class="card-body">
        <h2 class="card-title">Phase Currents</h2>
        <div class="grid grid-cols-3 gap-4 mt-4">
          <div class="stat bg-base-200 rounded-lg">
            <div class="stat-title">Phase A</div>
            <div class="stat-value text-primary text-2xl font-mono">
              {{ latestSample ? latestSample.values[0].toFixed(3) : '—' }}
            </div>
            <div class="stat-desc">Normalized (-1.0 to 1.0)</div>
          </div>
          <div class="stat bg-base-200 rounded-lg">
            <div class="stat-title">Phase B</div>
            <div class="stat-value text-secondary text-2xl font-mono">
              {{ latestSample ? latestSample.values[1].toFixed(3) : '—' }}
            </div>
            <div class="stat-desc">Normalized (-1.0 to 1.0)</div>
          </div>
          <div class="stat bg-base-200 rounded-lg">
            <div class="stat-title">Phase C</div>
            <div class="stat-value text-accent text-2xl font-mono">
              {{ latestSample ? latestSample.values[2].toFixed(3) : '—' }}
            </div>
            <div class="stat-desc">Normalized (-1.0 to 1.0)</div>
          </div>
        </div>
      </div>
    </div>

    <!-- Voltage & Temperature -->
    <div class="card bg-base-100 shadow-xl">
      <div class="card-body">
        <h2 class="card-title">Voltage & Temperature</h2>
        <div class="grid grid-cols-2 gap-4 mt-4">
          <div class="stat bg-base-200 rounded-lg">
            <div class="stat-title">DC Bus Voltage</div>
            <div class="stat-value text-warning text-2xl font-mono">
              {{ latestSample ? (latestSample.raw.vbusMv / 1000).toFixed(2) : '—' }}
            </div>
            <div class="stat-desc">Volts</div>
          </div>
          <div class="stat bg-base-200 rounded-lg">
            <div class="stat-title">FET Temperature</div>
            <div class="stat-value text-error text-2xl font-mono">
              {{ latestSample ? (latestSample.raw.fetTempCX10 / 10).toFixed(1) : '—' }}
            </div>
            <div class="stat-desc">°C</div>
          </div>
        </div>
      </div>
    </div>

    <!-- Raw ADC Values -->
    <div class="card bg-base-100 shadow-xl">
      <div class="card-body">
        <h2 class="card-title">Raw ADC Values</h2>
        <div class="grid grid-cols-3 gap-4 mt-4">
          <div class="stat bg-base-200 rounded-lg">
            <div class="stat-title">Phase A (raw)</div>
            <div class="stat-value text-sm font-mono">
              {{ latestSample ? latestSample.raw.ia : '—' }}
            </div>
          </div>
          <div class="stat bg-base-200 rounded-lg">
            <div class="stat-title">Phase B (raw)</div>
            <div class="stat-value text-sm font-mono">
              {{ latestSample ? latestSample.raw.ib : '—' }}
            </div>
          </div>
          <div class="stat bg-base-200 rounded-lg">
            <div class="stat-title">Phase C (raw)</div>
            <div class="stat-value text-sm font-mono">
              {{ latestSample ? latestSample.raw.ic : '—' }}
            </div>
          </div>
        </div>
        <div class="mt-2">
          <div class="text-sm text-base-content/70">
            Sequence:
            <span class="font-mono">{{ latestSample ? latestSample.raw.seq : '—' }}</span>
          </div>
        </div>
      </div>
    </div>

    <!-- Window Control for Charts -->
    <div class="flex items-center gap-2">
      <span class="text-sm font-semibold text-base-content/80">Chart Window</span>
      <input
        v-model.number="windowMs"
        type="range"
        min="500"
        max="20000"
        step="500"
        class="range range-primary range-sm flex-1" />
      <span class="w-14 text-right font-mono text-sm">{{ windowSeconds.toFixed(1) }}s</span>
    </div>

    <!-- Charts -->
    <EChartsPlot :windowMs="windowMs" />
    <VbusTempPlot :windowMs="windowMs" />
  </div>
</template>

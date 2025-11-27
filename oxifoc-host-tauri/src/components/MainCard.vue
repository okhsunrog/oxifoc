<script setup lang="ts">
import { useConnectionStore } from '../stores/connectionStore'
import TelemetryCharts from './TelemetryCharts.vue'

const connectionStore = useConnectionStore()
</script>

<template>
  <div class="card bg-base-100 shadow-xl">
    <div class="card-body">
      <div class="flex justify-between items-center mb-4">
        <div class="flex items-center gap-3">
          <h2 class="card-title text-xl">Device</h2>
          <div class="badge badge-success gap-1">
            <span class="w-1.5 h-1.5 bg-success-content rounded-full animate-pulse"></span>
            Connected
          </div>
        </div>
        <button class="btn btn-sm btn-error" @click="connectionStore.disconnect()">
          Disconnect
        </button>
      </div>

      <TelemetryCharts />
    </div>
  </div>
</template>

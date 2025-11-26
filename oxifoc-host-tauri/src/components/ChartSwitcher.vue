<script setup lang="ts">
import { computed, ref } from 'vue'
import { storeToRefs } from 'pinia'
import TimeChartStream from './TimeChartStream.vue'
import { useStreamStore } from '../stores/streamStore'

const windowMs = ref(20000)
const windowSeconds = computed(() => windowMs.value / 1000)

const streamStore = useStreamStore()
const { approxUpdateHz } = storeToRefs(streamStore)
</script>

<template>
  <div class="w-full space-y-3">
    <div>
      <h2 class="text-xl font-bold">Real-Time Plot</h2>
    </div>

    <div class="flex flex-col gap-3 sm:flex-row sm:items-center sm:gap-4">
      <div class="flex items-center gap-2">
        <span class="text-xs text-base-content/70 uppercase tracking-wide">Incoming</span>
        <span class="badge badge-outline">
          {{ approxUpdateHz ? `${approxUpdateHz.toFixed(1)} Hz` : '— Hz' }}
        </span>
      </div>
      <div class="flex flex-1 items-center gap-2">
        <span class="text-sm font-semibold text-base-content/80">Window</span>
        <input
          v-model.number="windowMs"
          type="range"
          min="500"
          max="20000"
          step="500"
          class="range range-primary flex-1" />
        <span class="w-14 text-right font-mono text-sm">{{ windowSeconds.toFixed(1) }}s</span>
      </div>
    </div>

    <div class="mt-1">
      <TimeChartStream :windowMs="windowMs" />
    </div>
  </div>
</template>

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
    <div class="flex items-center justify-between">
      <h2 class="text-xl font-bold">Real-Time Plot</h2>
      <span class="badge badge-outline font-mono">
        {{ approxUpdateHz ? `${approxUpdateHz.toFixed(1)} Hz` : '— Hz' }}
      </span>
    </div>

    <div class="flex items-center gap-2">
      <span class="text-sm font-semibold text-base-content/80">Window</span>
      <input
        v-model.number="windowMs"
        type="range"
        min="500"
        max="20000"
        step="500"
        class="range range-primary range-sm flex-1" />
      <span class="w-14 text-right font-mono text-sm">{{ windowSeconds.toFixed(1) }}s</span>
    </div>

    <TimeChartStream :windowMs="windowMs" />
  </div>
</template>

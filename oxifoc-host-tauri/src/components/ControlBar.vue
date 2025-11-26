<script setup lang="ts">
import { computed, ref } from 'vue'
import { storeToRefs } from 'pinia'
import { useStreamStore } from '../stores/streamStore'
import { commands } from '../bindings'

const streamStore = useStreamStore()
const { latestSample } = storeToRefs(streamStore)

// Motor state
const isRunning = ref(false)
const dutyCycle = ref(10)
const isLoading = ref(false)

// Format VBUS voltage (millivolts to volts)
const vbusVolts = computed(() => {
  if (!latestSample.value) return '—'
  return (latestSample.value.vbusMv / 1000).toFixed(2)
})

// Format FET temperature (0.1°C units to °C)
const fetTempC = computed(() => {
  if (!latestSample.value) return '—'
  return (latestSample.value.fetTempCX10 / 10).toFixed(1)
})

const startMotor = async () => {
  isLoading.value = true
  const result = await commands.motorStart(dutyCycle.value)
  if (result.status === 'ok') {
    isRunning.value = true
  }
  isLoading.value = false
}

const stopMotor = async () => {
  isLoading.value = true
  const result = await commands.motorStop()
  if (result.status === 'ok') {
    isRunning.value = false
  }
  isLoading.value = false
}

const updateSpeed = async () => {
  if (!isRunning.value) return
  await commands.motorSetSpeed(dutyCycle.value)
}
</script>

<template>
  <div class="fixed bottom-0 left-0 right-0 bg-base-300 border-t border-base-content/20 z-40">
    <div class="container mx-auto max-w-5xl">
      <div class="flex items-center justify-between px-4 py-2 gap-4">
        <!-- Left: Telemetry Indicators -->
        <div class="flex items-center gap-4">
          <div class="flex flex-col items-center min-w-16">
            <span class="text-xs text-base-content/60 uppercase">VBUS</span>
            <span class="font-mono text-sm font-semibold">{{ vbusVolts }} V</span>
          </div>
          <div class="divider divider-horizontal mx-0"></div>
          <div class="flex flex-col items-center min-w-16">
            <span class="text-xs text-base-content/60 uppercase">Temp</span>
            <span class="font-mono text-sm font-semibold">{{ fetTempC }} °C</span>
          </div>
        </div>

        <!-- Center: Duty Cycle Control -->
        <div class="flex items-center gap-3">
          <span class="text-xs text-base-content/60 uppercase">Duty</span>
          <input
            v-model.number="dutyCycle"
            type="range"
            min="0"
            max="100"
            step="5"
            class="range range-sm range-primary w-32"
            @change="updateSpeed" />
          <span class="font-mono text-sm font-semibold w-12 text-right">{{ dutyCycle }}%</span>
        </div>

        <!-- Right: Motor Controls -->
        <div class="flex items-center gap-2">
          <button
            class="btn btn-sm btn-success"
            :class="{ 'btn-disabled': isRunning || isLoading }"
            :disabled="isRunning || isLoading"
            @click="startMotor">
            <svg
              xmlns="http://www.w3.org/2000/svg"
              class="h-4 w-4"
              viewBox="0 0 20 20"
              fill="currentColor">
              <path
                fill-rule="evenodd"
                d="M10 18a8 8 0 100-16 8 8 0 000 16zM9.555 7.168A1 1 0 008 8v4a1 1 0 001.555.832l3-2a1 1 0 000-1.664l-3-2z"
                clip-rule="evenodd" />
            </svg>
            Start
          </button>
          <button
            class="btn btn-sm btn-error"
            :class="{ 'btn-disabled': !isRunning || isLoading }"
            :disabled="!isRunning || isLoading"
            @click="stopMotor">
            <svg
              xmlns="http://www.w3.org/2000/svg"
              class="h-4 w-4"
              viewBox="0 0 20 20"
              fill="currentColor">
              <path
                fill-rule="evenodd"
                d="M10 18a8 8 0 100-16 8 8 0 000 16zM8 7a1 1 0 00-1 1v4a1 1 0 001 1h4a1 1 0 001-1V8a1 1 0 00-1-1H8z"
                clip-rule="evenodd" />
            </svg>
            Stop
          </button>
        </div>
      </div>
    </div>
  </div>
</template>

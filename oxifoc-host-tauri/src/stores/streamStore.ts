import { defineStore, acceptHMRUpdate } from 'pinia'
import { Channel } from '@tauri-apps/api/core'
import { computed, ref, shallowRef, triggerRef } from 'vue'
import { commands, type AdcSample } from '../bindings'
import { error as logError } from '@tauri-apps/plugin-log'

const RETENTION_MS = 25_000 // Must be >= max window size (20s) + buffer
const RATE_SAMPLE_WINDOW = 120

// ADC configuration for normalization
const ADC_MIDPOINT = 2048 // 12-bit ADC center
const ADC_SCALE = 2048 // Scale factor for normalization

/**
 * Extended sample with client-side timestamp and normalized values.
 * timestampMs is added when samples arrive for time-based windowing.
 */
export interface StreamSample {
  timestampMs: number
  values: [number, number, number] // Normalized phase currents (ia, ib, ic)
  raw: AdcSample // Original sample for telemetry access
}

/**
 * Normalize raw ADC value to -1.0 to 1.0 range.
 * ADC values are 12-bit (0-4095), centered around 2048.
 */
const normalizeAdcValue = (raw: number): number => {
  return (raw - ADC_MIDPOINT) / ADC_SCALE
}

export const useStreamStore = defineStore('stream', () => {
  // Use shallowRef for samples array to avoid Vue deeply tracking every mutation
  // This prevents massive performance degradation at high sample rates (60Hz)
  const samples = shallowRef<StreamSample[]>([])
  const isStreaming = ref(false)
  const activeChannel = shallowRef<Channel<AdcSample> | null>(null)
  let startPromise: Promise<void> | null = null

  /**
   * Counter that increments with each incoming sample.
   * Used as a reliable watch target for chart components.
   */
  const sampleCount = ref(0)

  const latestSample = computed(() => samples.value[samples.value.length - 1] ?? null)

  const approxUpdateHz = computed(() => {
    if (samples.value.length < 2) return null as number | null
    const window = samples.value.slice(-RATE_SAMPLE_WINDOW)
    if (window.length < 2) return null
    const first = window[0]
    const last = window[window.length - 1]
    const deltaMs = last.timestampMs - first.timestampMs
    if (deltaMs <= 0) return null
    return ((window.length - 1) * 1000) / deltaMs
  })

  const handleSample = (sample: AdcSample) => {
    const timestampMs = performance.now()
    const streamSample: StreamSample = {
      timestampMs,
      values: [
        normalizeAdcValue(sample.ia),
        normalizeAdcValue(sample.ib),
        normalizeAdcValue(sample.ic),
      ],
      raw: sample,
    }

    samples.value.push(streamSample)
    sampleCount.value++

    // Trim old samples based on timestamp
    const cutoff = timestampMs - RETENTION_MS
    while (samples.value.length && samples.value[0].timestampMs < cutoff) {
      samples.value.shift()
    }

    // Trigger reactivity manually since we're using shallowRef
    // triggerRef is the proper way to notify Vue of shallow ref changes
    triggerRef(samples)
  }

  const clearSamples = () => {
    samples.value = []
    sampleCount.value = 0
  }

  /**
   * Check and update the device connection status.
   */
  const checkConnection = async () => {
    return await commands.isDeviceConnected()
  }

  /**
   * Wait for device connection with timeout.
   */
  const waitForDevice = async (timeoutSecs: number = 5) => {
    return await commands.waitForDevice(timeoutSecs)
  }

  const startStream = async () => {
    const channel = new Channel<AdcSample>()
    channel.onmessage = (sample) => handleSample(sample)

    const result = await commands.startAdcStream(channel)
    if (result.status === 'error') {
      throw new Error(String(result.error))
    }

    activeChannel.value = channel
    isStreaming.value = true
  }

  const stopStream = () => {
    if (activeChannel.value) {
      activeChannel.value.onmessage = () => {}
    }
    // Drop the reference so the Rust sender stops on send failure.
    activeChannel.value = null
    isStreaming.value = false
  }

  const ensureStream = async () => {
    if (isStreaming.value) return
    if (!startPromise) {
      startPromise = startStream()
        .catch((err) => {
          logError(`Failed to start stream: ${err}`)
          stopStream()
          throw err
        })
        .finally(() => {
          startPromise = null
        })
    }
    return startPromise
  }

  /**
   * Get samples within a time window from the latest sample.
   */
  const windowedSamples = (windowMs: number) => {
    const latestTs = latestSample.value?.timestampMs
    if (!latestTs) return [] as StreamSample[]
    const cutoff = latestTs - windowMs
    return samples.value.filter((sample) => sample.timestampMs >= cutoff)
  }

  const reset = () => {
    clearSamples()
    activeChannel.value = null
    isStreaming.value = false
    startPromise = null
  }

  return {
    samples,
    latestSample,
    isStreaming,
    approxUpdateHz,
    sampleCount,
    checkConnection,
    waitForDevice,
    ensureStream,
    windowedSamples,
    reset,
  }
})

if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useStreamStore, import.meta.hot))
}

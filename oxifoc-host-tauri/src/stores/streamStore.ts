import { defineStore, acceptHMRUpdate } from 'pinia'
import { Channel } from '@tauri-apps/api/core'
import { computed, ref, shallowRef } from 'vue'
import { commands, type AdcSample } from '../bindings'
import { error as logError } from '@tauri-apps/plugin-log'

const RETENTION_SAMPLES = 1500 // Keep ~25 seconds at 60Hz
const RATE_SAMPLE_WINDOW = 120

export const useStreamStore = defineStore('stream', () => {
  const samples = ref<AdcSample[]>([])
  const isStreaming = ref(false)
  const activeChannel = shallowRef<Channel<AdcSample> | null>(null)
  let startPromise: Promise<void> | null = null

  /**
   * Counter that increments with each incoming sample.
   * Used by TimeChartStream.vue as a reliable watch target.
   */
  const sampleCount = ref(0)

  const latestSample = computed(() => samples.value[samples.value.length - 1] ?? null)

  const approxUpdateHz = computed(() => {
    if (samples.value.length < 2) return null as number | null
    const window = samples.value.slice(-RATE_SAMPLE_WINDOW)
    if (window.length < 2) return null
    const first = window[0]
    const last = window[window.length - 1]
    // Use seq difference to calculate rate (seq increments per sample)
    const seqDelta = last.seq - first.seq
    if (seqDelta <= 0) return null
    // Assuming roughly constant sample rate, estimate Hz
    // This is an approximation since we don't have timestamps
    return (seqDelta / (window.length - 1)) * 60 // Assuming ~60Hz base rate
  })

  const handleSample = (sample: AdcSample) => {
    samples.value.push(sample)
    sampleCount.value++
    // Trim old samples based on count (not timestamp, since AdcSample has no timestamp)
    while (samples.value.length > RETENTION_SAMPLES) {
      samples.value.shift()
    }
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
   * Get the most recent N samples.
   */
  const recentSamples = (count: number) => {
    return samples.value.slice(-count)
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
    recentSamples,
    reset,
  }
})

if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useStreamStore, import.meta.hot))
}

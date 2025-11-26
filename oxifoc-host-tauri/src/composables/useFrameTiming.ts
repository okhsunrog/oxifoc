import { ref, onUnmounted } from 'vue'

const SAMPLE_SIZE = 60 // Average over last 60 frames

export function useFrameTiming() {
  const frameTimes = ref<number[]>([])
  const avgFrameTimeMs = ref<number | null>(null)
  const maxFps = ref<number | null>(null)

  /**
   * Measures actual render cost by wrapping the update function.
   * Call with a function that performs the update work.
   */
  const measureFrame = (updateFn: () => void) => {
    const start = performance.now()
    updateFn()
    const elapsed = performance.now() - start

    frameTimes.value.push(elapsed)

    // Keep only last SAMPLE_SIZE frames
    if (frameTimes.value.length > SAMPLE_SIZE) {
      frameTimes.value.shift()
    }

    // Calculate average
    if (frameTimes.value.length > 0) {
      const sum = frameTimes.value.reduce((a, b) => a + b, 0)
      avgFrameTimeMs.value = sum / frameTimes.value.length
      maxFps.value = 1000 / avgFrameTimeMs.value
    }
  }

  const reset = () => {
    frameTimes.value = []
    avgFrameTimeMs.value = null
    maxFps.value = null
  }

  onUnmounted(() => {
    reset()
  })

  return {
    measureFrame,
    reset,
    avgFrameTimeMs,
    maxFps,
  }
}

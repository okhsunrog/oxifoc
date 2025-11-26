<script setup lang="ts">
import { ref, onMounted, onUnmounted, watch, nextTick } from 'vue'
import { Terminal } from '@xterm/xterm'
import { FitAddon } from '@xterm/addon-fit'
import { useTerminalStore } from '../../stores/terminalStore'
import { THEMES } from '../../constants/themes'
import { commands, type LogLevel } from '../../bindings'
import '@xterm/xterm/css/xterm.css'
import type { ITheme } from '@xterm/xterm'

// --- Refs and Store ---
const terminal = ref<Terminal | null>(null)
const terminalElement = ref<HTMLElement | null>(null)
let fitAddon: FitAddon | null = null
const terminalStore = useTerminalStore()

// Track component mount state to prevent duplicate initialization
const isMounted = ref(false)

// --- Log Level Controls ---
const LOG_LEVELS: LogLevel[] = ['trace', 'debug', 'info', 'warn', 'error', 'off']
const hostLevel = ref<LogLevel>('info')
const deviceLevel = ref<LogLevel>('info')

// Load initial log levels from backend
const loadLogLevels = async () => {
  try {
    const [host, device] = await commands.getLogLevels()
    hostLevel.value = host
    deviceLevel.value = device
  } catch (e) {
    console.error('Failed to load log levels:', e)
  }
}

// Handle log level changes
const onHostLevelChange = async (event: Event) => {
  const level = (event.target as HTMLSelectElement).value as LogLevel
  try {
    await commands.setHostLogLevel(level)
    hostLevel.value = level
  } catch (e) {
    console.error('Failed to set host log level:', e)
  }
}

const onDeviceLevelChange = async (event: Event) => {
  const level = (event.target as HTMLSelectElement).value as LogLevel
  try {
    await commands.setDeviceLogLevel(level)
    deviceLevel.value = level
  } catch (e) {
    console.error('Failed to set device log level:', e)
  }
}

// --- Theme Logic ---
const isLightTheme = ref(false)

// Helper: Get CSS Variable Value
const getCssVarValue = (varName: string): string => {
  if (typeof window === 'undefined' || !document?.documentElement) return ''
  const value = getComputedStyle(document.documentElement).getPropertyValue(varName).trim()
  return value
}

// Helper: Apply Opacity to OKLCH colors
const applyOpacity = (baseColorVarName: string, alpha: number): string => {
  const baseColor = getCssVarValue(baseColorVarName)

  if (!baseColor || !baseColor.startsWith('oklch')) {
    return `rgba(128, 128, 128, ${alpha})`
  }

  // Extract the OKLCH components (lightness, chroma, hue)
  const match = baseColor.match(/oklch\(([\d.]+)%\s+([\d.]+)\s+([\d.]+)(?:\s+\/\s+[\d.]+)?\)/)
  if (match) {
    const [, l, c, h] = match
    return `oklch(${l}% ${c} ${h} / ${alpha})`
  }

  return `rgba(128, 128, 128, ${alpha})`
}

// Theme Detection
const isCurrentThemeLight = (): boolean => {
  if (typeof document === 'undefined') return false
  const currentTheme = document.documentElement.getAttribute('data-theme')
  return currentTheme === THEMES.LIGHT
}

// Get theme colors
const getThemeColors = (): ITheme => {
  const isLight = isCurrentThemeLight()
  isLightTheme.value = isLight

  const baseVars = {
    background: '--color-base-200',
    foreground: '--color-base-content',
    cursor: '--color-primary',
    selection: '--color-primary',
    neutral: '--color-neutral',
    base100: '--color-base-100',
    base300: '--color-base-300',
  }

  let ansiColors: Partial<ITheme> = {}

  if (isLight) {
    // Light theme colors
    ansiColors = {
      // Standard colors
      red: getCssVarValue('--color-red-600'),
      green: getCssVarValue('--color-green-600'),
      yellow: getCssVarValue('--color-amber-500'),
      blue: getCssVarValue('--color-blue-600'),
      magenta: getCssVarValue('--color-purple-500'),
      cyan: getCssVarValue('--color-cyan-500'),

      // Bright variants
      brightRed: getCssVarValue('--color-red-500'),
      brightGreen: getCssVarValue('--color-green-500'),
      brightYellow: getCssVarValue('--color-amber-400'),
      brightBlue: getCssVarValue('--color-blue-500'),
      brightMagenta: getCssVarValue('--color-purple-400'),
      brightCyan: getCssVarValue('--color-cyan-400'),
    }
  } else {
    // Dark theme colors
    ansiColors = {
      red: getCssVarValue('--color-error'),
      green: getCssVarValue('--color-success'),
      yellow: getCssVarValue('--color-warning'),
      blue: getCssVarValue('--color-info'),
      magenta: getCssVarValue('--color-accent'),
      cyan: getCssVarValue('--color-secondary'),
      brightRed: getCssVarValue('--color-error'),
      brightGreen: getCssVarValue('--color-success'),
      brightYellow: getCssVarValue('--color-warning'),
      brightBlue: getCssVarValue('--color-info'),
      brightMagenta: getCssVarValue('--color-accent'),
      brightCyan: getCssVarValue('--color-secondary'),
    }
  }

  return {
    background: getCssVarValue(baseVars.background),
    foreground: getCssVarValue(baseVars.foreground),
    cursor: getCssVarValue(baseVars.cursor),
    selectionBackground: applyOpacity(baseVars.selection, 0.4),
    selectionForeground: undefined,
    black: getCssVarValue(baseVars.neutral),
    white: getCssVarValue(baseVars.foreground),
    brightBlack: getCssVarValue(baseVars.base300),
    brightWhite: getCssVarValue(baseVars.base100),
    ...ansiColors,
  }
}

// --- Terminal Logic ---
const dimCodeRegex = /\x1b\[2m/g

// Track the last processed message ID to avoid duplicates
let lastProcessedId = 0

// Update terminal theme
const updateTerminalTheme = () => {
  if (!terminal.value) return

  const colors = getThemeColors()
  terminal.value.options.theme = colors
  terminal.value.refresh(0, terminal.value.rows - 1)
}

// Handle window resize
let resizeTimeout: ReturnType<typeof setTimeout> | null = null
const handleResize = () => {
  if (resizeTimeout) clearTimeout(resizeTimeout)

  resizeTimeout = setTimeout(() => {
    if (fitAddon && terminalStore.isVisible && terminal.value) {
      fitAddon.fit()
    }
  }, 150)
}

// Theme change observer
let themeObserver: MutationObserver | null = null
const observeThemeChanges = () => {
  if (typeof MutationObserver === 'undefined' || !document?.documentElement) return

  themeObserver = new MutationObserver((mutations) => {
    for (const mutation of mutations) {
      if (mutation.type === 'attributes' && mutation.attributeName === 'data-theme') {
        nextTick(updateTerminalTheme)
      }
    }
  })

  themeObserver.observe(document.documentElement, {
    attributes: true,
    attributeFilter: ['data-theme'],
  })
}

// Initialize terminal
const initializeTerminal = async () => {
  if (!terminalElement.value || isMounted.value) return

  try {
    const initialColors = getThemeColors()

    terminal.value = new Terminal({
      fontFamily: 'Menlo, Monaco, "Courier New", monospace',
      fontSize: 13,
      theme: initialColors,
      cursorBlink: true,
      convertEol: true,
      scrollback: 10000,
      windowsMode: false,
      allowProposedApi: true,
    })

    // Create and load the fit addon
    fitAddon = new FitAddon()
    terminal.value.loadAddon(fitAddon)

    // Open the terminal in the DOM element
    terminal.value.open(terminalElement.value)

    // Process existing messages
    const msgs = terminalStore.messages
    if (msgs.length > 0) {
      lastProcessedId = Math.max(...msgs.map((msg) => msg.id))

      const currentIsLight = isLightTheme.value
      for (const msg of msgs) {
        let content = msg.content
        if (currentIsLight) {
          content = content.replace(dimCodeRegex, '')
        }
        terminal.value.write(content)
      }

      await nextTick()
      terminal.value.scrollToBottom()
    }

    // Initial fit if visible
    if (terminalStore.isVisible) {
      setTimeout(() => {
        try {
          fitAddon?.fit()
        } catch (e) {
          console.error('Initial fit error:', e)
        }
      }, 100)
    }

    // Set up event listeners and observers
    window.addEventListener('resize', handleResize)
    observeThemeChanges()

    // Mark as mounted
    isMounted.value = true

    // Set up HMR handling
    if (import.meta.hot) {
      import.meta.hot.dispose(() => {
        try {
          // Clean up in reverse order of creation

          // 1. Remove event listeners
          window.removeEventListener('resize', handleResize)

          // 2. Clear any pending timeouts
          if (resizeTimeout) {
            clearTimeout(resizeTimeout)
            resizeTimeout = null
          }

          // 3. Disconnect observers
          if (themeObserver) {
            themeObserver.disconnect()
            themeObserver = null
          }

          // 4. Set fitAddon to null BEFORE disposing terminal
          // This prevents terminal from trying to dispose the addon
          fitAddon = null

          // 5. Dispose terminal
          if (terminal.value) {
            try {
              terminal.value.dispose()
            } catch (e) {
              // Check if this is the addon error we expect
              if (e instanceof Error && e.message.includes('addon that has not been loaded')) {
                console.log('Ignoring expected addon disposal error during HMR')
              } else {
                console.error('Error during terminal disposal:', e)
              }
            } finally {
              terminal.value = null
            }
          }

          // 6. Reset mounted flag
          isMounted.value = false

          console.log('Terminal component successfully cleaned up for HMR')
        } catch (error) {
          console.error('Error during HMR cleanup:', error)
        }
      })
    }
  } catch (initError) {
    console.error('Terminal initialization failed:', initError)
    cleanupTerminalResources()
  }
}

// Watch for new messages
watch(
  () => terminalStore.messages,
  (messages) => {
    if (!terminal.value || !isMounted.value) return

    const newMessages = messages.filter((msg) => msg.id > lastProcessedId)
    if (newMessages.length === 0) return

    const buffer = terminal.value.buffer.active
    const isNearBottom = buffer.viewportY + terminal.value.rows >= buffer.length - 1

    // Update last processed ID
    lastProcessedId = Math.max(...newMessages.map((msg) => msg.id))

    // Process new messages
    const currentIsLight = isLightTheme.value
    for (const msg of newMessages) {
      let content = msg.content
      if (currentIsLight) {
        content = content.replace(dimCodeRegex, '')
      }
      terminal.value.write(content)
    }

    // Auto-scroll if near bottom
    if (isNearBottom) {
      nextTick(() => terminal.value?.scrollToBottom())
    }
  },
  { deep: true },
)

// Watch for visibility changes
watch(
  () => terminalStore.isVisible,
  (isVisible) => {
    if (isVisible && fitAddon && isMounted.value) {
      nextTick(() => {
        setTimeout(() => {
          try {
            fitAddon?.fit()
          } catch (e) {
            console.error('Visibility fit error:', e)
          }
        }, 50)
      })
    }
  },
)

onMounted(async () => {
  try {
    await nextTick()
    // Log listener is already initialized in main.ts
    await initializeTerminal()
    // Load current log levels from backend
    await loadLogLevels()
  } catch (error) {
    console.error('Failed to initialize terminal:', error)
  }
})

// Comprehensive cleanup function that handles all terminal resources
const cleanupTerminalResources = () => {
  try {
    // 1. Remove event listeners
    if (typeof window !== 'undefined') {
      window.removeEventListener('resize', handleResize)
    }

    // 2. Clear any pending timeouts
    if (resizeTimeout) {
      clearTimeout(resizeTimeout)
      resizeTimeout = null
    }

    // 3. Disconnect observers
    if (themeObserver) {
      themeObserver.disconnect()
      themeObserver = null
    }

    // 4. Set fitAddon to null BEFORE disposing terminal
    // This prevents terminal from trying to use the addon during disposal
    fitAddon = null

    // 5. Dispose terminal
    if (terminal.value) {
      try {
        terminal.value.dispose()
      } catch (e) {
        // Check if this is the addon error we expect
        if (e instanceof Error && e.message.includes('addon that has not been loaded')) {
          console.debug('Ignoring expected addon disposal error')
        } else {
          console.error('Error during terminal disposal:', e)
        }
      } finally {
        terminal.value = null
      }
    }

    // 6. Reset mounted flag
    isMounted.value = false

    console.log('Terminal resources cleaned up successfully')
  } catch (error) {
    console.error('Error during terminal cleanup:', error)
  }
}

onUnmounted(() => {
  cleanupTerminalResources()
})

if (import.meta.hot) {
  import.meta.hot.dispose(() => {
    cleanupTerminalResources()
  })
}
</script>

<template>
  <div class="card bg-base-100 shadow-xl">
    <div class="card-body p-3 md:p-4">
      <div class="flex flex-wrap items-center justify-between gap-2 mb-2">
        <h2 class="card-title text-sm md:text-base">Terminal</h2>
        <div class="flex items-center gap-3 text-xs">
          <label class="flex items-center gap-1">
            <span class="opacity-70">Host:</span>
            <select
              class="select select-xs select-bordered"
              :value="hostLevel"
              @change="onHostLevelChange">
              <option v-for="level in LOG_LEVELS" :key="level" :value="level">
                {{ level }}
              </option>
            </select>
          </label>
          <label class="flex items-center gap-1">
            <span class="opacity-70">Device:</span>
            <select
              class="select select-xs select-bordered"
              :value="deviceLevel"
              @change="onDeviceLevelChange">
              <option v-for="level in LOG_LEVELS" :key="level" :value="level">
                {{ level }}
              </option>
            </select>
          </label>
        </div>
      </div>
      <div class="h-80 w-full overflow-hidden rounded-md border border-base-300">
        <div ref="terminalElement" class="terminal-container h-full w-full"></div>
      </div>
    </div>
  </div>
</template>

<style scoped>
.terminal-container {
  overflow: hidden;
}

.terminal-container :deep(.xterm) {
  padding: 0.4rem 0.6rem;
  height: 100% !important;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-rendering: optimizeLegibility;
}

.terminal-container :deep(.xterm-viewport) {
  background-color: transparent !important;
  overflow-y: auto;
  width: 100% !important;
  scrollbar-width: thin;
  scrollbar-color: var(--color-neutral, #555) var(--color-base-300, #ddd);
}

.terminal-container :deep(.xterm-viewport::-webkit-scrollbar) {
  width: 8px;
  height: 8px;
}

.terminal-container :deep(.xterm-viewport::-webkit-scrollbar-track) {
  background: var(--color-base-300, #ddd);
  border-radius: 4px;
}

.terminal-container :deep(.xterm-viewport::-webkit-scrollbar-thumb) {
  background-color: var(--color-neutral, #555);
  border-radius: 4px;
  border: 2px solid var(--color-base-300, #ddd);
}

.terminal-container :deep(.xterm-viewport::-webkit-scrollbar-thumb:hover) {
  background-color: var(--color-neutral-focus, #777);
}
</style>

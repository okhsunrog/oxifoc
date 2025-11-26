// src/stores/terminalStore.ts
import { defineStore, acceptHMRUpdate } from 'pinia'
import { ref } from 'vue'
import { events } from '../bindings'
import { warn, debug, trace, info, error } from '@tauri-apps/plugin-log'

export interface TerminalMessage {
  id: number
  content: string
  timestamp: Date
}

// Set up console forwarding to Tauri's logging system
function setupConsoleForwarding() {
  const setupForwarder = (
    fnName: 'log' | 'debug' | 'info' | 'warn' | 'error',
    logger: (message: string) => Promise<void>,
  ) => {
    const original = console[fnName]
    console[fnName] = (...args) => {
      original(...args)

      const message = args
        .map((arg) => {
          if (arg instanceof Error) {
            return `${arg.name}: ${arg.message}\nStack: ${arg.stack || 'No stack trace available'}`
          } else if (typeof arg === 'object') {
            try {
              return JSON.stringify(arg)
            } catch {
              return '[Object]'
            }
          } else {
            return String(arg)
          }
        })
        .join(' ')

      logger(message).catch(() => {})
    }
  }

  setupForwarder('log', trace)
  setupForwarder('debug', debug)
  setupForwarder('info', info)
  setupForwarder('warn', warn)
  setupForwarder('error', error)
}

// Set up console forwarding immediately
setupConsoleForwarding()

export const useTerminalStore = defineStore(
  'terminal',
  () => {
    // State
    const messages = ref<TerminalMessage[]>([])
    const nextId = ref(1)
    const isVisible = ref(true)
    const backendListenerInitialized = ref(false)
    let unlistenFunction: (() => void) | null = null

    // Toggle terminal visibility
    function toggleVisibility() {
      isVisible.value = !isVisible.value
    }

    // Initialize log listener
    async function initLogListener() {
      if (backendListenerInitialized.value) return

      try {
        // Clean up existing listener
        if (unlistenFunction) {
          unlistenFunction()
          unlistenFunction = null
        }

        // Set up new listener
        unlistenFunction = await events.logEvent.listen((event) => {
          // Skip messages containing "task queue exceeded allotted deadline"
          if (event.payload.message.includes('task queue exceeded allotted deadline')) {
            return
          }

          addMessage(event.payload.message)
        })

        backendListenerInitialized.value = true
      } catch (err) {
        error(`Error initializing log listener: ${err}`)
      }
    }

    // Add message
    function addMessage(content: string) {
      if (!content.trim()) return

      messages.value.push({
        id: nextId.value++,
        content,
        timestamp: new Date(),
      })
    }

    // Clean up function
    function cleanup() {
      if (unlistenFunction) {
        debug('Cleaning up log event listener')
        unlistenFunction()
        unlistenFunction = null
        backendListenerInitialized.value = false
      }
    }

    // Clear all messages
    function clear() {
      messages.value = []
    }

    return {
      messages,
      nextId,
      isVisible,
      backendListenerInitialized,
      toggleVisibility,
      initLogListener,
      addMessage,
      clear,
      cleanup,
    }
  },
  {
    persist: {
      pick: ['isVisible'],
    },
  },
)

// Enable HMR for this store
if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useTerminalStore, import.meta.hot))
}

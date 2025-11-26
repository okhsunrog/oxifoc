import { createApp } from 'vue'
import { createPinia } from 'pinia'
import piniaPluginPersistedstate from 'pinia-plugin-persistedstate'
import App from './App.vue'

import './style.css'
import '@xterm/xterm/css/xterm.css'

const app = createApp(App)
const pinia = createPinia()
pinia.use(piniaPluginPersistedstate)

app.use(pinia)

// Initialize terminal store BEFORE mounting to catch early logs
import { useTerminalStore } from './stores/terminalStore'
const terminalStore = useTerminalStore()

// Wait for listener to be ready before mounting
;(async () => {
  await terminalStore.initLogListener()
  app.mount('#app')
})()

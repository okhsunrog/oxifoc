<script setup lang="ts">
import ThemeToggle from './components/common/ThemeToggle.vue'
import TerminalDisplay from './components/terminal/TerminalDisplay.vue'
import ChartSwitcher from './components/ChartSwitcher.vue'
import ConnectionScreen from './components/ConnectionScreen.vue'
import { useTerminalStore } from './stores/terminalStore'
import { useConnectionStore } from './stores/connectionStore'

const terminalStore = useTerminalStore()
const connectionStore = useConnectionStore()
</script>

<template>
  <!-- Connection Screen (shown when not connected) -->
  <ConnectionScreen v-if="!connectionStore.isConnected" />

  <!-- Main App (shown when connected) -->
  <main v-else class="min-h-screen bg-base-200 p-8">
    <div class="container mx-auto max-w-5xl">
      <div class="flex justify-between items-center mb-8">
        <div class="flex items-center gap-4">
          <h1 class="text-3xl font-bold">Oxifoc</h1>
          <div class="badge badge-success gap-2">
            <span class="w-2 h-2 bg-success-content rounded-full animate-pulse"></span>
            Connected
          </div>
        </div>
        <div class="flex items-center gap-2">
          <button class="btn btn-sm btn-ghost" @click="connectionStore.disconnect()">
            Disconnect
          </button>
          <ThemeToggle />
        </div>
      </div>

      <ChartSwitcher />

      <div class="mt-6">
        <div class="flex justify-between items-center mb-2">
          <button class="btn btn-sm" @click="terminalStore.toggleVisibility()">
            {{ terminalStore.isVisible ? 'Hide Terminal' : 'Show Terminal' }}
          </button>
        </div>
        <div v-show="terminalStore.isVisible">
          <TerminalDisplay />
        </div>
      </div>
    </div>
  </main>
</template>

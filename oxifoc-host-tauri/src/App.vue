<script setup lang="ts">
import ThemeToggle from './components/common/ThemeToggle.vue'
import TerminalDisplay from './components/terminal/TerminalDisplay.vue'
import ConnectionCard from './components/ConnectionCard.vue'
import MainCard from './components/MainCard.vue'
import ControlBar from './components/ControlBar.vue'
import { useTerminalStore } from './stores/terminalStore'
import { useConnectionStore } from './stores/connectionStore'

const terminalStore = useTerminalStore()
const connectionStore = useConnectionStore()
</script>

<template>
  <main class="min-h-screen bg-base-200 p-8" :class="{ 'pb-20': connectionStore.isConnected }">
    <div class="container mx-auto max-w-5xl">
      <!-- Header -->
      <div class="flex justify-between items-center mb-8">
        <h1 class="text-3xl font-bold">Oxifoc</h1>
        <ThemeToggle />
      </div>

      <!-- Main Content Card -->
      <ConnectionCard v-if="!connectionStore.isConnected" />
      <MainCard v-else />

      <!-- Terminal Section -->
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

    <!-- Bottom Control Bar (only when connected) -->
    <ControlBar v-if="connectionStore.isConnected" />
  </main>
</template>

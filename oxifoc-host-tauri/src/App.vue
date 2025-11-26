<script setup lang="ts">
import ThemeToggle from './components/common/ThemeToggle.vue'
import TerminalDisplay from './components/terminal/TerminalDisplay.vue'
import ChartSwitcher from './components/ChartSwitcher.vue'
import { useTerminalStore } from './stores/terminalStore'
const terminalStore = useTerminalStore()
</script>

<template>
  <main class="min-h-screen bg-base-200 p-8">
    <div class="container mx-auto max-w-5xl">
      <div class="flex justify-between items-center mb-8">
        <h1 class="text-3xl font-bold">Hello World App</h1>
        <ThemeToggle />
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

<script setup lang="ts">
import { onMounted } from 'vue'
import { useConnectionStore } from '../stores/connectionStore'

const store = useConnectionStore()

onMounted(() => {
  store.refreshDevices()
})
</script>

<template>
  <div class="card bg-base-100 shadow-xl">
    <div class="card-body">
      <h2 class="card-title text-xl mb-4">Connect to Device</h2>

      <!-- Transport Selection Tabs -->
      <div role="tablist" class="tabs tabs-boxed mb-4">
        <a
          role="tab"
          class="tab"
          :class="{ 'tab-active': store.selectedTransport === 'serial' }"
          @click="store.selectTransport('serial')">
          Serial (UART)
        </a>
        <a
          role="tab"
          class="tab"
          :class="{ 'tab-active': store.selectedTransport === 'rtt' }"
          @click="store.selectTransport('rtt')">
          RTT (Debug Probe)
        </a>
      </div>

      <!-- Error Alert -->
      <div v-if="store.connectionError" class="alert alert-error mb-4">
        <svg
          xmlns="http://www.w3.org/2000/svg"
          class="stroke-current shrink-0 h-6 w-6"
          fill="none"
          viewBox="0 0 24 24">
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            stroke-width="2"
            d="M10 14l2-2m0 0l2-2m-2 2l-2-2m2 2l2 2m7-2a9 9 0 11-18 0 9 9 0 0118 0z" />
        </svg>
        <span>{{ store.connectionError }}</span>
      </div>

      <!-- Serial Port Selection -->
      <div v-if="store.selectedTransport === 'serial'" class="space-y-4">
        <div class="flex justify-between items-center">
          <h3 class="text-lg font-semibold">Serial Ports</h3>
          <div class="flex items-center gap-3">
            <label class="label cursor-pointer gap-2">
              <input
                type="checkbox"
                v-model="store.showUsbSerialOnly"
                class="checkbox checkbox-sm" />
              <span class="label-text text-sm">USB-Serial only</span>
            </label>
            <button
              class="btn btn-sm btn-ghost"
              :class="{ loading: store.isLoadingSerialPorts }"
              @click="store.refreshSerialPorts()"
              :disabled="store.isLoadingSerialPorts">
              <svg
                xmlns="http://www.w3.org/2000/svg"
                class="h-4 w-4"
                fill="none"
                viewBox="0 0 24 24"
                stroke="currentColor">
                <path
                  stroke-linecap="round"
                  stroke-linejoin="round"
                  stroke-width="2"
                  d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" />
              </svg>
            </button>
          </div>
        </div>

        <!-- Serial Port Cards -->
        <div v-if="store.isLoadingSerialPorts" class="flex justify-center py-4">
          <span class="loading loading-spinner loading-md"></span>
        </div>

        <div
          v-else-if="store.filteredSerialPorts.length === 0"
          class="text-center py-4 text-base-content/60 text-sm">
          <p>
            {{ store.showUsbSerialOnly ? 'No USB-Serial devices found' : 'No serial ports found' }}
          </p>
        </div>

        <div v-else class="grid gap-2 max-h-48 overflow-y-auto">
          <div
            v-for="port in store.filteredSerialPorts"
            :key="port.path"
            class="card card-compact bg-base-200 cursor-pointer hover:bg-base-300 transition-colors"
            :class="{ 'ring-2 ring-primary': store.selectedSerialPath === port.path }"
            @click="store.selectSerialPort(port.path)">
            <div class="card-body py-2">
              <div class="flex justify-between items-center">
                <div>
                  <h4 class="font-mono font-bold text-sm">{{ port.path }}</h4>
                  <p v-if="port.product" class="text-xs text-base-content/70">
                    {{ port.product }}
                  </p>
                </div>
                <div class="text-right text-xs text-base-content/50">
                  <p v-if="port.vid && port.pid">
                    {{ port.vid.toString(16).padStart(4, '0') }}:{{
                      port.pid.toString(16).padStart(4, '0')
                    }}
                  </p>
                </div>
              </div>
            </div>
          </div>
        </div>

        <!-- Baud Rate -->
        <div class="form-control">
          <label class="label py-1">
            <span class="label-text text-sm">Baud Rate</span>
          </label>
          <select
            v-model.number="store.serialBaudRate"
            class="select select-bordered select-sm w-full max-w-xs">
            <option :value="115200">115200</option>
            <option :value="230400">230400</option>
            <option :value="460800">460800</option>
            <option :value="921600">921600</option>
            <option :value="1000000">1000000</option>
            <option :value="2000000">2000000</option>
          </select>
        </div>
      </div>

      <!-- Debug Probe Selection -->
      <div v-if="store.selectedTransport === 'rtt'" class="space-y-4">
        <div class="flex justify-between items-center">
          <h3 class="text-lg font-semibold">Debug Probes</h3>
          <button
            class="btn btn-sm btn-ghost"
            :class="{ loading: store.isLoadingProbes }"
            @click="store.refreshProbes()"
            :disabled="store.isLoadingProbes">
            <svg
              xmlns="http://www.w3.org/2000/svg"
              class="h-4 w-4"
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor">
              <path
                stroke-linecap="round"
                stroke-linejoin="round"
                stroke-width="2"
                d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" />
            </svg>
          </button>
        </div>

        <!-- Debug Probe Cards -->
        <div v-if="store.isLoadingProbes" class="flex justify-center py-4">
          <span class="loading loading-spinner loading-md"></span>
        </div>

        <div
          v-else-if="store.debugProbes.length === 0"
          class="text-center py-4 text-base-content/60 text-sm">
          No debug probes found
        </div>

        <div v-else class="grid gap-2 max-h-48 overflow-y-auto">
          <div
            v-for="probe in store.debugProbes"
            :key="probe.identifier"
            class="card card-compact bg-base-200 cursor-pointer hover:bg-base-300 transition-colors"
            :class="{ 'ring-2 ring-primary': store.selectedProbeId === probe.identifier }"
            @click="store.selectProbe(probe.identifier)">
            <div class="card-body py-2">
              <div class="flex justify-between items-center">
                <div>
                  <h4 class="font-bold text-sm">{{ probe.probeType }}</h4>
                  <p class="text-xs font-mono text-base-content/70">
                    {{ probe.vid.toString(16).padStart(4, '0') }}:{{
                      probe.pid.toString(16).padStart(4, '0')
                    }}
                  </p>
                </div>
                <div class="text-right text-xs text-base-content/50">
                  <p v-if="probe.serialNumber">S/N: {{ probe.serialNumber }}</p>
                </div>
              </div>
            </div>
          </div>
        </div>

        <!-- Chip Selection -->
        <div class="form-control">
          <label class="label py-1">
            <span class="label-text text-sm">Target Chip</span>
          </label>
          <input
            v-model="store.rttChip"
            type="text"
            placeholder="e.g., STM32G431CBUx"
            class="input input-bordered input-sm w-full max-w-xs" />
        </div>
      </div>

      <!-- Connect Button -->
      <div class="card-actions justify-end mt-4">
        <button
          class="btn btn-primary btn-sm"
          :class="{ loading: store.isConnecting }"
          :disabled="!store.canConnect"
          @click="store.connect()">
          {{ store.isConnecting ? 'Connecting...' : 'Connect' }}
        </button>
      </div>
    </div>
  </div>
</template>

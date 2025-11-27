import { defineStore, acceptHMRUpdate } from 'pinia'
import { ref, computed } from 'vue'
import { commands, type SerialPort, type DebugProbe, type ConnectionConfig } from '../bindings'
import { error as logError, info as logInfo } from '@tauri-apps/plugin-log'

export type TransportType = 'serial' | 'rtt'

export const useConnectionStore = defineStore('connection', () => {
  // Connection state
  const isConnected = ref(false)
  const isConnecting = ref(false)
  const connectionError = ref<string | null>(null)

  // Selected transport and device
  const selectedTransport = ref<TransportType>('serial')

  // Serial configuration
  const serialPorts = ref<SerialPort[]>([])
  const selectedSerialPath = ref<string | null>(null)
  const serialBaudRate = ref(115200)

  // RTT configuration
  const debugProbes = ref<DebugProbe[]>([])
  const selectedProbeId = ref<string | null>(null)
  const rttChip = ref('STM32G431CBUx')

  // Loading states for device lists
  const isLoadingSerialPorts = ref(false)
  const isLoadingProbes = ref(false)

  // Filter preferences
  const showUsbSerialOnly = ref(true)

  // Computed helpers
  const filteredSerialPorts = computed(() => {
    if (!showUsbSerialOnly.value) {
      return serialPorts.value
    }
    // Filter to show only USB-Serial devices (ttyACM*, ttyUSB*, ttyAMA*)
    return serialPorts.value.filter((port) => {
      const path = port.path.toLowerCase()
      return path.includes('ttyacm') || path.includes('ttyusb') || path.includes('ttyama')
    })
  })

  const selectedSerialPort = computed(
    () => serialPorts.value.find((p) => p.path === selectedSerialPath.value) ?? null,
  )

  const selectedProbe = computed(
    () => debugProbes.value.find((p) => p.identifier === selectedProbeId.value) ?? null,
  )

  const canConnect = computed(() => {
    if (isConnecting.value) return false
    if (selectedTransport.value === 'serial') {
      return selectedSerialPath.value !== null
    } else {
      return selectedProbeId.value !== null && rttChip.value.trim() !== ''
    }
  })

  // Actions
  const refreshSerialPorts = async () => {
    isLoadingSerialPorts.value = true
    connectionError.value = null
    try {
      const ports = await commands.listSerialPortsCmd()
      serialPorts.value = ports
      logInfo(`Found ${ports.length} serial ports`)
      // Auto-select first port if none selected
      if (!selectedSerialPath.value && ports.length > 0) {
        selectedSerialPath.value = ports[0].path
      }
    } catch (err) {
      connectionError.value = `Failed to list serial ports: ${err}`
      logError(connectionError.value)
    } finally {
      isLoadingSerialPorts.value = false
    }
  }

  const refreshProbes = async () => {
    isLoadingProbes.value = true
    connectionError.value = null
    try {
      const probes = await commands.listProbesCmd()
      debugProbes.value = probes
      logInfo(`Found ${probes.length} debug probes`)
      // Auto-select first probe if none selected
      if (!selectedProbeId.value && probes.length > 0) {
        selectedProbeId.value = probes[0].identifier
      }
    } catch (err) {
      connectionError.value = `Failed to list debug probes: ${err}`
      logError(connectionError.value)
    } finally {
      isLoadingProbes.value = false
    }
  }

  const refreshDevices = async () => {
    await Promise.all([refreshSerialPorts(), refreshProbes()])
  }

  const connect = async () => {
    if (!canConnect.value) return

    isConnecting.value = true
    connectionError.value = null

    try {
      const config: ConnectionConfig = {
        transport: selectedTransport.value,
        serialPath: selectedTransport.value === 'serial' ? selectedSerialPath.value : null,
        baudRate: selectedTransport.value === 'serial' ? serialBaudRate.value : null,
        probe: selectedTransport.value === 'rtt' ? selectedProbeId.value : null,
        chip: selectedTransport.value === 'rtt' ? rttChip.value : null,
      }

      logInfo(`Connecting with transport: ${selectedTransport.value}`)
      const result = await commands.connectDevice(config)

      if (result.status === 'error') {
        throw new Error(String(result.error))
      }

      isConnected.value = true
      logInfo('Device connected successfully')
    } catch (err) {
      connectionError.value = `Connection failed: ${err}`
      logError(connectionError.value)
      isConnected.value = false
    } finally {
      isConnecting.value = false
    }
  }

  const disconnect = async () => {
    try {
      const result = await commands.disconnectDevice()
      if (result.status === 'error') {
        throw new Error(String(result.error))
      }
      isConnected.value = false
      logInfo('Device disconnected')
    } catch (err) {
      connectionError.value = `Disconnect failed: ${err}`
      logError(connectionError.value)
    }
  }

  const selectTransport = (transport: TransportType) => {
    selectedTransport.value = transport
  }

  const selectSerialPort = (path: string) => {
    selectedSerialPath.value = path
  }

  const selectProbe = (identifier: string) => {
    selectedProbeId.value = identifier
  }

  return {
    // State
    isConnected,
    isConnecting,
    connectionError,
    selectedTransport,
    serialPorts,
    filteredSerialPorts,
    selectedSerialPath,
    selectedSerialPort,
    serialBaudRate,
    debugProbes,
    selectedProbeId,
    selectedProbe,
    rttChip,
    isLoadingSerialPorts,
    isLoadingProbes,
    showUsbSerialOnly,
    canConnect,

    // Actions
    refreshSerialPorts,
    refreshProbes,
    refreshDevices,
    connect,
    disconnect,
    selectTransport,
    selectSerialPort,
    selectProbe,
  }
})

if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useConnectionStore, import.meta.hot))
}

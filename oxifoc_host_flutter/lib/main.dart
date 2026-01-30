import 'dart:async';
import 'package:flutter/material.dart';
import 'package:rinf/rinf.dart';
import 'src/bindings/bindings.dart';

Future<void> main() async {
  await initializeRust(assignRustSignal);
  runApp(const OxifocApp());
}

class OxifocApp extends StatelessWidget {
  const OxifocApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Oxifoc',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(
          seedColor: Colors.blue,
          brightness: Brightness.dark,
        ),
        useMaterial3: true,
      ),
      home: const HomePage(),
    );
  }
}

class HomePage extends StatefulWidget {
  const HomePage({super.key});

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  bool _isConnected = false;
  String? _statusMessage;
  StreamSubscription? _connectionSub;

  @override
  void initState() {
    super.initState();
    _connectionSub = ConnectionStatus.rustSignalStream.listen((signalPack) {
      setState(() {
        _isConnected = signalPack.message.connected;
        _statusMessage = signalPack.message.message;
      });
    });
  }

  @override
  void dispose() {
    _connectionSub?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Oxifoc'),
        actions: [
          if (_isConnected)
            IconButton(
              icon: const Icon(Icons.logout),
              onPressed: () => const Disconnect().sendSignalToRust(),
              tooltip: 'Disconnect',
            ),
        ],
      ),
      body: _isConnected ? const TelemetryView() : const ConnectionView(),
      bottomNavigationBar: _statusMessage != null
          ? Container(
              color: _isConnected
                  ? Colors.green.shade900
                  : Colors.orange.shade900,
              padding: const EdgeInsets.all(8),
              child: Text(_statusMessage!, textAlign: TextAlign.center),
            )
          : null,
    );
  }
}

// ============================================================================
// Connection View
// ============================================================================

class ConnectionView extends StatefulWidget {
  const ConnectionView({super.key});

  @override
  State<ConnectionView> createState() => _ConnectionViewState();
}

class _ConnectionViewState extends State<ConnectionView> {
  List<SerialPortInfo> _ports = [];
  List<ProbeInfo> _probes = [];
  int _selectedPortIndex = -1;
  int _selectedProbeIndex = -1;
  int _selectedBaudRate = 921600;
  bool _useRtt = false;
  String _chip = 'STM32G431CBUx';

  StreamSubscription? _portsSub;
  StreamSubscription? _probesSub;

  final _baudRates = [115200, 230400, 460800, 921600, 1000000, 2000000];

  @override
  void initState() {
    super.initState();

    _portsSub = SerialPortsList.rustSignalStream.listen((signalPack) {
      setState(() {
        _ports = signalPack.message.ports;
        _selectedPortIndex = -1;
      });
    });

    _probesSub = ProbesList.rustSignalStream.listen((signalPack) {
      setState(() {
        _probes = signalPack.message.probes;
        _selectedProbeIndex = -1;
      });
    });

    // Request initial lists
    const ListSerialPorts().sendSignalToRust();
    const ListProbes().sendSignalToRust();
  }

  @override
  void dispose() {
    _portsSub?.cancel();
    _probesSub?.cancel();
    super.dispose();
  }

  void _refresh() {
    const ListSerialPorts().sendSignalToRust();
    const ListProbes().sendSignalToRust();
  }

  void _connect() {
    if (_useRtt) {
      if (_selectedProbeIndex >= 0) {
        ConnectRtt(
          probeId: _probes[_selectedProbeIndex].identifier,
          chip: _chip,
        ).sendSignalToRust();
      }
    } else {
      if (_selectedPortIndex >= 0) {
        ConnectSerial(
          portPath: _ports[_selectedPortIndex].path,
          baudRate: _selectedBaudRate,
        ).sendSignalToRust();
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          // Transport selection
          SegmentedButton<bool>(
            segments: const [
              ButtonSegment(value: false, label: Text('Serial')),
              ButtonSegment(value: true, label: Text('RTT')),
            ],
            selected: {_useRtt},
            onSelectionChanged: (selected) {
              setState(() => _useRtt = selected.first);
            },
          ),
          const SizedBox(height: 16),

          // Refresh button
          OutlinedButton.icon(
            onPressed: _refresh,
            icon: const Icon(Icons.refresh),
            label: const Text('Refresh'),
          ),
          const SizedBox(height: 16),

          if (!_useRtt) ...[
            // Serial port selection
            Text(
              'Serial Ports',
              style: Theme.of(context).textTheme.titleMedium,
            ),
            const SizedBox(height: 8),
            if (_ports.isEmpty)
              const Card(
                child: Padding(
                  padding: EdgeInsets.all(16),
                  child: Text('No serial ports found'),
                ),
              )
            else
              ...List.generate(_ports.length, (index) {
                final port = _ports[index];
                return Card(
                  color: _selectedPortIndex == index
                      ? Theme.of(context).colorScheme.primaryContainer
                      : null,
                  child: ListTile(
                    title: Text(port.path),
                    subtitle: Text(port.product ?? 'Unknown device'),
                    selected: _selectedPortIndex == index,
                    onTap: () => setState(() => _selectedPortIndex = index),
                  ),
                );
              }),
            const SizedBox(height: 16),

            // Baud rate selection
            DropdownButtonFormField<int>(
              decoration: const InputDecoration(
                labelText: 'Baud Rate',
                border: OutlineInputBorder(),
              ),
              initialValue: _selectedBaudRate,
              items: _baudRates
                  .map(
                    (rate) =>
                        DropdownMenuItem(value: rate, child: Text('$rate')),
                  )
                  .toList(),
              onChanged: (value) {
                if (value != null) setState(() => _selectedBaudRate = value);
              },
            ),
          ] else ...[
            // RTT probe selection
            Text(
              'Debug Probes',
              style: Theme.of(context).textTheme.titleMedium,
            ),
            const SizedBox(height: 8),
            if (_probes.isEmpty)
              const Card(
                child: Padding(
                  padding: EdgeInsets.all(16),
                  child: Text('No debug probes found'),
                ),
              )
            else
              ...List.generate(_probes.length, (index) {
                final probe = _probes[index];
                return Card(
                  color: _selectedProbeIndex == index
                      ? Theme.of(context).colorScheme.primaryContainer
                      : null,
                  child: ListTile(
                    title: Text(probe.probeType),
                    subtitle: Text(probe.identifier),
                    selected: _selectedProbeIndex == index,
                    onTap: () => setState(() => _selectedProbeIndex = index),
                  ),
                );
              }),
            const SizedBox(height: 16),

            // Chip selection
            TextFormField(
              decoration: const InputDecoration(
                labelText: 'Target Chip',
                border: OutlineInputBorder(),
              ),
              initialValue: _chip,
              onChanged: (value) => setState(() => _chip = value),
            ),
          ],

          const SizedBox(height: 24),

          // Connect button
          FilledButton.icon(
            onPressed:
                (_useRtt ? _selectedProbeIndex >= 0 : _selectedPortIndex >= 0)
                ? _connect
                : null,
            icon: const Icon(Icons.cable),
            label: const Text('Connect'),
          ),
        ],
      ),
    );
  }
}

// ============================================================================
// Telemetry View
// ============================================================================

class TelemetryView extends StatefulWidget {
  const TelemetryView({super.key});

  @override
  State<TelemetryView> createState() => _TelemetryViewState();
}

class _TelemetryViewState extends State<TelemetryView> {
  AdcSample? _latestSample;
  double _iqTarget = 1.0;
  bool _motorRunning = false;
  StreamSubscription? _adcSub;

  @override
  void initState() {
    super.initState();
    _adcSub = AdcSample.rustSignalStream.listen((signalPack) {
      setState(() {
        _latestSample = signalPack.message;
      });
    });
  }

  @override
  void dispose() {
    _adcSub?.cancel();
    super.dispose();
  }

  void _startMotor() {
    MotorCommand(
      command: MotorCommandTypeStart(iqTarget: _iqTarget),
    ).sendSignalToRust();
    setState(() => _motorRunning = true);
  }

  void _stopMotor() {
    MotorCommand(command: const MotorCommandTypeStop()).sendSignalToRust();
    setState(() => _motorRunning = false);
  }

  @override
  Widget build(BuildContext context) {
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          // Motor control card
          Card(
            child: Padding(
              padding: const EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    'Motor Control',
                    style: Theme.of(context).textTheme.titleLarge,
                  ),
                  const SizedBox(height: 16),
                  Row(
                    children: [
                      Expanded(
                        child: Slider(
                          value: _iqTarget,
                          min: 0,
                          max: 10,
                          divisions: 100,
                          label: '${_iqTarget.toStringAsFixed(1)} A',
                          onChanged: (value) =>
                              setState(() => _iqTarget = value),
                        ),
                      ),
                      SizedBox(
                        width: 60,
                        child: Text('${_iqTarget.toStringAsFixed(1)} A'),
                      ),
                    ],
                  ),
                  const SizedBox(height: 8),
                  Row(
                    mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                    children: [
                      FilledButton.icon(
                        onPressed: _motorRunning ? null : _startMotor,
                        icon: const Icon(Icons.play_arrow),
                        label: const Text('Start'),
                      ),
                      FilledButton.tonalIcon(
                        onPressed: _motorRunning ? _stopMotor : null,
                        icon: const Icon(Icons.stop),
                        label: const Text('Stop'),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ),
          const SizedBox(height: 16),

          // Telemetry card
          Card(
            child: Padding(
              padding: const EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    'Telemetry',
                    style: Theme.of(context).textTheme.titleLarge,
                  ),
                  const SizedBox(height: 16),
                  if (_latestSample == null)
                    const Text('Waiting for data...')
                  else ...[
                    _buildTelemetryRow('Phase A', '${_latestSample!.ia}'),
                    _buildTelemetryRow('Phase B', '${_latestSample!.ib}'),
                    _buildTelemetryRow('Phase C', '${_latestSample!.ic}'),
                    const Divider(),
                    _buildTelemetryRow(
                      'Bus Voltage',
                      '${(_latestSample!.vbusMv / 1000).toStringAsFixed(2)} V',
                    ),
                    _buildTelemetryRow(
                      'FET Temp',
                      '${(_latestSample!.fetTempCX10 / 10).toStringAsFixed(1)} °C',
                    ),
                    const Divider(),
                    _buildTelemetryRow('Sequence', '${_latestSample!.seq}'),
                  ],
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildTelemetryRow(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label),
          Text(value, style: const TextStyle(fontFamily: 'monospace')),
        ],
      ),
    );
  }
}

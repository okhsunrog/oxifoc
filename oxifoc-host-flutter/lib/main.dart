import 'dart:async';
import 'package:flutter/material.dart';
import 'package:rinf/rinf.dart';
import 'package:fl_chart/fl_chart.dart';
import 'package:xterm/xterm.dart';
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
      body: _isConnected ? const MainView() : const ConnectionView(),
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
// Main View (Chart + Terminal + Controls)
// ============================================================================

class MainView extends StatelessWidget {
  const MainView({super.key});

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (context, constraints) {
        // Use column layout on narrow screens, row on wide
        if (constraints.maxWidth < 800) {
          return const SingleChildScrollView(
            child: Column(
              children: [
                Padding(padding: EdgeInsets.all(8), child: MotorControlCard()),
                Padding(
                  padding: EdgeInsets.all(8),
                  child: SizedBox(height: 300, child: PhaseCurrentChart()),
                ),
                Padding(padding: EdgeInsets.all(8), child: TelemetryCard()),
                Padding(
                  padding: EdgeInsets.all(8),
                  child: SizedBox(height: 250, child: TerminalCard()),
                ),
              ],
            ),
          );
        } else {
          return const Row(
            children: [
              Expanded(
                flex: 2,
                child: Column(
                  children: [
                    Expanded(flex: 2, child: PhaseCurrentChart()),
                    Expanded(child: TerminalCard()),
                  ],
                ),
              ),
              SizedBox(
                width: 320,
                child: SingleChildScrollView(
                  child: Column(
                    children: [
                      Padding(
                        padding: EdgeInsets.all(8),
                        child: MotorControlCard(),
                      ),
                      Padding(
                        padding: EdgeInsets.all(8),
                        child: TelemetryCard(),
                      ),
                    ],
                  ),
                ),
              ),
            ],
          );
        }
      },
    );
  }
}

// ============================================================================
// Phase Current Chart
// ============================================================================

class PhaseCurrentChart extends StatefulWidget {
  const PhaseCurrentChart({super.key});

  @override
  State<PhaseCurrentChart> createState() => _PhaseCurrentChartState();
}

class _PhaseCurrentChartState extends State<PhaseCurrentChart> {
  final List<_ChartSample> _samples = [];
  StreamSubscription? _adcSub;
  static const int _maxSamples = 1200; // 20 seconds at 60Hz
  static const double _windowSeconds = 5.0;

  // ADC normalization
  static const int _adcMidpoint = 2048;
  static const double _adcScale = 2048.0;

  @override
  void initState() {
    super.initState();
    _adcSub = AdcSample.rustSignalStream.listen((signalPack) {
      final sample = signalPack.message;
      final now = DateTime.now().millisecondsSinceEpoch.toDouble();

      setState(() {
        _samples.add(
          _ChartSample(
            timestamp: now,
            ia: (sample.ia - _adcMidpoint) / _adcScale,
            ib: (sample.ib - _adcMidpoint) / _adcScale,
            ic: (sample.ic - _adcMidpoint) / _adcScale,
          ),
        );

        // Trim old samples
        while (_samples.length > _maxSamples) {
          _samples.removeAt(0);
        }
      });
    });
  }

  @override
  void dispose() {
    _adcSub?.cancel();
    super.dispose();
  }

  List<FlSpot> _buildSpots(double Function(_ChartSample) getValue) {
    if (_samples.isEmpty) return [];

    final now = _samples.last.timestamp;
    final cutoff = now - (_windowSeconds * 1000);

    return _samples
        .where((s) => s.timestamp >= cutoff)
        .map((s) => FlSpot((s.timestamp - now) / 1000, getValue(s)))
        .toList();
  }

  @override
  Widget build(BuildContext context) {
    final colorScheme = Theme.of(context).colorScheme;

    return Card(
      margin: const EdgeInsets.all(8),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Phase Currents',
              style: Theme.of(context).textTheme.titleMedium,
            ),
            const SizedBox(height: 8),
            Expanded(
              child: LineChart(
                LineChartData(
                  minY: -1.6,
                  maxY: 1.6,
                  minX: -_windowSeconds,
                  maxX: 0,
                  clipData: const FlClipData.all(),
                  gridData: FlGridData(
                    show: true,
                    horizontalInterval: 0.4,
                    verticalInterval: 1,
                    getDrawingHorizontalLine: (value) => FlLine(
                      color: colorScheme.outline.withValues(alpha: 0.3),
                      strokeWidth: 1,
                    ),
                    getDrawingVerticalLine: (value) => FlLine(
                      color: colorScheme.outline.withValues(alpha: 0.3),
                      strokeWidth: 1,
                    ),
                  ),
                  titlesData: FlTitlesData(
                    leftTitles: AxisTitles(
                      sideTitles: SideTitles(
                        showTitles: true,
                        reservedSize: 40,
                        getTitlesWidget: (value, meta) => Text(
                          value.toStringAsFixed(1),
                          style: TextStyle(
                            fontSize: 10,
                            color: colorScheme.onSurface,
                          ),
                        ),
                      ),
                    ),
                    bottomTitles: AxisTitles(
                      sideTitles: SideTitles(
                        showTitles: true,
                        reservedSize: 22,
                        getTitlesWidget: (value, meta) => Text(
                          '${value.toInt()}s',
                          style: TextStyle(
                            fontSize: 10,
                            color: colorScheme.onSurface,
                          ),
                        ),
                      ),
                    ),
                    topTitles: const AxisTitles(
                      sideTitles: SideTitles(showTitles: false),
                    ),
                    rightTitles: const AxisTitles(
                      sideTitles: SideTitles(showTitles: false),
                    ),
                  ),
                  borderData: FlBorderData(
                    show: true,
                    border: Border.all(
                      color: colorScheme.outline.withValues(alpha: 0.5),
                    ),
                  ),
                  lineTouchData: const LineTouchData(enabled: false),
                  lineBarsData: [
                    _buildLineData(
                      _buildSpots((s) => s.ia),
                      Colors.cyan,
                      'Phase A',
                    ),
                    _buildLineData(
                      _buildSpots((s) => s.ib),
                      Colors.purple,
                      'Phase B',
                    ),
                    _buildLineData(
                      _buildSpots((s) => s.ic),
                      Colors.orange,
                      'Phase C',
                    ),
                  ],
                ),
                duration: Duration.zero, // Disable animation for streaming
              ),
            ),
            const SizedBox(height: 8),
            // Legend
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                _legendItem('Phase A', Colors.cyan),
                const SizedBox(width: 16),
                _legendItem('Phase B', Colors.purple),
                const SizedBox(width: 16),
                _legendItem('Phase C', Colors.orange),
              ],
            ),
          ],
        ),
      ),
    );
  }

  LineChartBarData _buildLineData(List<FlSpot> spots, Color color, String id) {
    return LineChartBarData(
      spots: spots,
      isCurved: false,
      color: color,
      barWidth: 2,
      isStrokeCapRound: true,
      dotData: const FlDotData(show: false),
      belowBarData: BarAreaData(show: false),
    );
  }

  Widget _legendItem(String label, Color color) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Container(
          width: 12,
          height: 12,
          decoration: BoxDecoration(color: color, shape: BoxShape.circle),
        ),
        const SizedBox(width: 4),
        Text(label, style: const TextStyle(fontSize: 12)),
      ],
    );
  }
}

class _ChartSample {
  final double timestamp;
  final double ia, ib, ic;

  _ChartSample({
    required this.timestamp,
    required this.ia,
    required this.ib,
    required this.ic,
  });
}

// ============================================================================
// Motor Control Card
// ============================================================================

class MotorControlCard extends StatefulWidget {
  const MotorControlCard({super.key});

  @override
  State<MotorControlCard> createState() => _MotorControlCardState();
}

class _MotorControlCardState extends State<MotorControlCard> {
  double _iqTarget = 1.0;
  bool _motorRunning = false;

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
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Motor Control',
              style: Theme.of(context).textTheme.titleMedium,
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
                    onChanged: (value) => setState(() => _iqTarget = value),
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
    );
  }
}

// ============================================================================
// Telemetry Card
// ============================================================================

class TelemetryCard extends StatefulWidget {
  const TelemetryCard({super.key});

  @override
  State<TelemetryCard> createState() => _TelemetryCardState();
}

class _TelemetryCardState extends State<TelemetryCard> {
  AdcSample? _latestSample;
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

  @override
  Widget build(BuildContext context) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Telemetry', style: Theme.of(context).textTheme.titleMedium),
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

// ============================================================================
// Terminal Card
// ============================================================================

class TerminalCard extends StatefulWidget {
  const TerminalCard({super.key});

  @override
  State<TerminalCard> createState() => _TerminalCardState();
}

class _TerminalCardState extends State<TerminalCard> {
  late final Terminal _terminal;
  StreamSubscription? _logSub;

  @override
  void initState() {
    super.initState();
    _terminal = Terminal(maxLines: 1000);

    // Write welcome message
    _terminal.write('Oxifoc Terminal\r\n');
    _terminal.write('---------------\r\n');

    // Listen for log output from Rust
    _logSub = LogOutput.rustSignalStream.listen((signalPack) {
      _terminal.write('${signalPack.message.text}\r\n');
    });
  }

  @override
  void dispose() {
    _logSub?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Card(
      margin: const EdgeInsets.all(8),
      clipBehavior: Clip.antiAlias,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Padding(
            padding: const EdgeInsets.all(12),
            child: Text(
              'Console',
              style: Theme.of(context).textTheme.titleMedium,
            ),
          ),
          Expanded(
            child: TerminalView(
              _terminal,
              theme: TerminalTheme(
                cursor: Colors.white,
                selection: Colors.white24,
                foreground: Colors.white,
                background: Colors.black,
                black: Colors.black,
                white: Colors.white,
                red: Colors.red,
                green: Colors.green,
                yellow: Colors.yellow,
                blue: Colors.blue,
                magenta: Colors.purple,
                cyan: Colors.cyan,
                brightBlack: Colors.grey,
                brightWhite: Colors.white,
                brightRed: Colors.redAccent,
                brightGreen: Colors.greenAccent,
                brightYellow: Colors.yellowAccent,
                brightBlue: Colors.blueAccent,
                brightMagenta: Colors.purpleAccent,
                brightCyan: Colors.cyanAccent,
                searchHitBackground: Colors.yellow,
                searchHitBackgroundCurrent: Colors.orange,
                searchHitForeground: Colors.black,
              ),
            ),
          ),
        ],
      ),
    );
  }
}

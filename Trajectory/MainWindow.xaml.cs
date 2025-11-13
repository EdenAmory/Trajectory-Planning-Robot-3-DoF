using System;
using System.Linq;
using System.IO.Ports;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using System.Globalization;
using System.Timers;
using System.Diagnostics;
using System.Collections.Generic;
using System.Text.Json;
using System.IO;
using Microsoft.Win32;

namespace Trajectory
{
    // DTO used for file serialization (public so System.Text.Json can instantiate)
    public record ManualPointDto(double Qx, double Qy, double Ori);

    public partial class MainWindow : Window
    {
        // runtime state
        private System.Timers.Timer _timer;
        private Stopwatch _stopwatch = new Stopwatch();
        private readonly object _timerLock = new object();
        int _idx;               // current index in animation
        int _maxIndex;          // last index
        int jenis = 0;          // 0 = Time/Path, 1 = Time/Track (matches original)
        int space = 0;          // 0 = Joint space; 1 = Work space

        // run parameters used by timer logic
        private int _runTotalMs = 1000;
        private int _runIntervalMs = 50;
        private int _runN = 1;

        // trajectory/storage arrays (allocated when Calculate pressed)
        double[]? teta1, teta2, teta3;
        double[]? qx, qy, orientasi;

        // parsed link lengths (mm) — defaults updated earlier: a1=120, a2=128, a3=85
        double _a1Val = 120.0, _a2Val = 128.0, _a3Val = 85.0;

        // --- SSC-32 serial comm ---
        private SerialPort _serialPort = null!;
        private bool _isConnected = false;

        // servo channels (physical)
        private int ch0 = 0, ch1 = 1, ch2 = 2, ch3 = 3;

        // per-channel pulse limits (adjust as hardware requires)
        private double pwmMinCh0 = 2380, pwmMaxCh0 = 570;   // channel 0
        private double pwmMinCh1 = 2410, pwmMaxCh1 = 510;   // channel 1 (we will keep this fixed)
        private double pwmMinCh2 = 2450, pwmMaxCh2 = 610;   // channel 2
        private double pwmMinCh3 = 2360, pwmMaxCh3 = 560;   // channel 3

        // workspace indicator ellipse (kept behind other drawings)
        private Ellipse? m_workspaceEllipse;

        // Each grid square represents this many millimeters (10 mm)
        private const double GridSquareMm = 10.0;

        // in addition to existing arrays:
        private List<ManualPoint> _manualPoints = new List<ManualPoint>();

        // small model for manual points
        private class ManualPoint
        {
            public double Qx { get; set; }
            public double Qy { get; set; }
            public double Ori { get; set; }

            public override string ToString()
            {
                // compact display for the list
                return $"{Qx:0.0}, {Qy:0.0}, {Ori:0.0}";
            }
        }

        public MainWindow()
        {
            InitializeComponent();

            // populate COM list
            try
            {
                cmbPorts.ItemsSource = SerialPort.GetPortNames();
                if (cmbPorts.Items.Count > 0) cmbPorts.SelectedIndex = 0;
            }
            catch { /* ignore */ }

            // wire events
            m_calcJoint.Click += M_calcJoint_Click;
            m_runJoint.Click += M_runJoint_Click;
            m_calcWork.Click += M_calcWork_Click;
            m_runWork.Click += M_runWork_Click;
            m_clearPath.Click += M_clearPath_Click;
            m_exit.Click += (_, __) => Close();

            m_rbTimePath.Checked += (_, __) => { jenis = 0; };
            m_rbTimeTrack.Checked += (_, __) => { jenis = 1; };

            // COM controls
            btnConnect.Click += BtnConnect_Click;
            btnDisconnect.Click += BtnDisconnect_Click;
            btn_originPose.Click += BtnOriginPose_Click;
            btnDisconnect.IsEnabled = false;

            // List selection handlers: show pose on canvas when a trajectory item is selected
            m_joint_list.SelectionChanged += M_joint_list_SelectionChanged;
            m_work_list.SelectionChanged += M_work_list_SelectionChanged;

            // Manual points UI events
            btnAddPoint.Click += BtnAddPoint_Click;
            btnRemovePoint.Click += BtnRemovePoint_Click;
            btnCalcManual.Click += BtnCalcManual_Click;
            btnRunManual.Click += BtnRunManual_Click;
            btnSavePoints.Click += BtnSavePoints_Click;
            btnLoadPoints.Click += BtnLoadPoints_Click;
            lbManualPoints.SelectionChanged += LbManualPoints_SelectionChanged;
            btnUpdatePoint.Click += BtnUpdatePoint_Click;

            // init timer (use System.Timers.Timer + Stopwatch for higher precision)
            _timer = new System.Timers.Timer(10); // poll every 10 ms
            _timer.AutoReset = true;
            _timer.Elapsed += Timer_Elapsed;

            // init serial port object (not opened)
            InitializeSerialPort();

            // create workspace ellipse (indicator)
            CreateWorkspaceEllipse();

            // update workspace circle when chart resizes or link lengths change
            m_chart.SizeChanged += (_, __) => UpdateWorkspaceCircle();

            // update when link-length textboxes change
            m_a1.TextChanged += (_, __) => UpdateWorkspaceCircle();
            m_a2.TextChanged += (_, __) => UpdateWorkspaceCircle();
            m_a3.TextChanged += (_, __) => UpdateWorkspaceCircle();

            // initial draw/update (may be no-op if sizes not ready)
            UpdateWorkspaceCircle();
        }

        // Add a manual point (now adds row in list)
        private void BtnAddPoint_Click(object? sender, RoutedEventArgs e)
        {
            _manualPoints.Add(new ManualPoint { Qx = 0.0, Qy = 0.0, Ori = 90.0 });
            RefreshManualList();
            lbManualPoints.SelectedIndex = _manualPoints.Count - 1;
        }

        // Remove selected or last manual point
        private void BtnRemovePoint_Click(object? sender, RoutedEventArgs e)
        {
            int sel = lbManualPoints.SelectedIndex;
            if (sel >= 0 && sel < _manualPoints.Count)
                _manualPoints.RemoveAt(sel);
            else if (_manualPoints.Count > 0)
                _manualPoints.RemoveAt(_manualPoints.Count - 1);

            RefreshManualList();

            // clear editor if no selection
            if (_manualPoints.Count == 0)
            {
                txtManualQx.Text = txtManualQy.Text = txtManualOri.Text = string.Empty;
            }
            else
            {
                lbManualPoints.SelectedIndex = Math.Min(Math.Max(0, sel - 1), _manualPoints.Count - 1);
            }
        }

        private void RefreshManualList()
        {
            lbManualPoints.Items.Clear();
            for (int i = 0; i < _manualPoints.Count; i++)
            {
                lbManualPoints.Items.Add($"{i}  {_manualPoints[i].ToString()}");
            }
        }

        private void LbManualPoints_SelectionChanged(object? sender, SelectionChangedEventArgs e)
        {
            int idx = lbManualPoints.SelectedIndex;
            if (idx < 0 || idx >= _manualPoints.Count)
            {
                txtManualQx.Text = txtManualQy.Text = txtManualOri.Text = string.Empty;
                return;
            }

            var p = _manualPoints[idx];
            txtManualQx.Text = p.Qx.ToString("0.0", CultureInfo.InvariantCulture);
            txtManualQy.Text = p.Qy.ToString("0.0", CultureInfo.InvariantCulture);
            txtManualOri.Text = p.Ori.ToString("0.0", CultureInfo.InvariantCulture);
        }

        // Update selected point from editor fields
        private void BtnUpdatePoint_Click(object? sender, RoutedEventArgs e)
        {
            int idx = lbManualPoints.SelectedIndex;
            if (idx < 0 || idx >= _manualPoints.Count) return;

            if (!double.TryParse(txtManualQx.Text, NumberStyles.Number | NumberStyles.AllowLeadingSign | NumberStyles.AllowDecimalPoint, CultureInfo.InvariantCulture, out double qxVal))
            {
                MessageBox.Show("Qx tidak valid", "Input error", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }
            if (!double.TryParse(txtManualQy.Text, NumberStyles.Number | NumberStyles.AllowLeadingSign | NumberStyles.AllowDecimalPoint, CultureInfo.InvariantCulture, out double qyVal))
            {
                MessageBox.Show("Qy tidak valid", "Input error", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }
            if (!double.TryParse(txtManualOri.Text, NumberStyles.Number | NumberStyles.AllowLeadingSign | NumberStyles.AllowDecimalPoint, CultureInfo.InvariantCulture, out double oriVal))
            {
                MessageBox.Show("Orientasi tidak valid", "Input error", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            _manualPoints[idx].Qx = qxVal;
            _manualPoints[idx].Qy = qyVal;
            _manualPoints[idx].Ori = oriVal;

            RefreshManualList();
            lbManualPoints.SelectedIndex = idx;
        }

        // Save manual points to JSON file
        private void BtnSavePoints_Click(object? sender, RoutedEventArgs e)
        {
            if (_manualPoints.Count == 0)
            {
                MessageBox.Show("Tidak ada titik untuk disimpan.", "Info", MessageBoxButton.OK, MessageBoxImage.Information);
                return;
            }

            var dlg = new SaveFileDialog
            {
                Title = "Save manual points",
                Filter = "JSON files (*.json)|*.json|All files (*.*)|*.*",
                DefaultExt = ".json",
                FileName = "manual_points.json"
            };

            if (dlg.ShowDialog() != true) return;

            try
            {
                var dtos = _manualPoints.Select(p => new ManualPointDto(p.Qx, p.Qy, p.Ori)).ToList();
                var options = new JsonSerializerOptions { WriteIndented = true };
                string json = JsonSerializer.Serialize(dtos, options);
                File.WriteAllText(dlg.FileName, json, Encoding.UTF8);
            }
            catch (Exception ex)
            {
                MessageBox.Show("Gagal menyimpan file: " + ex.Message, "Error", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            MessageBox.Show("Manual points saved.", "Saved", MessageBoxButton.OK, MessageBoxImage.Information);
        }

        // Load manual points from JSON file (replaces current list)
        private void BtnLoadPoints_Click(object? sender, RoutedEventArgs e)
        {
            var dlg = new OpenFileDialog
            {
                Title = "Load manual points",
                Filter = "JSON files (*.json)|*.json|All files (*.*)|*.*",
                DefaultExt = ".json"
            };

            if (dlg.ShowDialog() != true) return;

            try
            {
                string json = File.ReadAllText(dlg.FileName, Encoding.UTF8);
                var dtos = JsonSerializer.Deserialize<List<ManualPointDto>>(json);
                if (dtos == null || dtos.Count == 0)
                {
                    MessageBox.Show("File kosong atau format tidak dikenali.", "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }

                _manualPoints = dtos.Select(d => new ManualPoint { Qx = d.Qx, Qy = d.Qy, Ori = d.Ori }).ToList();
                RefreshManualList();
                lbManualPoints.SelectedIndex = Math.Min(0, _manualPoints.Count - 1);
                if (_manualPoints.Count > 0) lbManualPoints.SelectedIndex = 0;
            }
            catch (Exception ex)
            {
                MessageBox.Show("Gagal memuat file: " + ex.Message, "Error", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            MessageBox.Show("Manual points loaded.", "Loaded", MessageBoxButton.OK, MessageBoxImage.Information);
        }

        // Read manual points (used by calculate/run). returns arrays built from _manualPoints
        private bool ReadManualPoints(out double[] outQx, out double[] outQy, out double[] outOri, out string reason)
        {
            reason = string.Empty;
            if (_manualPoints.Count == 0)
            {
                outQx = Array.Empty<double>();
                outQy = Array.Empty<double>();
                outOri = Array.Empty<double>();
                reason = "Tidak ada titik manual. Tambahkan minimal satu titik.";
                return false;
            }

            outQx = _manualPoints.Select(p => p.Qx).ToArray();
            outQy = _manualPoints.Select(p => p.Qy).ToArray();
            outOri = _manualPoints.Select(p => p.Ori).ToArray();
            return true;
        }

        // Calculate manual points: validate and draw path (uses ReadManualPoints above)
        private void BtnCalcManual_Click(object? sender, RoutedEventArgs e)
        {
            if (!ReadManualPoints(out double[] mx, out double[] my, out double[] mor, out string r))
            {
                MessageBox.Show(r, "Input error", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            ReadLinkLengths();

            for (int i = 0; i < mx.Length; i++)
            {
                if (!IsWorkspacePoseReachable(mx[i], my[i], mor[i], out string reason))
                {
                    MessageBox.Show($"Target ({mx[i]:0.0}, {my[i]:0.0}) with orientation {mor[i]:0.0}° is outside robot reach:\n{reason}", "Target out of reach", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }

                var (deg1, deg2, deg3) = ComputeInverseKinematicAngles(mx[i], my[i], mor[i]);
                if (!ValidateJointAngles(deg1, deg2, deg3, out string jreason))
                {
                    MessageBox.Show($"Target ({mx[i]:0.0}, {my[i]:0.0}) with orientation {mor[i]:0.0}° produces invalid joint angles:\n{jreason}", "Joint limit", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }
            }

            qx = mx;
            qy = my;
            orientasi = mor;

            m_work_list.Items.Clear();
            for (int i = 0; i < qx.Length; i++)
            {
                m_work_list.Items.Add($"{i,3}  {qx[i],6:0.0} {qy[i],6:0.0} {orientasi[i],6:0.0}");
            }

            // draw path
            DrawPathFromWorkspace(qx, qy);
        }

        // Run manual points: same as existing BtnRunManual_Click but using qx,qy,orientasi set by Calculate
        private void BtnRunManual_Click(object? sender, RoutedEventArgs e)
        {
            if (qx == null || qx.Length == 0)
            {
                MessageBox.Show("Tidak ada titik yang dihitung. Tekan Calculate terlebih dahulu.", "Info", MessageBoxButton.OK, MessageBoxImage.Information);
                return;
            }

            if (!int.TryParse(m_t.Text, out int totalMs) || totalMs <= 0) totalMs = 1000;
            int N = qx.Length - 1;
            int intervalMs = (jenis == 0) ? totalMs : Math.Max(1, totalMs / N);

            // validate all planned points before starting run
            ReadLinkLengths();
            for (int i = 0; i <= N; i++)
            {
                if (!IsWorkspacePoseReachable(qx[i], qy[i], orientasi![i], out string reason))
                {
                    MessageBox.Show($"Planned target ({qx[i]:0.0}, {qy[i]:0.0}) with orientation {orientasi[i]:0.0}° is outside robot reach:\n{reason}", "Target out of reach", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }

                var (deg1, deg2, deg3) = ComputeInverseKinematicAngles(qx[i], qy[i], orientasi[i]);
                if (!ValidateJointAngles(deg1, deg2, deg3, out string jreason))
                {
                    MessageBox.Show($"Planned target ({qx[i]:0.0}, {qy[i]:0.0}) with orientation {orientasi[i]:0.0}° produces invalid joint angles:\n{jreason}", "Joint limit", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }
            }

            space = 1;
            _idx = 0;
            _maxIndex = N;

            // store run params for timer logic
            _runTotalMs = totalMs;
            _runIntervalMs = intervalMs;
            _runN = N;

            // restart stopwatch and start timer (background)
            lock (_timerLock)
            {
                _stopwatch.Restart();
                _timer.Interval = 10; // poll resolution (ms)
                _timer.Start();
            }
        }

        // ensure link-length fields reflect UI (with safe defaults)
        private void ReadLinkLengths()
        {
            // If we're on a non-UI thread, marshal the read to the UI thread.
            if (!Dispatcher.CheckAccess())
            {
                Dispatcher.Invoke(() =>
                {
                    if (!double.TryParse(m_a1.Text, out _a1Val) || _a1Val <= 0) _a1Val = 120.0;
                    if (!double.TryParse(m_a2.Text, out _a2Val) || _a2Val <= 0) _a2Val = 128.0;
                    if (!double.TryParse(m_a3.Text, out _a3Val) || _a3Val <= 0) _a3Val = 85.0;
                });
                return;
            }

            if (!double.TryParse(m_a1.Text, out _a1Val) || _a1Val <= 0) _a1Val = 120.0;
            if (!double.TryParse(m_a2.Text, out _a2Val) || _a2Val <= 0) _a2Val = 128.0;
            if (!double.TryParse(m_a3.Text, out _a3Val) || _a3Val <= 0) _a3Val = 85.0;
        }

        // ---------------------
        // Serial helpers (SSC-32)
        // ---------------------
        private void InitializeSerialPort()
        {
            _serialPort = new SerialPort
            {
                BaudRate = 115200,
                Parity = Parity.None,
                DataBits = 8,
                StopBits = StopBits.One,
                Handshake = Handshake.None,
                DtrEnable = true,
                RtsEnable = true,
                Encoding = Encoding.ASCII,
                ReadTimeout = 200,
                WriteTimeout = 500
            };
            _serialPort.DataReceived += (s, e) =>
            {
                try { var _ = _serialPort.ReadExisting(); } catch { /* ignore */ }
            };
        }

        public bool Connect(string portName)
        {
            try
            {
                if (_serialPort.IsOpen) _serialPort.Close();
                _serialPort.PortName = portName;
                _serialPort.Open();
                _isConnected = true;
                return true;
            }
            catch
            {
                _isConnected = false;
                return false;
            }
        }

        public void Disconnect()
        {
            try
            {
                if (_serialPort.IsOpen) _serialPort.Close();
            }
            catch { }
            _isConnected = false;
        }

        // helper to send a raw SSC-32 command string (ends with CRLF)
        private void SendRawCommand(string cmd)
        {
            if (!_isConnected) return;
            try
            {
                // ensure newline termination so commands appear on separate lines in serial monitor
                if (!cmd.EndsWith("\r") && !cmd.EndsWith("\n")) cmd += "\r\n";
                _serialPort.Write(cmd);
            }
            catch
            {
                try { _serialPort.Close(); } catch { }
                _isConnected = false;
            }
        }

        // map an angle (deg) to SSC-32 pulse width using linear interpolation.
        // angleMin/Max are the logical angle range for the servo.
        private int MapAngleToPulse(double angleDeg, double angleMin, double angleMax, double pwmMin, double pwmMax)
        {
            // clamp angle to range
            if (angleDeg < Math.Min(angleMin, angleMax)) angleDeg = Math.Min(angleMin, angleMax);
            if (angleDeg > Math.Max(angleMin, angleMax)) angleDeg = Math.Max(angleMin, angleMax);

            double t = (angleDeg - angleMin) / (angleMax - angleMin);
            double pw = pwmMin + t * (pwmMax - pwmMin);
            return (int)Math.Round(pw);
        }

        // Send only servos 0,2,3 (servo 1 must remain unchanged).
        // Each servo is sent as its own command line so they don't "stack" in the serial monitor.
        private void SendServos0_2_3(double deg0, double deg2, double deg3, int timeMs)
        {
            if (!_isConnected) return;

            // Map logical angles to pulses. Adjust logical ranges if your servos use different ranges.
            int p0 = MapAngleToPulse(deg0, 0.0, 180.0, pwmMinCh0, pwmMaxCh0);    // servo channel 0
            int p2 = MapAngleToPulse(deg2, -90.0, 90.0, pwmMinCh2, pwmMaxCh2);    // servo channel 2
            int p3 = MapAngleToPulse(deg3, -90.0, 90.0, pwmMinCh3, pwmMaxCh3);    // servo channel 3

            // send each as separate line (so monitor shows each on its own line)
            SendRawCommand($"#{ch0} P{p0} T{timeMs}");
            SendRawCommand($"#{ch2} P{p2} T{timeMs}");
            SendRawCommand($"#{ch3} P{p3} T{timeMs}");
        }

        // ---------------------
        // UI: COM button handlers
        // ---------------------
        private void BtnConnect_Click(object sender, RoutedEventArgs e)
        {
            if (_isConnected)
            {
                MessageBox.Show("Already connected.");
                return;
            }

            if (cmbPorts.SelectedItem == null)
            {
                MessageBox.Show("Pilih COM port dulu.");
                return;
            }

            var port = cmbPorts.SelectedItem.ToString();
            if (Connect(port!))
            {
                btnConnect.IsEnabled = false;
                btnDisconnect.IsEnabled = true;

                // Send origin pose on connect. Send each servo command on its own line (T1000).
                // Keep servo 1 set as well to the origin value (so robot is flat).
                SendRawCommand($"#{ch0} P1480 T1000");
                SendRawCommand($"#{ch1} P740 T1000");   // servo 1 fixed
                SendRawCommand($"#{ch2} P1530 T1000");
                SendRawCommand($"#{ch3} P1450 T1000");
            }
            else
            {
                MessageBox.Show("Gagal membuka port " + port);
            }
        }

        private void BtnDisconnect_Click(object sender, RoutedEventArgs e)
        {
            if (!_isConnected)
            {
                MessageBox.Show("Port belum terbuka.");
                return;
            }

            Disconnect();
            btnConnect.IsEnabled = true;
            btnDisconnect.IsEnabled = false;
        }

        private void BtnOriginPose_Click(object sender, RoutedEventArgs e)
        {
            // origin pulse widths (each on its own line) so monitor displays them properly
            if (!_isConnected)
            {
                MessageBox.Show("Port belum terbuka. Silakan Connect terlebih dahulu.");
                return;
            }

            SendRawCommand($"#{ch0} P1480 T1000");
            SendRawCommand($"#{ch1} P740 T1000");   // servo 1 fixed
            SendRawCommand($"#{ch2} P1530 T1000");
            SendRawCommand($"#{ch3} P1450 T1000");
        }

        // ---------------------
        // UI handlers (trajectory, run, timer, drawing, IK)
        // ---------------------
        private void M_clearPath_Click(object sender, RoutedEventArgs e)
        {
            // remove only non-grid children so grid and workspace ellipse remain
            for (int i = m_pathCanvas.Children.Count - 1; i >= 0; i--)
            {
                if (m_pathCanvas.Children[i] is FrameworkElement fe && fe.Tag as string == "__grid") continue;
                // preserve ellipse (no tag) by checking reference
                if (m_workspaceEllipse != null && m_pathCanvas.Children[i] == m_workspaceEllipse) continue;
                m_pathCanvas.Children.RemoveAt(i);
            }

            m_armCanvas.Children.Clear();
            m_joint_list.Items.Clear();
            m_work_list.Items.Clear();
        }

        // Joint Space Calculate
        private void M_calcJoint_Click(object sender, RoutedEventArgs e)
        {
            // read parameters
            if (!int.TryParse(m_n.Text, out int m_nPoints) || m_nPoints <= 0) m_nPoints = 25;
            if (!double.TryParse(m_teta1_0.Text, out double t1_0)) t1_0 = 0;
            if (!double.TryParse(m_teta2_0.Text, out double t2_0)) t2_0 = 0;
            if (!double.TryParse(m_teta3_0.Text, out double t3_0)) t3_0 = 0;
            if (!double.TryParse(m_teta1_1.Text, out double t1_1)) t1_1 = t1_0;
            if (!double.TryParse(m_teta2_1.Text, out double t2_1)) t2_1 = t2_0;
            if (!double.TryParse(m_teta3_1.Text, out double t3_1)) t3_1 = t3_0;

            int N = m_nPoints; // keep name similar to pictures; we'll generate N+1 samples (inclusive)
            teta1 = new double[N + 1];
            teta2 = new double[N + 1];
            teta3 = new double[N + 1];

            m_joint_list.Items.Clear();

            for (int i = 0; i <= N; i++)
            {
                double f = (double)i / N;
                teta1[i] = t1_0 + (t1_1 - t1_0) * f;
                teta2[i] = t2_0 + (t2_1 - t2_0) * f;
                teta3[i] = t3_0 + (t3_1 - t3_0) * f;

                // validate joint-limits for joint-space generation
                if (!ValidateJointAngles(teta1[i], teta2![i], teta3![i], out string reason))
                {
                    MessageBox.Show($"Generated joint sample {i} invalid: {reason}", "Joint limit", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }

                m_joint_list.Items.Add($"{i,3}  {teta1[i],6:0.0} {teta2[i],6:0.0} {teta3[i],6:0.0}");
            }

            ReadLinkLengths();
            // draw end-effector path computed from these joint angles
            DrawPathFromJointAngles(teta1, teta2, teta3);
        }

        // Joint Space Run
        private void M_runJoint_Click(object sender, RoutedEventArgs e)
        {
            if (teta1 == null || teta1.Length == 0) return;

            // validate all joint samples before running
            for (int i = 0; i < teta1.Length; i++)
            {
                if (!ValidateJointAngles(teta1[i], teta2![i], teta3![i], out string reason))
                {
                    MessageBox.Show($"Joint sample {i} invalid: {reason}", "Joint limit", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }
            }

            if (!int.TryParse(m_t.Text, out int totalMs) || totalMs <= 0) totalMs = 1000;
            int N = teta1.Length - 1;

            int intervalMs = (jenis == 0) ? totalMs : Math.Max(1, totalMs / N);

            space = 0;
            _idx = 0;
            _maxIndex = N;

            // store run params for timer logic
            _runTotalMs = totalMs;
            _runIntervalMs = intervalMs;
            _runN = N;

            // restart stopwatch and start timer (background)
            lock (_timerLock)
            {
                _stopwatch.Restart();
                _timer.Interval = 10; // poll resolution (ms). Keep small for accurate scheduling.
                _timer.Start();
            }
        }

        // Work Space Calculate
        private void M_calcWork_Click(object sender, RoutedEventArgs e)
        {
            if (!int.TryParse(m_n.Text, out int m_nPoints) || m_nPoints <= 0) m_nPoints = 25;
            if (!double.TryParse(m_qx_0.Text, out double qx0)) qx0 = 0;
            if (!double.TryParse(m_qy_0.Text, out double qy0)) qy0 = 0;
            if (!double.TryParse(m_orientasi_0.Text, out double ori0)) ori0 = 0;
            if (!double.TryParse(m_qx_1.Text, out double qx1)) qx1 = qx0;
            if (!double.TryParse(m_qy_1.Text, out double qy1)) qy1 = qy0;
            if (!double.TryParse(m_orientasi_1.Text, out double ori1)) ori1 = ori0;

            int N = m_nPoints;
            qx = new double[N + 1];
            qy = new double[N + 1];
            orientasi = new double[N + 1];

            m_work_list.Items.Clear();

            for (int i = 0; i <= N; i++)
            {
                double f = (double)i / N;
                qx[i] = qx0 + (qx1 - qx0) * f;
                qy[i] = qy0 + (qy1 - qy0) * f;
                orientasi[i] = ori0 + (ori1 - ori0) * f;

                m_work_list.Items.Add($"{i,3}  {qx[i],6:0.0} {qy[i],6:0.0} {orientasi[i],6:0.0}");
            }

            // read link lengths and validate workspace reachability for generated targets
            ReadLinkLengths();

            // validate every sample's feasibility (wrist inside 2-link reach for a given orientation)
            for (int i = 0; i <= N; i++)
            {
                if (!IsWorkspacePoseReachable(qx[i], qy[i], orientasi[i], out string reason))
                {
                    MessageBox.Show($"Target ({qx[i]:0.0}, {qy[i]:0.0}) with orientation {orientasi[i]:0.0}° is outside robot reach:\n{reason}", "Target out of reach", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }

                // also check resulting joint angles respect servo limits
                var (deg1, deg2, deg3) = ComputeInverseKinematicAngles(qx[i], qy[i], orientasi[i]);
                if (!ValidateJointAngles(deg1, deg2, deg3, out string jreason))
                {
                    MessageBox.Show($"Target ({qx[i]:0.0}, {qy[i]:0.0}) with orientation {orientasi[i]:0.0}° produces invalid joint angles:\n{jreason}", "Joint limit", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }
            }

            // draw path from workspace points
            DrawPathFromWorkspace(qx, qy);
        }

        // Work Space Run
        private void M_runWork_Click(object sender, RoutedEventArgs e)
        {
            if (qx == null || qx.Length == 0) return;

            if (!int.TryParse(m_t.Text, out int totalMs) || totalMs <= 0) totalMs = 1000;
            int N = qx.Length - 1;
            int intervalMs = (jenis == 0) ? totalMs : Math.Max(1, totalMs / N);

            // validate all planned points before starting run
            ReadLinkLengths();
            for (int i = 0; i <= N; i++)
            {
                if (!IsWorkspacePoseReachable(qx[i], qy[i], orientasi![i], out string reason))
                {
                    MessageBox.Show($"Planned target ({qx[i]:0.0}, {qy[i]:0.0}) with orientation {orientasi[i]:0.0}° is outside robot reach:\n{reason}", "Target out of reach", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }

                var (deg1, deg2, deg3) = ComputeInverseKinematicAngles(qx[i], qy[i], orientasi[i]);
                if (!ValidateJointAngles(deg1, deg2, deg3, out string jreason))
                {
                    MessageBox.Show($"Planned target ({qx[i]:0.0}, {qy[i]:0.0}) with orientation {orientasi[i]:0.0}° produces invalid joint angles:\n{jreason}", "Joint limit", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }
            }

            space = 1;
            _idx = 0;
            _maxIndex = N;

            // store run params for timer logic
            _runTotalMs = totalMs;
            _runIntervalMs = intervalMs;
            _runN = N;

            // restart stopwatch and start timer (background)
            lock (_timerLock)
            {
                _stopwatch.Restart();
                _timer.Interval = 10; // poll resolution (ms)
                _timer.Start();
            }
        }

        // ---------------------
        // Timer (background) elapsed handler using Stopwatch for accurate indexing
        // ---------------------
        private void Timer_Elapsed(object? sender, ElapsedEventArgs e)
        {
            // compute expected sample index based on elapsed time
            int expectedIdx;
            long elapsedMs = _stopwatch.ElapsedMilliseconds;

            lock (_timerLock)
            {
                if (_runTotalMs <= 0 || _runN <= 0)
                {
                    expectedIdx = 0;
                }
                else
                {
                    if (jenis == 1)
                    {
                        // Time/Track: total time split across N segments -> map elapsed to index 0..N
                        double ratio = elapsedMs / (double)_runTotalMs;
                        expectedIdx = (int)Math.Floor(ratio * _runN);
                    }
                    else
                    {
                        // Time/Path: interval is treated as per-step (existing app semantics)
                        if (_runIntervalMs <= 0) expectedIdx = 0;
                        else expectedIdx = (int)Math.Floor(elapsedMs / (double)_runIntervalMs);
                    }
                }

                if (expectedIdx > _maxIndex) expectedIdx = _maxIndex + 1; // indicate done
                if (expectedIdx <= _idx)
                {
                    // nothing new to do
                    if (expectedIdx > _maxIndex)
                    {
                        // stop and cleanup
                        _stopwatch.Stop();
                        _timer.Stop();
                    }
                    return;
                }

                // advance to the expected index (latest). We'll show only the latest sample to UI
                int newIdx = expectedIdx;
                if (newIdx > _maxIndex) newIdx = _maxIndex;

                // capture local copies needed for sending
                int sendInterval = Math.Max(1, _runIntervalMs);
                int curSpace = space;
                int idxToShow = newIdx;

                // update _idx now
                _idx = idxToShow;

                // For UI drawing we must marshal to UI thread
                if (curSpace == 0)
                {
                    if (teta1 == null) return;
                    double a = teta1[idxToShow], b = teta2![idxToShow], c = teta3![idxToShow];

                    // UI update
                    Application.Current.Dispatcher.BeginInvoke(new Action(() =>
                    {
                        ArmDraw(a, b, c);
                    }));

                    // send to SSC-32 if connected (only servos 0,2,3)
                    if (_isConnected)
                    {
                        try
                        {
                            SendServos0_2_3(a, b, c, sendInterval);
                        }
                        catch
                        {
                            // ignore send errors here; connection state handled in SendRawCommand
                        }
                    }
                }
                else
                {
                    if (qx == null) return;
                    double qxLocal = qx[idxToShow], qyLocal = qy[idxToShow], oriLocal = orientasi![idxToShow];

                    // compute IK on background thread
                    var (deg1, deg2, deg3) = ComputeInverseKinematicAngles(qxLocal, qyLocal, oriLocal);

                    // validate then update UI and send
                    if (!ValidateJointAngles(deg1, deg2, deg3, out string jreason))
                    {
                        // stop timer and show message on UI thread
                        _stopwatch.Stop();
                        _timer.Stop();
                        Application.Current.Dispatcher.BeginInvoke(new Action(() =>
                        {
                            MessageBox.Show($"Planned target {idxToShow} produces invalid joint angles: {jreason}", "Joint limit", MessageBoxButton.OK, MessageBoxImage.Warning);
                        }));
                        return;
                    }

                    // draw on UI thread
                    Application.Current.Dispatcher.BeginInvoke(new Action(() =>
                    {
                        ArmDraw(deg1, deg2, deg3);
                    }));

                    if (_isConnected)
                    {
                        try
                        {
                            SendServos0_2_3(deg1, deg2, deg3, sendInterval);
                        }
                        catch
                        {
                            // ignore send errors here
                        }
                    }
                }

                // check for completion
                if (_idx >= _maxIndex)
                {
                    _stopwatch.Stop();
                    _timer.Stop();
                }
            }
        }

        // ---------------------
        // List selection handlers
        // ---------------------
        private void M_joint_list_SelectionChanged(object? sender, SelectionChangedEventArgs e)
        {
            // show the selected joint-space pose on the chart
            int idx = m_joint_list.SelectedIndex;
            if (idx < 0) return;
            if (teta1 == null || teta2 == null || teta3 == null) return;
            if (idx >= teta1.Length) return;

            // stop animation to show static pose
            if (_timer.Enabled) _timer.Stop();

            // draw selected pose
            double th1 = teta1[idx];
            double th2 = teta2[idx];
            double th3 = teta3[idx];
            ArmDraw(th1, th2, th3);

            // validate selected joint angles
            if (!ValidateJointAngles(th1, th2, th3, out string reason))
            {
                MessageBox.Show($"Selected joint pose invalid: {reason}", "Joint limit", MessageBoxButton.OK, MessageBoxImage.Warning);
                // still update indicators with values but do not send to robot
                var (qxVal, qyVal) = ComputeForwardKinematicsFromAngles(th1, th2, th3);
                UpdateIndicators(qxVal, qyVal, th1, th2, th3);
                return;
            }

            // compute Qx,Qy and update indicators
            var (qxVal2, qyVal2) = ComputeForwardKinematicsFromAngles(th1, th2, th3);
            UpdateIndicators(qxVal2, qyVal2, th1, th2, th3);

            // Send to robot (servo 0,2,3) so real robot mirrors selected pose
            if (_isConnected)
            {
                int timeMs = 1000;
                if (!int.TryParse(m_t.Text, out timeMs) || timeMs <= 0) timeMs = 1000;
                try
                {
                    SendServos0_2_3(th1, th2, th3, timeMs);
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Gagal mengirim posisi ke robot: " + ex.Message, "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                }
            }
        }

        private void M_work_list_SelectionChanged(object? sender, SelectionChangedEventArgs e)
        {
            // show the selected workspace pose on the chart (by computing IK and drawing)
            int idx = m_work_list.SelectedIndex;
            if (idx < 0) return;
            if (qx == null || qy == null || orientasi == null) return;
            if (idx >= qx.Length) return;

            // stop animation to show static pose
            if (_timer.Enabled) _timer.Stop();

            // compute IK angles for this workspace sample
            var (deg1, deg2, deg3) = ComputeInverseKinematicAngles(qx[idx], qy[idx], orientasi[idx]);

            // validate
            if (!ValidateJointAngles(deg1, deg2, deg3, out string reason))
            {
                MessageBox.Show($"Workspace pose {idx} produces invalid joint angles: {reason}", "Joint limit", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            // draw using computed angles
            ArmDraw(deg1, deg2, deg3);

            // compute forward kinematics (sanity) and update indicators with Qx/Qy from workspace arrays
            UpdateIndicators(qx[idx], qy[idx], deg1, deg2, deg3);

            // Send to robot (servo 0,2,3) so real robot mirrors selected workspace pose
            if (_isConnected)
            {
                int timeMs = 1000;
                if (!int.TryParse(m_t.Text, out timeMs) || timeMs <= 0) timeMs = 1000;
                try
                {
                    SendServos0_2_3(deg1, deg2, deg3, timeMs);
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Gagal mengirim posisi ke robot: " + ex.Message, "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                }
            }
        }

        // Add this helper method to compute forward kinematics (end-effector position) from joint angles.
        private (double qxVal, double qyVal) ComputeForwardKinematicsFromAngles(double deg1, double deg2, double deg3)
        {
            ReadLinkLengths();
            double t1 = deg1 * Math.PI / 180.0;
            double t2 = deg2 * Math.PI / 180.0;
            double t3 = deg3 * Math.PI / 180.0;

            double ex = _a1Val * Math.Cos(t1) + _a2Val * Math.Cos(t1 + t2) + _a3Val * Math.Cos(t1 + t2 + t3);
            double ey = _a1Val * Math.Sin(t1) + _a2Val * Math.Sin(t1 + t2) + _a3Val * Math.Sin(t1 + t2 + t3);
            return (ex, ey);
        }

        // Add this helper to update the indicator UI fields (Qx, Qy, teta1, teta2, teta3, orientasi)
        private void UpdateIndicators(double qxVal, double qyVal, double t1Deg, double t2Deg, double t3Deg)
        {
            // Format with one decimal place
            txtQx.Text = qxVal.ToString("0.0", CultureInfo.InvariantCulture);
            txtQy.Text = qyVal.ToString("0.0", CultureInfo.InvariantCulture);
            txtT1.Text = t1Deg.ToString("0.0", CultureInfo.InvariantCulture);
            txtT2.Text = t2Deg.ToString("0.0", CultureInfo.InvariantCulture);
            txtT3.Text = t3Deg.ToString("0.0", CultureInfo.InvariantCulture);

            // compute Orientasi (∅) = θ1 + θ2 + θ3 and normalize
            double orient = t1Deg + t2Deg + t3Deg;
            orient = NormalizeAngleDeg(orient);

            // Update orientasi textbox if it exists in the XAML.
            // Use FindName to avoid compile-time dependency if the control hasn't been added yet.
            var orientCtrl = this.FindName("txtOrientasi") as System.Windows.Controls.TextBox;
            if (orientCtrl != null)
            {
                orientCtrl.Text = orient.ToString("0.0", CultureInfo.InvariantCulture);
            }
        }

        // ---------------------
        // Drawing helpers (unchanged)...
        // ---------------------
        // Map logical robot coords (mm) to canvas pixels so full reach fits
        private (double x, double y) ToCanvas(double ux, double uy, double canvasWidth, double canvasHeight)
        {
            // ensure current link lengths
            ReadLinkLengths();
            double totalLength = Math.Max(1.0, _a1Val + _a2Val + _a3Val); // 333 mm expected with defaults
            double marginFactor = 0.9; // leave 10% margin
            double minDim = Math.Min(canvasWidth, canvasHeight);
            double scale = (minDim * marginFactor) / (2.0 * totalLength); // map diameter to minDim*margin -> pixels per mm
            double cx = canvasWidth / 2.0 + ux * scale;
            double cy = canvasHeight / 2.0 - uy * scale; // invert Y (robot +Y is up)
            return (cx, cy);
        }

        private void DrawPathFromJointAngles(double[] th1, double[] th2, double[] th3)
        {
            if (th1 == null) return;

            // keep grid & workspace ellipse; remove other children
            for (int i = m_pathCanvas.Children.Count - 1; i >= 0; i--)
            {
                if (m_pathCanvas.Children[i] is FrameworkElement fe && fe.Tag as string == "__grid") continue;
                // preserve ellipse (no tag) by checking reference
                if (m_workspaceEllipse != null && m_pathCanvas.Children[i] == m_workspaceEllipse) continue;
                m_pathCanvas.Children.RemoveAt(i);
            }

            ReadLinkLengths();

            double w = m_pathCanvas.ActualWidth;
            double h = m_pathCanvas.ActualHeight;
            if (w == 0 || h == 0) { w = m_chart.ActualWidth; h = m_chart.ActualHeight; }

            DrawGrid(w, h);
            UpdateWorkspaceCircle();

            var poly = new Polyline
            {
                Stroke = Brushes.Blue,
                StrokeThickness = 1.8,
                StrokeLineJoin = PenLineJoin.Round
            };

            for (int i = 0; i < th1.Length; i++)
            {
                double t1 = th1[i] * Math.PI / 180.0;
                double t2 = th2[i] * Math.PI / 180.0;
                double t3 = th3[i] * Math.PI / 180.0;
                double ex = _a1Val * Math.Cos(t1) + _a2Val * Math.Cos(t1 + t2) + _a3Val * Math.Cos(t1 + t2 + t3);
                double ey = _a1Val * Math.Sin(t1) + _a2Val * Math.Sin(t1 + t2) + _a3Val * Math.Sin(t1 + t2 + t3);
                var p = ToCanvas(ex, ey, w, h);
                poly.Points.Add(new Point(p.x, p.y));
            }

            m_pathCanvas.Children.Add(poly);
        }

        private void DrawPathFromWorkspace(double[] xs, double[] ys)
        {
            if (xs == null) return;

            // keep grid & workspace ellipse; remove other children
            for (int i = m_pathCanvas.Children.Count - 1; i >= 0; i--)
            {
                if (m_pathCanvas.Children[i] is FrameworkElement fe && fe.Tag as string == "__grid") continue;
                if (m_workspaceEllipse != null && m_pathCanvas.Children[i] == m_workspaceEllipse) continue;
                m_pathCanvas.Children.RemoveAt(i);
            }

            ReadLinkLengths();

            double w = m_pathCanvas.ActualWidth;
            double h = m_pathCanvas.ActualHeight;
            if (w == 0 || h == 0) { w = m_chart.ActualWidth; h = m_chart.ActualHeight; }

            DrawGrid(w, h);
            UpdateWorkspaceCircle();

            var poly = new Polyline
            {
                Stroke = Brushes.DarkGreen,
                StrokeThickness = 1.6,
                StrokeLineJoin = PenLineJoin.Round
            };

            for (int i = 0; i < xs.Length; i++)
            {
                var p = ToCanvas(xs[i], ys[i], w, h);
                poly.Points.Add(new Point(p.x, p.y));
            }

            m_pathCanvas.Children.Add(poly);
        }

        // Draw the 3-DoF planar arm on m_armCanvas using joint angles in degrees.
        private void ArmDraw(double sdt1, double sdt2, double sdt3)
        {
            m_armCanvas.Children.Clear();
            ReadLinkLengths();

            double w = m_armCanvas.ActualWidth;
            double h = m_armCanvas.ActualHeight;
            if (w == 0 || h == 0) { w = m_chart.ActualWidth; h = m_chart.ActualHeight; }

            // convert to radians
            double t1 = sdt1 * Math.PI / 180.0;
            double t2 = sdt2 * Math.PI / 180.0;
            double t3 = sdt3 * Math.PI / 180.0;

            // compute joint positions in robot units (origin at 0,0)
            double x0 = 0, y0 = 0;
            double x1 = x0 + _a1Val * Math.Cos(t1);
            double y1 = y0 + _a1Val * Math.Sin(t1);
            double x2 = x1 + _a2Val * Math.Cos(t1 + t2);
            double y2 = y1 + _a2Val * Math.Sin(t1 + t2);
            double x3 = x2 + _a3Val * Math.Cos(t1 + t2 + t3);
            double y3 = y2 + _a3Val * Math.Sin(t1 + t2 + t3);

            // map to canvas
            var p0 = ToCanvas(x0, y0, w, h);
            var p1 = ToCanvas(x1, y1, w, h);
            var p2 = ToCanvas(x2, y2, w, h);
            var p3 = ToCanvas(x3, y3, w, h);

            // draw links
            var linkPen = new SolidColorBrush(Color.FromRgb(180, 30, 30));
            var link1 = new Line { X1 = p0.x, Y1 = p0.y, X2 = p1.x, Y2 = p1.y, Stroke = linkPen, StrokeThickness = 6, StrokeStartLineCap = PenLineCap.Round, StrokeEndLineCap = PenLineCap.Round };
            var link2 = new Line { X1 = p1.x, Y1 = p1.y, X2 = p2.x, Y2 = p2.y, Stroke = Brushes.DarkOrange, StrokeThickness = 6, StrokeStartLineCap = PenLineCap.Round, StrokeEndLineCap = PenLineCap.Round };
            var link3 = new Line { X1 = p2.x, Y1 = p2.y, X2 = p3.x, Y2 = p3.y, Stroke = Brushes.DodgerBlue, StrokeThickness = 5, StrokeStartLineCap = PenLineCap.Round, StrokeEndLineCap = PenLineCap.Round };

            m_armCanvas.Children.Add(link1);
            m_armCanvas.Children.Add(link2);
            m_armCanvas.Children.Add(link3);

            // draw joints
            AddJointEllipse(p0.x, p0.y, 10, Brushes.Black);
            AddJointEllipse(p1.x, p1.y, 10, Brushes.LightGray);
            AddJointEllipse(p2.x, p2.y, 10, Brushes.LightGray);
            AddJointEllipse(p3.x, p3.y, 8, Brushes.Green);
        }

        private void AddJointEllipse(double cx, double cy, double r, Brush fill)
        {
            var el = new Ellipse
            {
                Width = r,
                Height = r,
                Fill = fill,
                Stroke = Brushes.Black,
                StrokeThickness = 1
            };
            Canvas.SetLeft(el, cx - r / 2.0);
            Canvas.SetTop(el, cy - r / 2.0);
            m_armCanvas.Children.Add(el);
        }

        // ---------------------
        // Inverse kinematics: compute joint angles for end-effector pose (qx,qy,orientasi deg)
        // Uses classic 3-link planar approach: compute wrist then 2-link IK.
        // ---------------------
        private void InverseKinematic1(double iqx, double iqy, double iorientasi)
        {
            ReadLinkLengths();
            double phi = iorientasi * Math.PI / 180.0;

            // wrist position (position of joint between link2 and link3)
            double wx = iqx - _a3Val * Math.Cos(phi);
            double wy = iqy - _a3Val * Math.Sin(phi);

            double D = (wx * wx + wy * wy - _a1Val * _a1Val - _a2Val * _a2Val) / (2.0 * _a1Val * _a2Val);
            D = Math.Max(-1.0, Math.Min(1.0, D));
            double theta2 = Math.Acos(D); // principal solution
            // choose elbow-down (+) or elbow-up (-). we'll use elbow-down (positive sin)
            double k1 = _a1Val + _a2Val * Math.Cos(theta2);
            double k2 = _a2Val * Math.Sin(theta2);
            double theta1 = Math.Atan2(wy, wx) - Math.Atan2(k2, k1);
            double theta3 = phi - theta1 - theta2;

            // convert radians to degrees and normalize
            double deg1 = theta1 * 180.0 / Math.PI;
            double deg2 = theta2 * 180.0 / Math.PI;
            double deg3 = theta3 * 180.0 / Math.PI;

            // normalize to -180..180
            deg1 = NormalizeAngleDeg(deg1);
            deg2 = NormalizeAngleDeg(deg2);
            deg3 = NormalizeAngleDeg(deg3);

            // draw
            ArmDraw(deg1, deg2, deg3);
        }

        // compute IK angles and return them (used when sending commands)
        private (double deg1, double deg2, double deg3) ComputeInverseKinematicAngles(double iqx, double iqy, double iorientasi)
        {
            ReadLinkLengths();
            double phi = iorientasi * Math.PI / 180.0;
            double wx = iqx - _a3Val * Math.Cos(phi);
            double wy = iqy - _a3Val * Math.Sin(phi);

            double D = (wx * wx + wy * wy - _a1Val * _a1Val - _a2Val * _a2Val) / (2.0 * _a1Val * _a2Val);
            D = Math.Max(-1.0, Math.Min(1.0, D));
            double theta2 = Math.Acos(D); // principal solution
            double k1 = _a1Val + _a2Val * Math.Cos(theta2);
            double k2 = _a2Val * Math.Sin(theta2);
            double theta1 = Math.Atan2(wy, wx) - Math.Atan2(k2, k1);
            double theta3 = phi - theta1 - theta2;

            double deg1 = NormalizeAngleDeg(theta1 * 180.0 / Math.PI);
            double deg2 = NormalizeAngleDeg(theta2 * 180.0 / Math.PI);
            double deg3 = NormalizeAngleDeg(theta3 * 180.0 / Math.PI);
            return (deg1, deg2, deg3);
        }

        private static double NormalizeAngleDeg(double a)
        {
            while (a > 180) a -= 360;
            while (a <= -180) a += 360;
            return a;
        }

        // ---------------------
        // Grid + Workspace indicator helpers
        // ---------------------
        private void CreateWorkspaceEllipse()
        {
            if (m_workspaceEllipse != null) return;
            m_workspaceEllipse = new Ellipse
            {
                Stroke = Brushes.Red,
                StrokeThickness = 1.8,
                Fill = Brushes.Transparent,
                IsHitTestVisible = false,
                Tag = "__workspace"
            };
            // ensure the ellipse is at the back of the path canvas
            if (m_pathCanvas != null)
                m_pathCanvas.Children.Insert(0, m_workspaceEllipse);
        }

        private void UpdateWorkspaceCircle()
        {
            if (m_pathCanvas == null || m_chart == null) return;
            double w = m_chart.ActualWidth;
            double h = m_chart.ActualHeight;
            if (w <= 0 || h <= 0) return;

            ReadLinkLengths();

            // use total reach (sum of links). Defaults to 333 with current textboxes
            double totalReachMm = Math.Max(1.0, _a1Val + _a2Val + _a3Val);

            // compute pixel radius using same scale as ToCanvas
            double marginFactor = 0.9;
            double minDim = Math.Min(w, h);
            double pixelsPerMm = (minDim * marginFactor) / (2.0 * totalReachMm);

            // user requested exactly 333 mm radius indicator (use actual total if you prefer)
            double indicatorRadiusMm = 333.0; // fixed indicator radius requested
            double radiusPx = indicatorRadiusMm * pixelsPerMm;

            // center in pixels
            var center = ToCanvas(0, 0, w, h);

            if (m_workspaceEllipse != null)
            {
                m_workspaceEllipse.Width = radiusPx * 2.0;
                m_workspaceEllipse.Height = radiusPx * 2.0;
                Canvas.SetLeft(m_workspaceEllipse, center.x - radiusPx);
                Canvas.SetTop(m_workspaceEllipse, center.y - radiusPx);
            }

            // redraw grid so it aligns with scale
            DrawGrid(w, h);
        }

        // Draw grid where each square equals GridSquareMm (10 mm) in robot space
        private void DrawGrid(double w, double h)
        {
            if (w <= 0 || h <= 0) return;
            ReadLinkLengths();

            double totalReachMm = Math.Max(1.0, _a1Val + _a2Val + _a3Val);
            double marginFactor = 0.9;
            double minDim = Math.Min(w, h);
            double pixelsPerMm = (minDim * marginFactor) / (2.0 * totalReachMm);
            if (pixelsPerMm <= 0) return;

            double gridPx = GridSquareMm * pixelsPerMm; // pixel size of a 10mm square

            // remove old grid elements (tagged)
            for (int i = m_pathCanvas.Children.Count - 1; i >= 0; i--)
            {
                if (m_pathCanvas.Children[i] is FrameworkElement fe && fe.Tag as string == "__grid")
                    m_pathCanvas.Children.RemoveAt(i);
            }

            // center in pixels (origin 0,0 in robot coords)
            var center = ToCanvas(0, 0, w, h);

            var gridBrush = new SolidColorBrush(Color.FromArgb(0x22, 0, 0, 0));

            // vertical lines: find range that covers canvas
            int startN = (int)Math.Floor((0 - center.x) / gridPx) - 2;
            int endN = (int)Math.Ceiling((w - center.x) / gridPx) + 2;
            for (int n = startN; n <= endN; n++)
            {
                double x = center.x + n * gridPx;
                var line = new Line
                {
                    X1 = x,
                    X2 = x,
                    Y1 = 0,
                    Y2 = h,
                    Stroke = gridBrush,
                    StrokeThickness = 1,
                    Tag = "__grid"
                };
                m_pathCanvas.Children.Insert(0, line); // at back
            }

            // horizontal lines
            int startM = (int)Math.Floor((0 - center.y) / gridPx) - 2;
            int endM = (int)Math.Ceiling((h - center.y) / gridPx) + 2;
            for (int m = startM; m <= endM; m++)
            {
                double y = center.y + m * gridPx;
                var line = new Line
                {
                    X1 = 0,
                    X2 = w,
                    Y1 = y,
                    Y2 = y,
                    Stroke = gridBrush,
                    StrokeThickness = 1,
                    Tag = "__grid"
                };
                m_pathCanvas.Children.Insert(0, line);
            }

            // optional: draw thicker axes lines for origin
            var axisBrush = new SolidColorBrush(Color.FromArgb(0x88, 0, 0, 0));
            var vAxis = new Line { X1 = center.x, X2 = center.x, Y1 = 0, Y2 = h, Stroke = axisBrush, StrokeThickness = 1.6, Tag = "__grid" };
            var hAxis = new Line { X1 = 0, X2 = w, Y1 = center.y, Y2 = center.y, Stroke = axisBrush, StrokeThickness = 1.6, Tag = "__grid" };
            m_pathCanvas.Children.Insert(0, vAxis);
            m_pathCanvas.Children.Insert(0, hAxis);
        }

        // ---------------------
        // Reachability helper (new)
        // ---------------------
        // Returns true if the target (qx,qy) with orientation phiDeg has a solvable wrist position
        // for the 2-link subproblem: |a1 - a2| <= d <= a1 + a2 where d is distance to wrist.
        private bool IsWorkspacePoseReachable(double iqx, double iqy, double phiDeg, out string reason)
        {
            // ensure current link lengths are used
            ReadLinkLengths();

            double phi = phiDeg * Math.PI / 180.0;
            // wrist position (joint between link2 and link3)
            double wx = iqx - _a3Val * Math.Cos(phi);
            double wy = iqy - _a3Val * Math.Sin(phi);
            double d = Math.Sqrt(wx * wx + wy * wy);

            double min = Math.Abs(_a1Val - _a2Val);
            double max = _a1Val + _a2Val;

            if (d > max + 1e-6)
            {
                reason = $"wrist distance {d:0.0} mm > max 2-link reach {max:0.0} mm (a1+a2).";
                return false;
            }
            if (d < min - 1e-6)
            {
                reason = $"wrist distance {d:0.0} mm < min 2-link reach {min:0.0} mm (|a1-a2|).";
                return false;
            }

            reason = string.Empty;
            return true;
        }

        // ---------------------
        // Joint limits validation (new)
        // ---------------------
        // teta1: allowed [0, 180]
        // teta2, teta3: allowed [-90, 90]
        private bool ValidateJointAngles(double deg1, double deg2, double deg3, out string reason)
        {
            if (double.IsNaN(deg1) || double.IsInfinity(deg1))
            {
                reason = "teta1 is not a valid number.";
                return false;
            }
            if (double.IsNaN(deg2) || double.IsInfinity(deg2))
            {
                reason = "teta2 is not a valid number.";
                return false;
            }
            if (double.IsNaN(deg3) || double.IsInfinity(deg3))
            {
                reason = "teta3 is not a valid number.";
                return false;
            }

            // Normalize deg1 to 0..360 for checking, but requirement is 0..180
            double d1 = deg1;
            // If deg1 normalized to [-180,180], convert negative to positive by adding 360 if needed
            if (d1 <= -180) d1 += 360;
            if (d1 < 0) { /* leave as-is because negative is invalid */ }

            if (d1 < 0.0 - 1e-6 || d1 > 180.0 + 1e-6)
            {
                reason = $"teta1 = {deg1:0.0}° out of allowed range [0°, 180°].";
                return false;
            }

            if (deg2 < -90.0 - 1e-6 || deg2 > 90.0 + 1e-6)
            {
                reason = $"teta2 = {deg2:0.0}° out of allowed range [-90°, 90°].";
                return false;
            }

            if (deg3 < -90.0 - 1e-6 || deg3 > 90.0 + 1e-6)
            {
                reason = $"teta3 = {deg3:0.0}° out of allowed range [-90°, 90°].";
                return false;
            }

            reason = string.Empty;
            return true;
        }
    }
}
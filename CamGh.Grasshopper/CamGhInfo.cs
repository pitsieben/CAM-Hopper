using Grasshopper.Kernel;
using Rhino;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Net.Http;
using System.Net.Http.Json;
using System.Text.Json.Serialization;

namespace CamGh.Grasshopper;

public sealed class CamGhInfo : GH_AssemblyInfo
{
    public override string Name => "CamGh";
    public override string Version => "0.1.0";
    public override string AuthorName => "CamGh";
    public override string Description => "Research-oriented mesh CAM with a separate local compute engine.";
    public override Guid Id => new("ed81647d-baf1-4eeb-b38d-41e47a046418");
}

internal static class EngineProcess
{
    private const uint KillOnJobClose = 0x00002000;
    private static Process? _process;
    private static string? _enginePath;
    private static IntPtr _jobHandle;
    private static readonly object Gate = new();
    private static Task? _startup;
    private static string _status = "Waiting for CAM Core";
    private static string _detail = "Place CAM Core to start the local worker.";
    private static readonly HttpClient Client = new() { Timeout = TimeSpan.FromSeconds(2) };
    static EngineProcess() => AppDomain.CurrentDomain.ProcessExit += (_, _) => ShutdownWorker();
    public static event Action<string, string>? StatusChanged;
    public static string Status { get { lock (Gate) return _status; } }
    public static string Detail { get { lock (Gate) return _detail; } }

    public static Task EnsureStartedAsync()
    {
        lock (Gate)
        {
            if (_startup is { IsCompleted: false }) return _startup;
            _startup = StartAndCheckAsync();
            return _startup;
        }
    }

    public static void EnsureStarted() => EnsureStartedAsync().GetAwaiter().GetResult();

    public static async Task<bool> IsHealthyAsync()
    {
        try
        {
            using var response = await Client.GetAsync("http://127.0.0.1:47831/health").ConfigureAwait(false);
            if (!response.IsSuccessStatusCode) return false;
            var payload = await response.Content.ReadFromJsonAsync<HealthReply>().ConfigureAwait(false);
            return payload?.Service == "CamGh.Engine";
        }
        catch { return false; }
    }

    private static async Task StartAndCheckAsync()
    {
        try
        {
            var pluginDirectory = Path.GetDirectoryName(typeof(CamGhInfo).Assembly.Location) ?? AppContext.BaseDirectory;
            var exe = Path.GetFullPath(Path.Combine(pluginDirectory, "CamGh.Engine.exe"));
            lock (Gate) _enginePath = exe;
            SetStatus("Connecting", "Checking the local CAM worker at 127.0.0.1:47831.");
            if (await IsHealthyAsync().ConfigureAwait(false))
            {
                AdoptWorker(exe);
                SetStatus("Running", "Engine is running and awaiting commands.");
                return;
            }

            if (!File.Exists(exe)) throw new FileNotFoundException("CamGh.Engine.exe must be beside CamGh.gha.", exe);

            Process worker;
            lock (Gate)
            {
                if (_process is { HasExited: false })
                    throw new InvalidOperationException("The worker process exists but its health endpoint is not responding. Check whether port 47831 is blocked or already in use.");

                worker = new Process
                {
                    StartInfo = new ProcessStartInfo(exe)
                    {
                        WorkingDirectory = pluginDirectory,
                        UseShellExecute = false,
                        CreateNoWindow = true,
                        WindowStyle = ProcessWindowStyle.Hidden,
                        RedirectStandardOutput = true,
                        RedirectStandardError = true
                    },
                    EnableRaisingEvents = true
                };
                worker.OutputDataReceived += (_, e) => { if (!string.IsNullOrWhiteSpace(e.Data)) SetStatus("Starting", e.Data); };
                worker.ErrorDataReceived += (_, e) => { if (!string.IsNullOrWhiteSpace(e.Data)) SetStatus("Startup error", e.Data); };
                worker.Exited += (_, _) =>
                {
                    string reason;
                    try { reason = $"Worker exited with code {worker.ExitCode}."; }
                    catch { reason = "Worker exited."; }
                    SetStatus("Stopped", reason);
                    lock (Gate) _startup = null;
                };
                if (!worker.Start()) throw new InvalidOperationException("Windows did not start CamGh.Engine.exe.");
                _process = worker;
                TryAssignToKillOnHostExit(worker);
                worker.BeginOutputReadLine();
                worker.BeginErrorReadLine();
            }

            SetStatus("Starting", "Waiting for the CAM worker to become ready…");
            for (var attempt = 0; attempt < 40; attempt++)
            {
                if (await IsHealthyAsync().ConfigureAwait(false))
                {
                    SetStatus("Running", "Engine is running and awaiting commands.");
                    return;
                }
                if (WorkerExited())
                    throw new InvalidOperationException("CamGh.Engine.exe exited during startup. Check the engine files beside CamGh.gha.");
                await Task.Delay(250).ConfigureAwait(false);
            }
            throw new TimeoutException("CamGh.Engine.exe started but did not answer /health within 10 seconds. Check whether port 47831 is blocked or already in use.");
        }
        catch (Exception ex)
        {
            SetStatus("Startup failed", ex.Message);
            throw;
        }
    }

    private static bool WorkerExited()
    {
        lock (Gate) return _process is null || _process.HasExited;
    }

    private static void AdoptWorker(string executable)
    {
        lock (Gate)
        {
            if (_process is { HasExited: false }) return;
            foreach (var candidate in Process.GetProcessesByName("CamGh.Engine"))
            {
                try
                {
                    if (!string.Equals(Path.GetFullPath(candidate.MainModule?.FileName ?? ""), executable, StringComparison.OrdinalIgnoreCase))
                    { candidate.Dispose(); continue; }
                    _process = candidate;
                    TryAssignToKillOnHostExit(candidate);
                    return;
                }
                catch { candidate.Dispose(); }
            }
        }
    }

    private static void TryAssignToKillOnHostExit(Process process)
    {
        if (!OperatingSystem.IsWindows()) return;
        lock (Gate)
        {
            if (_jobHandle == IntPtr.Zero)
            {
                var handle = CreateJobObject(IntPtr.Zero, null);
                if (handle == IntPtr.Zero) return;
                var info = new JobObjectExtendedLimitInformation
                {
                    BasicLimitInformation = new JobObjectBasicLimitInformation { LimitFlags = KillOnJobClose }
                };
                var size = Marshal.SizeOf<JobObjectExtendedLimitInformation>();
                var memory = Marshal.AllocHGlobal(size);
                try
                {
                    Marshal.StructureToPtr(info, memory, false);
                    if (!SetInformationJobObject(handle, 9, memory, (uint)size)) { CloseHandle(handle); return; }
                    _jobHandle = handle;
                }
                finally { Marshal.FreeHGlobal(memory); }
            }
            if (!AssignProcessToJobObject(_jobHandle, process.Handle))
            {
                // ProcessExit below remains the fallback if Windows rejects nested job assignment.
                CloseHandle(_jobHandle);
                _jobHandle = IntPtr.Zero;
            }
        }
    }

    private static void ShutdownWorker()
    {
        Process? process;
        IntPtr job;
        string? executable;
        lock (Gate) { process = _process; executable = _enginePath; job = _jobHandle; _jobHandle = IntPtr.Zero; _process = null; }
        try
        {
            if (process is null && executable is not null)
            {
                foreach (var candidate in Process.GetProcessesByName("CamGh.Engine"))
                {
                    try { if (string.Equals(Path.GetFullPath(candidate.MainModule?.FileName ?? ""), executable, StringComparison.OrdinalIgnoreCase)) { process = candidate; break; } }
                    catch { candidate.Dispose(); }
                }
            }
            if (process is { HasExited: false }) { process.Kill(entireProcessTree: true); process.WaitForExit(2000); }
        }
        catch { }
        finally { process?.Dispose(); if (job != IntPtr.Zero) CloseHandle(job); }
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct JobObjectBasicLimitInformation
    {
        public long PerProcessUserTimeLimit, PerJobUserTimeLimit;
        public uint LimitFlags;
        public UIntPtr MinimumWorkingSetSize, MaximumWorkingSetSize;
        public uint ActiveProcessLimit;
        public UIntPtr Affinity;
        public uint PriorityClass, SchedulingClass;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct IoCounters { public ulong ReadOperationCount, WriteOperationCount, OtherOperationCount, ReadTransferCount, WriteTransferCount, OtherTransferCount; }
    [StructLayout(LayoutKind.Sequential)]
    private struct JobObjectExtendedLimitInformation
    {
        public JobObjectBasicLimitInformation BasicLimitInformation;
        public IoCounters IoInfo;
        public UIntPtr ProcessMemoryLimit, JobMemoryLimit, PeakProcessMemoryUsed, PeakJobMemoryUsed;
    }
    [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)] private static extern IntPtr CreateJobObject(IntPtr attributes, string? name);
    [DllImport("kernel32.dll", SetLastError = true)] private static extern bool SetInformationJobObject(IntPtr job, int infoClass, IntPtr info, uint length);
    [DllImport("kernel32.dll", SetLastError = true)] private static extern bool AssignProcessToJobObject(IntPtr job, IntPtr process);
    [DllImport("kernel32.dll", SetLastError = true)] private static extern bool CloseHandle(IntPtr handle);

    private static void SetStatus(string status, string detail)
    {
        lock (Gate) { _status = status; _detail = detail; }
        StatusChanged?.Invoke(status, detail);
    }

    private sealed record HealthReply(
        [property: JsonPropertyName("service")] string? Service,
        [property: JsonPropertyName("version")] string? Version);
}

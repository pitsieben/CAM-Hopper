using CamGh.Core;

/// <summary>Process-wide worker state shown on the dashboard: uptime, activity log, issues and the last job scene.</summary>
public static class EngineState
{
    public const string Version = "0.3.0";
    public static readonly DateTime StartedUtc = DateTime.UtcNow;
    private static readonly object Gate = new();
    private static readonly LinkedList<ActivityEntry> Entries = new();
    private static JobScene? _scene;
    private static int _jobs, _issues;
    private static bool _busy;
    private static string _lastSummary = "Awaiting data.";

    public sealed record ActivityEntry(string Time, string Message, string Source, bool Issue);
    public sealed record StatusInfo(string Service, string Version, string StartedUtc, double UptimeSeconds, string Url, string Transport,
        int ProcessId, int Jobs, int Issues, bool Busy, bool HasJob, string? SceneCreatedUtc, string LastSummary, List<ActivityEntry> Activity);

    public static JobScene? Scene { get { lock (Gate) return _scene; } }

    public static void Log(string message, string source = "engine", bool issue = false)
    {
        lock (Gate)
        {
            if (issue) _issues++;
            Entries.AddFirst(new ActivityEntry(DateTime.Now.ToString("HH:mm:ss"), message, source, issue));
            while (Entries.Count > 200) Entries.RemoveLast();
        }
    }

    public static void BeginJob(int commands) { lock (Gate) _busy = true; Log($"Received job with {commands} command(s).", "CAM Core"); }

    public static void EndJob(JobScene scene)
    {
        lock (Gate) { _scene = scene; _jobs++; _busy = false; _lastSummary = scene.Result.Summary; }
        Log($"Finished processing in {scene.Seconds:0.0} s: {scene.Result.Operations.Count} operation(s), {scene.Result.Paths.Count} motion(s).", "CAM Core");
    }

    public static void FailJob(string error) { lock (Gate) { _busy = false; _lastSummary = "Last job failed: " + error; } Log(error, "CAM Core", issue: true); }

    public static StatusInfo Status()
    {
        lock (Gate)
            return new StatusInfo("CamGh.Engine", Version, StartedUtc.ToString("o"), (DateTime.UtcNow - StartedUtc).TotalSeconds,
                "http://127.0.0.1:47831", "HTTP/JSON, loopback only", Environment.ProcessId, _jobs, _issues, _busy, _scene is not null,
                _scene?.CreatedUtc.ToString("o"), _lastSummary, Entries.Take(60).ToList());
    }
}

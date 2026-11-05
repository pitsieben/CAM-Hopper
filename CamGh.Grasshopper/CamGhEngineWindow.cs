using Eto.Drawing;
using Eto.Forms;
using Rhino;
using System.Diagnostics;
using System.IO;

namespace CamGh.Grasshopper;

/// <summary>
/// The CAM Core "Settings" window: an Eto WebView showing the engine dashboard at http://127.0.0.1:47831/
/// (overview, tool, toolpaths, stock and part overlay, per-operation removal simulation). Falls back to the
/// system browser if no web view is available.
/// </summary>
internal static class CamGhEngineWindow
{
    public const string DashboardUrl = "http://127.0.0.1:47831/";
    private static readonly object Gate = new();
    private static CamGhDashboardForm? _form;

    public static void Show(bool simulation = false)
    {
        RhinoApp.InvokeOnUiThread(new Action(() =>
        {
            lock (Gate)
            {
                if (_form is { Visible: true }) { _form.ShowTab(simulation); _form.BringToFront(); return; }
                try
                {
                    _form = new CamGhDashboardForm(OnClosed, simulation);
                    _form.Show();
                    _form.BringToFront();
                }
                catch (Exception ex)
                {
                    _form = null;
                    var inner = ex.InnerException is { } ie ? $" <- {ie.GetType().Name}: {ie.Message}" : "";
                    RhinoApp.WriteLine($"CamGh: embedded dashboard unavailable ({ex.GetType().Name}: {ex.Message}{inner}); opening an app window.");
                    _ = OpenInBrowserWhenReadyAsync(simulation);
                }
            }
        }));
    }

    public static async Task OpenInBrowserWhenReadyAsync(bool simulation)
    {
        try { await EngineProcess.EnsureStartedAsync(); } catch { }
        OpenInBrowser(simulation);
    }

    public static void OpenInBrowser(bool simulation)
    {
        var url = DashboardUrl + (simulation ? "#simulation" : "");
        // Prefer a chrome-less Edge app window (looks like a native window), then the default browser.
        foreach (var edge in new[] {
            Environment.ExpandEnvironmentVariables(@"%ProgramFiles(x86)%\Microsoft\Edge\Application\msedge.exe"),
            Environment.ExpandEnvironmentVariables(@"%ProgramFiles%\Microsoft\Edge\Application\msedge.exe") })
        {
            if (!File.Exists(edge)) continue;
            try { Process.Start(new ProcessStartInfo(edge, $"--app={url} --window-size=1400,900") { UseShellExecute = false }); return; }
            catch { }
        }
        try { Process.Start(new ProcessStartInfo(url) { UseShellExecute = true }); }
        catch (Exception ex) { RhinoApp.WriteLine("CamGh: could not open the browser: " + ex.Message); }
    }

    private static void OnClosed(CamGhDashboardForm form)
    {
        lock (Gate) if (ReferenceEquals(_form, form)) _form = null;
    }
}

internal sealed class CamGhDashboardForm : Form
{
    private readonly WebView _web = new();
    private readonly Label _state = new() { VerticalAlignment = VerticalAlignment.Center };
    private readonly Action<CamGhDashboardForm> _onClosed;
    private bool _simulation, _loaded;

    public CamGhDashboardForm(Action<CamGhDashboardForm> onClosed, bool simulation)
    {
        _onClosed = onClosed; _simulation = simulation;
        Title = "CamGh — CAM Server";
        ClientSize = new Size(1360, 860);
        MinimumSize = new Size(900, 600);
        Resizable = true;
        BackgroundColor = Color.FromArgb(10, 16, 28);

        var browser = new Button { Text = "Open in browser" };
        browser.Click += (_, _) => CamGhEngineWindow.OpenInBrowser(_simulation);
        var reload = new Button { Text = "Reload" };
        reload.Click += (_, _) => Navigate();
        var retry = new Button { Text = "Restart engine" };
        retry.Click += async (_, _) =>
        {
            retry.Enabled = false;
            try { await EngineProcess.EnsureStartedAsync(); } catch { }
            retry.Enabled = true;
            Navigate();
        };
        _state.TextColor = Color.FromArgb(139, 149, 169);

        var bar = new TableLayout
        {
            Padding = new Padding(10, 6), Spacing = new Size(8, 0),
            Rows = { new TableRow(new TableCell(_state, true), retry, reload, browser) }
        };
        Content = new TableLayout { Rows = { new TableRow(new TableCell(_web, true)) { ScaleHeight = true }, bar } };

        _web.LoadHtml(Placeholder("Starting the CAM engine…"));
        EngineProcess.StatusChanged += UpdateStatus;
        UpdateStatus(EngineProcess.Status, EngineProcess.Detail);
        Closed += (_, _) => { EngineProcess.StatusChanged -= UpdateStatus; _onClosed(this); };
        _ = StartAsync();
    }

    public void ShowTab(bool simulation)
    {
        _simulation = simulation;
        if (_loaded) try { _web.ExecuteScript($"showTab({(simulation ? "true" : "false")})"); } catch { }
    }

    private async Task StartAsync()
    {
        try { await EngineProcess.EnsureStartedAsync(); } catch { }
        RhinoApp.InvokeOnUiThread(new Action(Navigate));
    }

    private void Navigate()
    {
        if (EngineProcess.Status != "Running")
        {
            _web.LoadHtml(Placeholder($"{EngineProcess.Status}: {EngineProcess.Detail}"));
            _loaded = false;
            return;
        }
        _web.Url = new Uri(CamGhEngineWindow.DashboardUrl + (_simulation ? "#simulation" : "#overview"));
        _loaded = true;
    }

    private void UpdateStatus(string status, string detail)
    {
        RhinoApp.InvokeOnUiThread(new Action(() =>
        {
            if (!Visible) return;
            _state.Text = $"Engine: {status}  ·  {CamGhEngineWindow.DashboardUrl}";
            if (status == "Running" && !_loaded) Navigate();
        }));
    }

    private static string Placeholder(string text) =>
        "<html><body style=\"margin:0;height:100vh;display:flex;align-items:center;justify-content:center;background:#0a101c;" +
        "color:#8b95a9;font:15px 'Segoe UI',sans-serif\"><div><span style=\"color:#ff9d2e;font-weight:700\">CAMGH</span>&nbsp;&nbsp;" +
        System.Net.WebUtility.HtmlEncode(text) + "</div></body></html>";
}

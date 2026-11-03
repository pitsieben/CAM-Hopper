using CamGh.Core;
using System.Diagnostics;

var baseDirectory = AppContext.BaseDirectory;
var builder = WebApplication.CreateBuilder(new WebApplicationOptions { Args = args, ContentRootPath = baseDirectory });
builder.WebHost.UseUrls("http://127.0.0.1:47831");
// Part meshes and multi-orientation results can exceed Kestrel's 30 MB default body limit.
builder.WebHost.ConfigureKestrel(options => options.Limits.MaxRequestBodySize = null);
var app = builder.Build();
EngineState.Log("Server started.");

app.MapGet("/health", () => Results.Ok(new { service = "CamGh.Engine", version = EngineState.Version }));

// ---- dashboard (opened by the Settings button on CAM Core)
string WebFile(string name) => Path.Combine(baseDirectory, "wwwroot", name);
app.MapGet("/", () => File.Exists(WebFile("index.html"))
    ? Results.File(WebFile("index.html"), "text/html; charset=utf-8")
    : Results.Text("Dashboard files are missing: wwwroot/index.html must be beside CamGh.Engine.exe.", "text/plain"));
app.MapGet("/api/v1/status", () => Results.Ok(EngineState.Status()));
app.MapGet("/api/v1/scene", () => EngineState.Scene is { } s
    ? Results.Ok(s.Info())
    : Results.Ok(new SceneInfo(false, "", 0, "Awaiting data.", null, 0, 0, new List<SceneOperation>(), new List<double>(), 0, 0)));
app.MapGet("/api/v1/scene/part", () => EngineState.Scene is { } s ? Results.Bytes(s.PartMeshBytes(), "application/octet-stream") : Results.NotFound());
app.MapGet("/api/v1/scene/stock/{index:int}", (int index) => EngineState.Scene?.StockMeshBytes(index) is { } b ? Results.Bytes(b, "application/octet-stream") : Results.NotFound());
app.MapGet("/api/v1/scene/paths/{index:int}", (int index) => EngineState.Scene?.PathBytes(index) is { } b ? Results.Bytes(b, "application/octet-stream") : Results.NotFound());
app.MapGet("/api/v1/scene/motions", () => EngineState.Scene is { } s ? Results.Bytes(s.MotionBytes(), "application/octet-stream") : Results.NotFound());

// ---- computation
app.MapPost("/api/v1/parallel", (ParallelRequest request) =>
{
    try { return Results.Ok(ParallelPlanner.Generate(request)); }
    catch (Exception ex) { EngineState.Log(ex.Message, "parallel", issue: true); return Results.BadRequest(new { error = ex.Message }); }
});
app.MapPost("/api/v1/horizontal-rough", (HorizontalRoughRequest request) =>
{
    try { return Results.Ok(HorizontalRoughPlanner.Generate(request)); }
    catch (Exception ex) { EngineState.Log(ex.Message, "horizontal-rough", issue: true); return Results.BadRequest(new { error = ex.Message }); }
});
app.MapPost("/api/v1/job", (CamJobRequest request) =>
{
    var watch = Stopwatch.StartNew();
    EngineState.BeginJob(request.Commands.Count);
    try
    {
        var snapshots = new List<TriDexelStock>();
        var result = CamJobPlanner.Generate(request, snapshots);
        EngineState.EndJob(new JobScene(request, result, snapshots, watch.Elapsed.TotalSeconds));
        return Results.Ok(result);
    }
    catch (Exception ex) { EngineState.FailJob(ex.Message); return Results.BadRequest(new { error = ex.Message }); }
});
app.Run();

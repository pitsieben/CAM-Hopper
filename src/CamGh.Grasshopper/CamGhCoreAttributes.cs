using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Attributes;
using System.Drawing;

namespace CamGh.Grasshopper;

/// <summary>
/// CAM Core layout with a "Settings" button under the component body (like the PRC component). The button opens
/// the engine dashboard. Mouse buttons are not inspected on purpose: reading GH_CanvasMouseEvent.Button would pull
/// in System.Windows.Forms, which RhinoCommon's .NET 7 runtime does not reference here; any click on the button opens it.
/// </summary>
internal sealed class CamGhCoreAttributes : GH_ComponentAttributes
{
    private const float ButtonHeight = 22, Gap = 4, Inset = 4;
    private RectangleF _button;
    private bool _pressed;

    public CamGhCoreAttributes(CamGhCoreComponent owner) : base(owner) { }

    protected override void Layout()
    {
        base.Layout();
        var body = GH_Convert.ToRectangle(Bounds);
        _button = new RectangleF(body.X + Inset, body.Bottom + Gap, body.Width - 2 * Inset, ButtonHeight);
        Bounds = RectangleF.Union(Bounds, new RectangleF(body.X, body.Y, body.Width, _button.Bottom - body.Y + Inset));
    }

    protected override void Render(GH_Canvas canvas, Graphics graphics, GH_CanvasChannel channel)
    {
        base.Render(canvas, graphics, channel);
        if (channel != GH_CanvasChannel.Objects) return;
        using var capsule = GH_Capsule.CreateTextCapsule(_button, _button, _pressed ? GH_Palette.Grey : GH_Palette.Black, "Settings", 2, 0);
        capsule.Render(graphics, Selected, Owner.Locked, false);
    }

    public override GH_ObjectResponse RespondToMouseDown(GH_Canvas sender, GH_CanvasMouseEvent e)
    {
        if (_button.Contains(e.CanvasLocation))
        {
            _pressed = true;
            global::Grasshopper.Instances.InvalidateCanvas();
            return GH_ObjectResponse.Capture;
        }
        return base.RespondToMouseDown(sender, e);
    }

    public override GH_ObjectResponse RespondToMouseUp(GH_Canvas sender, GH_CanvasMouseEvent e)
    {
        if (_pressed)
        {
            _pressed = false;
            global::Grasshopper.Instances.InvalidateCanvas();
            if (_button.Contains(e.CanvasLocation)) CamGhEngineWindow.Show(((CamGhCoreComponent)Owner).HasResult);
            return GH_ObjectResponse.Release;
        }
        return base.RespondToMouseUp(sender, e);
    }
}

using System.Drawing;
using System.Drawing.Drawing2D;

namespace CamGh.Grasshopper;

/// <summary>Small transparent 50x50 icons used by the CamGh Grasshopper components.</summary>
internal static class ComponentIcons
{
    private static readonly Color Ink = Color.FromArgb(27, 45, 59);
    private static readonly Color Cyan = Color.FromArgb(0, 174, 190);
    private static readonly Color Blue = Color.FromArgb(47, 112, 190);
    private static readonly Color Orange = Color.FromArgb(239, 139, 48);

    public static readonly Bitmap Core = DrawCore();
    public static readonly Bitmap CutParameters = DrawCutParameters();
    public static readonly Bitmap HorizontalRoughing = DrawRoughing();
    public static readonly Bitmap ParallelFinishing = DrawFinishing();
    public static readonly Bitmap MachineSetup = DrawMachine();
    public static readonly Bitmap Stock = DrawStock();
    public static readonly Bitmap Tool = DrawTool();
    public static readonly Bitmap Analysis = DrawAnalysis();
    public static readonly Bitmap MultiAxisRoughing = DrawMultiAxisRoughing();
    public static readonly Bitmap RegionFinishing = DrawRegionFinishing();

    private static Bitmap NewCanvas(out Graphics g)
    {
        var bitmap = new Bitmap(50, 50, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
        g = Graphics.FromImage(bitmap);
        g.Clear(Color.Transparent);
        g.SmoothingMode = SmoothingMode.AntiAlias;
        g.PixelOffsetMode = PixelOffsetMode.HighQuality;
        return bitmap;
    }

    private static Pen Pen(Color color, float width = 2.6f)
        => new(color, width) { StartCap = LineCap.Round, EndCap = LineCap.Round, LineJoin = LineJoin.Round };

    private static void Dot(Graphics g, float x, float y, float r, Color color)
    { using var b = new SolidBrush(color); g.FillEllipse(b, x-r, y-r, 2*r, 2*r); }

    private static void Line(Graphics g, Color color, float width, params PointF[] points)
    { using var p = Pen(color, width); g.DrawLines(p, points); }

    private static Bitmap DrawCore()
    {
        var bitmap=NewCanvas(out var g);using(g)
        {
            using var ring=Pen(Cyan,3);g.DrawEllipse(ring,7,7,36,36);
            using var inner=Pen(Ink,2.2f);g.DrawEllipse(inner,17,17,16,16);
            using var core=new SolidBrush(Orange);g.FillEllipse(core,21,21,8,8);
            Dot(g,25,7,2.5f,Ink);Dot(g,43,25,2.5f,Ink);Dot(g,25,43,2.5f,Ink);Dot(g,7,25,2.5f,Ink);
            Line(g,Blue,2,new PointF(25,11),new PointF(25,17));Line(g,Blue,2,new PointF(39,25),new PointF(33,25));
            Line(g,Blue,2,new PointF(25,39),new PointF(25,33));Line(g,Blue,2,new PointF(11,25),new PointF(17,25));
        }return bitmap;
    }

    private static Bitmap DrawCutParameters()
    {
        var bitmap=NewCanvas(out var g);using(g)
        {
            for(int i=0;i<3;i++)
            {
                float y=13+i*12;Line(g,Ink,2.2f,new PointF(7,y),new PointF(43,y));
            }
            Dot(g,17,13,4.2f,Cyan);Dot(g,32,25,4.2f,Orange);Dot(g,22,37,4.2f,Blue);
            using var outline=Pen(Ink,1.5f);g.DrawEllipse(outline,12.5f,8.5f,9,9);g.DrawEllipse(outline,27.5f,20.5f,9,9);g.DrawEllipse(outline,17.5f,32.5f,9,9);
        }return bitmap;
    }

    private static Bitmap DrawRoughing()
    {
        var bitmap=NewCanvas(out var g);using(g)
        {
            using var a=Pen(Cyan,2.3f);g.DrawRectangle(a,7,7,36,36);
            using var b=Pen(Blue,2.3f);g.DrawRectangle(b,13,13,24,24);
            using var c=Pen(Orange,2.3f);g.DrawRectangle(c,19,19,12,12);
            Line(g,Ink,2,new PointF(25,2),new PointF(25,8));Line(g,Ink,2,new PointF(22,5),new PointF(25,8),new PointF(28,5));
        }return bitmap;
    }

    private static Bitmap DrawFinishing()
    {
        var bitmap=NewCanvas(out var g);using(g)
        {
            using var p1=Pen(Cyan,2.4f);g.DrawBezier(p1,8,12,16,8,31,8,42,12);
            using var p2=Pen(Blue,2.4f);g.DrawBezier(p2,8,22,16,18,31,18,42,22);
            using var p3=Pen(Orange,2.4f);g.DrawBezier(p3,8,32,16,28,31,28,42,32);
            using var p4=Pen(Ink,2.4f);g.DrawBezier(p4,8,42,16,38,31,38,42,42);
        }return bitmap;
    }

    private static Bitmap DrawMachine()
    {
        var bitmap=NewCanvas(out var g);using(g)
        {
            Line(g,Orange,3,new PointF(25,40),new PointF(25,9));Line(g,Cyan,3,new PointF(25,40),new PointF(42,40));
            Line(g,Blue,3,new PointF(25,40),new PointF(11,28));
            Line(g,Orange,2,new PointF(21,14),new PointF(25,8),new PointF(29,14));
            Line(g,Cyan,2,new PointF(36,36),new PointF(43,40),new PointF(36,44));
            Line(g,Blue,2,new PointF(16,32),new PointF(10,27),new PointF(17,26));Dot(g,25,40,3,Ink);
        }return bitmap;
    }

    private static Bitmap DrawStock()
    {
        var bitmap=NewCanvas(out var g);using(g)
        {
            Line(g,Ink,2.5f,new PointF(25,6),new PointF(43,15),new PointF(43,36),new PointF(25,45),new PointF(7,36),new PointF(7,15),new PointF(25,6));
            Line(g,Cyan,2.5f,new PointF(7,15),new PointF(25,24),new PointF(43,15));
            Line(g,Blue,2.5f,new PointF(25,24),new PointF(25,45));
            Line(g,Orange,2.2f,new PointF(16,11),new PointF(34,20),new PointF(34,33));
        }return bitmap;
    }

    private static Bitmap DrawTool()
    {
        var bitmap=NewCanvas(out var g);using(g)
        {
            using var body=Pen(Ink,3);g.DrawRectangle(body,18,5,14,23);
            using var tip=Pen(Cyan,3);g.DrawLine(tip,18,28,18,39);g.DrawLine(tip,32,28,32,39);g.DrawLine(tip,18,39,23,44);g.DrawLine(tip,32,39,27,44);g.DrawLine(tip,23,44,27,44);
            Line(g,Orange,2.2f,new PointF(20,29),new PointF(30,18));Line(g,Blue,2.2f,new PointF(20,39),new PointF(30,29));
            Line(g,Ink,2,new PointF(22,5),new PointF(22,2));Line(g,Ink,2,new PointF(28,5),new PointF(28,2));Line(g,Ink,2,new PointF(22,2),new PointF(28,2));
        }return bitmap;
    }

    private static Bitmap DrawAnalysis()
    {
        var bitmap=NewCanvas(out var g);using(g)
        {
            using var axes=Pen(Ink,2);g.DrawLine(axes,8,41,8,9);g.DrawLine(axes,8,41,43,41);
            Line(g,Cyan,2.4f,new PointF(11,34),new PointF(19,27),new PointF(26,30),new PointF(37,14));
            Dot(g,11,34,2.8f,Blue);Dot(g,19,27,2.8f,Blue);Dot(g,26,30,2.8f,Orange);Dot(g,37,14,2.8f,Cyan);
            using var ring=Pen(Orange,1.8f);g.DrawEllipse(ring,29,7,13,13);
        }return bitmap;
    }

    private static Bitmap DrawMultiAxisRoughing()
    {
        var bitmap=NewCanvas(out var g);using(g)
        {
            using var block=Pen(Ink,2.2f);g.DrawRectangle(block,14,14,22,22);
            using var inner=Pen(Blue,2f);g.DrawEllipse(inner,19,19,12,12);
            // Three tool axes approaching from the top, the left and a diagonal.
            Line(g,Cyan,2.6f,new PointF(25,3),new PointF(25,12));Line(g,Cyan,2f,new PointF(22,9),new PointF(25,12),new PointF(28,9));
            Line(g,Orange,2.6f,new PointF(3,25),new PointF(12,25));Line(g,Orange,2f,new PointF(9,22),new PointF(12,25),new PointF(9,28));
            Line(g,Blue,2.6f,new PointF(46,4),new PointF(38,12));Line(g,Blue,2f,new PointF(38,8),new PointF(38,12),new PointF(42,12));
        }return bitmap;
    }

    private static Bitmap DrawRegionFinishing()
    {
        var bitmap=NewCanvas(out var g);using(g)
        {
            using var outline=Pen(Ink,2.2f);g.DrawEllipse(outline,7,7,36,36);
            using var p1=Pen(Cyan,2.2f);g.DrawBezier(p1,10,18,18,14,30,14,40,18);g.DrawBezier(p1,9,24,18,20,30,20,41,24);
            using var p2=Pen(Orange,2.2f);g.DrawLine(p2,15,30,15,40);g.DrawLine(p2,21,30,21,42);g.DrawLine(p2,27,30,27,42);g.DrawLine(p2,33,30,33,40);
            Line(g,Blue,2f,new PointF(7,28),new PointF(43,28));
        }return bitmap;
    }
}

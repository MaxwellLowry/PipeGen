using Leap71.ShapeKernel;
using PicoGK;
using System;
using System.Collections.Generic;
using System.Numerics;

try
{
    // Launch the app with a voxel resolution of 0.3f
    Library.Go(0.3f, Coding4Engineers.Pipe.App.Run);
}
catch (Exception e)
{
    Console.WriteLine(e.ToString());
}

namespace Coding4Engineers
{
    namespace Pipe
    {
        public static class App
        {
            // ======= User Parameters =======
            private const float OUTER_RADIUS = 20f;   // Pipe outer radius
            private const float WALL_THICKNESS = 4f;    // Pipe wall thickness
            private const float FLANGE_THICKNESS = 4f;    // Flange thickness
            private const int HOLE_COUNT = 8;     // Number of bolt holes
            private const float HOLE_RADIUS = 1.5f;  // Bolt-hole radius
            private const float VOX_RESOLUTION = 0.1f;  // For reference only (set in Library.Go above)

            // Bolt circle slightly wider than pipe outer radius
            private static readonly float BOLT_CIRCLE_RADIUS = OUTER_RADIUS * 1.15f;

            // Sweep settings
            private const int SPLINE_SAMPLES = 400;   // Number of samples along spline
            private const float MIN_SEG_LEN = 0.25f; // Cull tiny segments to avoid artifacts
            private const float OVERLAP_FACTOR = 0.1f;  // Extend each segment by r*factor to fuse neighbors

            // Toggle: false = straight pipe, true = bent pipe
            private const bool USE_BENT_PIPE = true;

            public static void Run()
            {
                Voxels pipeVoxels;

                if (!USE_BENT_PIPE)
                {
                    // ---------- Straight pipe ----------
                    Vector3 start = new(0, 0, 0);
                    Vector3 end = new(50, 0, 0);

                    pipeVoxels = MakeHollowPipe(start, end, OUTER_RADIUS, WALL_THICKNESS);

                    // Flanges
                    var flangeStart = MakeFlange(start, end - start, OUTER_RADIUS, FLANGE_THICKNESS);
                    var flangeEnd = MakeFlange(end, start - end, OUTER_RADIUS, FLANGE_THICKNESS);
                    pipeVoxels.BoolAdd(flangeStart);
                    pipeVoxels.BoolAdd(flangeEnd);

                    // Bolt holes (use a shared reference-up so both flanges are clocked the same)
                    var refUp = Vector3.UnitX; // pick any world axis; code will re-resolve if parallel
                    var boltStart = MakeBoltHoles(start, end - start, HOLE_COUNT, HOLE_RADIUS, BOLT_CIRCLE_RADIUS, FLANGE_THICKNESS, refUp);
                    var boltEnd = MakeBoltHoles(end, start - end, HOLE_COUNT, HOLE_RADIUS, BOLT_CIRCLE_RADIUS, FLANGE_THICKNESS, refUp);
                    pipeVoxels.BoolSubtract(boltStart);
                    pipeVoxels.BoolSubtract(boltEnd);
                }
                else
                {
                    // ---------- Bent pipe ----------
                    var controlPoints = new List<Vector3>
                    {
                        new(0,  0,  0),  
                        new(0, 40,  0),   
                        new(0, 50, 20),   
                        new(0, 60, 60),  
                        new(40,60,20),
                        new(50, 80, 0),
                        new(100, 80, 0),
                        new(150, 80, 20),
                        new(200, 80,  0),
                        new(150, 90,  60)
                    };


                    // Sample & clean once so sweep / flanges share identical geometry & tangents
                    var splinePts = SampleClean(controlPoints, SPLINE_SAMPLES, MIN_SEG_LEN);

                    // Hollow tube sweep with overlap for watertight joints
                    pipeVoxels = SweepHollowTube(splinePts, OUTER_RADIUS, WALL_THICKNESS, OVERLAP_FACTOR);

                    // Start/end frames for flanges from actual sweep tangents
                    Vector3 start = splinePts[0];
                    Vector3 end = splinePts[^1];
                    Vector3 startDir = Vector3.Normalize(splinePts[1] - splinePts[0]);
                    Vector3 endDir = Vector3.Normalize(splinePts[^1] - splinePts[^2]);

                    var flangeStart = MakeFlange(start, startDir, OUTER_RADIUS, FLANGE_THICKNESS);
                    var flangeEnd = MakeFlange(end, endDir, OUTER_RADIUS, FLANGE_THICKNESS);
                    pipeVoxels.BoolAdd(flangeStart);
                    pipeVoxels.BoolAdd(flangeEnd);

                    // Bolt holes on both flanges, clocked identically via shared reference-up
                    var refUp = Vector3.UnitX;
                    var boltStart = MakeBoltHoles(start, startDir, HOLE_COUNT, HOLE_RADIUS, BOLT_CIRCLE_RADIUS, FLANGE_THICKNESS, refUp);
                    var boltEnd = MakeBoltHoles(end, endDir, HOLE_COUNT, HOLE_RADIUS, BOLT_CIRCLE_RADIUS, FLANGE_THICKNESS, refUp);
                    pipeVoxels.BoolSubtract(boltStart);
                    pipeVoxels.BoolSubtract(boltEnd);
                }

                Library.oViewer().Add(pipeVoxels);
            }

            // ======= Geometry Builders =======

            // Straight hollow pipe between two points
            private static Voxels MakeHollowPipe(Vector3 start, Vector3 end, float outerRadius, float wallThickness)
            {
                float innerRadius = MathF.Max(outerRadius - wallThickness, 0.01f);

                var latOuter = new Lattice();
                latOuter.AddBeam(start, outerRadius, end, outerRadius, false);

                var latInner = new Lattice();
                latInner.AddBeam(start, innerRadius, end, innerRadius, false);

                var voxOuter = new Voxels(latOuter);
                var voxInner = new Voxels(latInner);
                voxOuter.BoolSubtract(voxInner);
                return voxOuter;
            }

            // Flange ring centered at `center`, normal aligned to `pipeDirection`
            private static Voxels MakeFlange(Vector3 center, Vector3 pipeDirection, float pipeOuterRadius, float flangeThickness)
            {
                Vector3 dir = SafeNormalize(pipeDirection);
                Vector3 half = dir * (flangeThickness * 0.5f);
                Vector3 p1 = center - half;
                Vector3 p2 = center + half;

                float flangeOuter = pipeOuterRadius * 1.35f;
                float flangeInner = pipeOuterRadius - WALL_THICKNESS;

                var latOuter = new Lattice();
                latOuter.AddBeam(p1, flangeOuter, p2, flangeOuter, false);

                var latInner = new Lattice();
                latInner.AddBeam(p1, flangeInner, p2, flangeInner, false);

                var vox = new Voxels(latOuter);
                vox.BoolSubtract(new Voxels(latInner));
                return vox;
            }

            // Bolt holes around a circle on a flange, with shared reference-up to lock rotation
            private static Voxels MakeBoltHoles(
                Vector3 center,
                Vector3 normal,
                int holeCount,
                float holeRadius,
                float boltCircleRadius,
                float flangeThickness,
                Vector3 referenceUp)
            {
                var lat = new Lattice();

                Vector3 up = SafeNormalize(normal);

                // Use a stable reference-up; if parallel to `up`, pick an alternate axis
                Vector3 refUp = referenceUp;
                if (MathF.Abs(Vector3.Dot(up, refUp)) > 0.99f)
                    refUp = MathF.Abs(up.Y) < 0.9f ? Vector3.UnitY : Vector3.UnitX;

                Vector3 right = SafeNormalize(Vector3.Cross(up, refUp));
                Vector3 forward = SafeNormalize(Vector3.Cross(right, up));

                for (int i = 0; i < holeCount; i++)
                {
                    float angle = (float)(2 * Math.PI * i / holeCount);
                    Vector3 radial = MathF.Cos(angle) * right + MathF.Sin(angle) * forward;

                    Vector3 a = center - (flangeThickness * 0.5f) * up + radial * boltCircleRadius;
                    Vector3 b = center + (flangeThickness * 0.5f) * up + radial * boltCircleRadius;

                    lat.AddBeam(a, holeRadius, b, holeRadius, false);
                }

                return new Voxels(lat);
            }

            // Sweep a hollow tube along a polyline with segment overlap for watertight joins
            private static Voxels SweepHollowTube(List<Vector3> pts, float outerRadius, float wallThickness, float overlapFactor)
            {
                float innerRadius = MathF.Max(outerRadius - wallThickness, 0.01f);
                float overlap = outerRadius * MathF.Max(overlapFactor, 0f);

                var latOuter = new Lattice();
                var latInner = new Lattice();

                for (int i = 0; i < pts.Count - 1; i++)
                {
                    Vector3 a = pts[i];
                    Vector3 b = pts[i + 1];
                    Vector3 dir = SafeNormalize(b - a);

                    Vector3 aEx = a - dir * overlap;
                    Vector3 bEx = b + dir * overlap;

                    latOuter.AddBeam(aEx, outerRadius, bEx, outerRadius, false);
                    latInner.AddBeam(aEx, innerRadius, bEx, innerRadius, false);
                }

                var voxOuter = new Voxels(latOuter);
                var voxInner = new Voxels(latInner);
                voxOuter.BoolSubtract(voxInner);
                return voxOuter;
            }

            // ======= Spline Utilities =======

            // Lightweight wrapper around PicoGK ControlPointSpline
            public class BSplinePipeCurve
            {
                private readonly ControlPointSpline _spline;
                public BSplinePipeCurve(List<Vector3> controlPoints) => _spline = new ControlPointSpline(controlPoints);
                public List<Vector3> SamplePoints(int nSamples = 200) => _spline.aGetPoints((uint)nSamples);
            }

            // Sample a spline and cull micro segments
            private static List<Vector3> SampleClean(List<Vector3> controlPoints, int samples, float minSegLen)
            {
                var pts = new BSplinePipeCurve(controlPoints).SamplePoints(samples);
                return CleanPath(pts, minSegLen);
            }

            // Remove consecutive duplicates / too-short segments
            private static List<Vector3> CleanPath(List<Vector3> pts, float minSegLen)
            {
                if (pts == null || pts.Count == 0) return new List<Vector3>();

                var outPts = new List<Vector3> { pts[0] };
                for (int i = 1; i < pts.Count; i++)
                {
                    if ((pts[i] - outPts[^1]).Length() >= minSegLen)
                        outPts.Add(pts[i]);
                }

                if (outPts.Count == 1 && pts.Count > 1)
                    outPts.Add(pts[^1]);

                return outPts;
            }

            // Safe normalize: returns UnitX if input is near-zero
            private static Vector3 SafeNormalize(Vector3 v)
            {
                float len = v.Length();
                return (len > 1e-6f) ? v / len : Vector3.UnitX;
            }
        }
    }
}



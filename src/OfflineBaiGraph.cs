using System.Numerics;

namespace AAEmu.Game.Bots.Navigation.Offline;

public static class NavigationCoordinates
{
    public const float PathTileSize = 256f;
    public const float WorldCellSize = 1024f;

    public static int PathTile(float worldCoordinate) => (int)MathF.Floor(worldCoordinate / PathTileSize);

    public static Vector3 PathTileToWorld(int tileX, int tileY, Vector3 local) =>
        local + new Vector3(tileX * PathTileSize, tileY * PathTileSize, 0f);

    public static Vector2 WorldToMapPixel(
        Vector2 world,
        Vector2 worldMinimum,
        Vector2 worldSize,
        Vector2 imageSize)
    {
        if (worldSize.X <= 0f || worldSize.Y <= 0f || imageSize.X <= 0f || imageSize.Y <= 0f)
            throw new ArgumentOutOfRangeException(nameof(worldSize));

        return new Vector2(
            (world.X - worldMinimum.X) / worldSize.X * imageSize.X,
            imageSize.Y - (world.Y - worldMinimum.Y) / worldSize.Y * imageSize.Y);
    }
}

public readonly record struct OfflineBaiNode(
    Vector3 Position,
    Vector3 Direction,
    int TileX,
    int TileY,
    long SourceId);

public readonly record struct OfflineBaiLink(
    int From,
    int To,
    double AuthoredLength,
    double MaxPassRadius,
    double Exposure,
    double MinWaterDepth,
    double MaxWaterDepth,
    bool IsStitch = false);

public readonly record struct OfflineBaiSourceNode(long Id, Vector3 Position, Vector3 Direction);

public readonly record struct OfflineBaiSourceLink(
    long FromId,
    long ToId,
    double Length,
    double MaxPassRadius = 0d,
    double Exposure = 0d,
    double MinWaterDepth = 0d,
    double MaxWaterDepth = 0d);

public sealed record OfflineBaiRoute(
    bool IsSuccess,
    IReadOnlyList<Vector3> Points,
    double Cost,
    int VisitedNodes,
    string Failure);

public sealed class RoadPriorIndex
{
    private readonly float _influence;
    private readonly float _cellSize;
    private readonly Dictionary<(int X, int Y), List<Segment>> _segments = [];
    private int _segmentCount;

    public RoadPriorIndex(float influence = 24f, float cellSize = 64f)
    {
        if (!float.IsFinite(influence) || influence <= 0f)
            throw new ArgumentOutOfRangeException(nameof(influence));
        if (!float.IsFinite(cellSize) || cellSize <= 0f)
            throw new ArgumentOutOfRangeException(nameof(cellSize));
        _influence = influence;
        _cellSize = cellSize;
    }

    public int SegmentCount => _segmentCount;

    public void Add(IEnumerable<Vector3> points)
    {
        var polyline = (points ?? throw new ArgumentNullException(nameof(points))).ToArray();
        for (var index = 1; index < polyline.Length; index++)
        {
            var start = new Vector2(polyline[index - 1].X, polyline[index - 1].Y);
            var end = new Vector2(polyline[index].X, polyline[index].Y);
            if (!IsFinite(start) || !IsFinite(end) || Vector2.DistanceSquared(start, end) <= 0.0001f)
                continue;

            var segment = new Segment(start, end);
            var minimum = Vector2.Min(start, end) - new Vector2(_influence);
            var maximum = Vector2.Max(start, end) + new Vector2(_influence);
            for (var x = Bucket(minimum.X); x <= Bucket(maximum.X); x++)
            for (var y = Bucket(minimum.Y); y <= Bucket(maximum.Y); y++)
            {
                if (!_segments.TryGetValue((x, y), out var bucket))
                    _segments[(x, y)] = bucket = [];
                bucket.Add(segment);
            }
            _segmentCount++;
        }
    }

    public float Distance(Vector3 point)
    {
        if (_segmentCount == 0)
            return 0f;
        if (!_segments.TryGetValue((Bucket(point.X), Bucket(point.Y)), out var segments))
            return _influence;

        var target = new Vector2(point.X, point.Y);
        var best = _influence;
        foreach (var segment in segments)
            best = MathF.Min(best, DistanceToSegment(target, segment.Start, segment.End));
        return best;
    }

    public double CostMultiplier(Vector3 point, double offRoadPenalty)
    {
        if (!double.IsFinite(offRoadPenalty) || offRoadPenalty < 0d)
            throw new ArgumentOutOfRangeException(nameof(offRoadPenalty));
        return _segmentCount == 0 ? 1d : 1d + offRoadPenalty * Distance(point) / _influence;
    }

    private int Bucket(float value) => (int)MathF.Floor(value / _cellSize);

    private static float DistanceToSegment(Vector2 point, Vector2 start, Vector2 end)
    {
        var delta = end - start;
        var lengthSquared = delta.LengthSquared();
        if (lengthSquared <= float.Epsilon)
            return Vector2.Distance(point, start);
        var projection = Math.Clamp(Vector2.Dot(point - start, delta) / lengthSquared, 0f, 1f);
        return Vector2.Distance(point, start + projection * delta);
    }

    private static bool IsFinite(Vector2 point) => float.IsFinite(point.X) && float.IsFinite(point.Y);

    private readonly record struct Segment(Vector2 Start, Vector2 End);
}

public sealed record OfflineTraversalCostProfile
{
    public double PreferredGrade { get; init; } = 0.35d;
    public double GradePenalty { get; init; } = 6d;
    public double WaterPenalty { get; init; } = 4d;

    internal void Validate()
    {
        if (!double.IsFinite(PreferredGrade) || PreferredGrade < 0d ||
            !double.IsFinite(GradePenalty) || GradePenalty < 0d ||
            !double.IsFinite(WaterPenalty) || WaterPenalty < 0d)
        {
            throw new ArgumentOutOfRangeException(nameof(OfflineTraversalCostProfile));
        }
    }
}

/// <summary>Spatial water test built from object.dat contours for offline cost generation.</summary>
public sealed class WaterTraversalIndex
{
    private readonly float _cellSize;
    private readonly float _bridgeClearance;
    private readonly List<WaterShape> _shapes = [];
    private readonly Dictionary<(int X, int Y), List<int>> _buckets = [];

    public WaterTraversalIndex(float cellSize = 128f, float bridgeClearance = 1f)
    {
        if (!float.IsFinite(cellSize) || cellSize <= 0f)
            throw new ArgumentOutOfRangeException(nameof(cellSize));
        if (!float.IsFinite(bridgeClearance) || bridgeClearance < 0f)
            throw new ArgumentOutOfRangeException(nameof(bridgeClearance));
        _cellSize = cellSize;
        _bridgeClearance = bridgeClearance;
    }

    public int ShapeCount => _shapes.Count;

    public void AddPolygon(IEnumerable<Vector3> points, float surfaceHeight)
    {
        var polygon = (points ?? throw new ArgumentNullException(nameof(points)))
            .Select(point => new Vector2(point.X, point.Y))
            .Where(IsFinite)
            .ToArray();
        if (polygon.Length < 3 || !float.IsFinite(surfaceHeight))
            return;
        Add(new WaterShape(polygon, 0f, surfaceHeight, false));
    }

    public void AddCorridor(IEnumerable<Vector3> points, float width, float surfaceHeight)
    {
        var centerline = (points ?? throw new ArgumentNullException(nameof(points)))
            .Select(point => new Vector2(point.X, point.Y))
            .Where(IsFinite)
            .ToArray();
        if (centerline.Length < 2 || !float.IsFinite(width) || width <= 0f ||
            !float.IsFinite(surfaceHeight))
        {
            return;
        }
        Add(new WaterShape(centerline, width * 0.5f, surfaceHeight, true));
    }

    public bool Contains(Vector3 point)
    {
        if (!_buckets.TryGetValue((Bucket(point.X), Bucket(point.Y)), out var candidates))
            return false;
        var target = new Vector2(point.X, point.Y);
        foreach (var index in candidates)
        {
            var shape = _shapes[index];
            if (point.Z > shape.SurfaceHeight + _bridgeClearance ||
                target.X < shape.Minimum.X || target.X > shape.Maximum.X ||
                target.Y < shape.Minimum.Y || target.Y > shape.Maximum.Y)
            {
                continue;
            }
            if (shape.IsCorridor ? InCorridor(target, shape) : InPolygon(target, shape.Points))
                return true;
        }
        return false;
    }

    private void Add(WaterShape shape)
    {
        var index = _shapes.Count;
        _shapes.Add(shape);
        for (var x = Bucket(shape.Minimum.X); x <= Bucket(shape.Maximum.X); x++)
        for (var y = Bucket(shape.Minimum.Y); y <= Bucket(shape.Maximum.Y); y++)
        {
            if (!_buckets.TryGetValue((x, y), out var bucket))
                _buckets[(x, y)] = bucket = [];
            bucket.Add(index);
        }
    }

    private static bool InCorridor(Vector2 target, WaterShape shape)
    {
        for (var index = 1; index < shape.Points.Length; index++)
        {
            if (DistanceToSegment(target, shape.Points[index - 1], shape.Points[index]) <= shape.HalfWidth)
                return true;
        }
        return false;
    }

    private static bool InPolygon(Vector2 target, IReadOnlyList<Vector2> polygon)
    {
        var inside = false;
        for (int current = 0, previous = polygon.Count - 1; current < polygon.Count; previous = current++)
        {
            var a = polygon[current];
            var b = polygon[previous];
            if ((a.Y > target.Y) == (b.Y > target.Y))
                continue;
            var crossingX = (b.X - a.X) * (target.Y - a.Y) / (b.Y - a.Y) + a.X;
            if (target.X < crossingX)
                inside = !inside;
        }
        return inside;
    }

    private static float DistanceToSegment(Vector2 point, Vector2 start, Vector2 end)
    {
        var delta = end - start;
        var lengthSquared = delta.LengthSquared();
        if (lengthSquared <= float.Epsilon)
            return Vector2.Distance(point, start);
        var projection = Math.Clamp(Vector2.Dot(point - start, delta) / lengthSquared, 0f, 1f);
        return Vector2.Distance(point, start + projection * delta);
    }

    private int Bucket(float value) => (int)MathF.Floor(value / _cellSize);
    private static bool IsFinite(Vector2 point) => float.IsFinite(point.X) && float.IsFinite(point.Y);

    private sealed class WaterShape
    {
        public WaterShape(Vector2[] points, float halfWidth, float surfaceHeight, bool isCorridor)
        {
            Points = points;
            HalfWidth = halfWidth;
            SurfaceHeight = surfaceHeight;
            IsCorridor = isCorridor;
            Minimum = points.Aggregate(Vector2.Min) - new Vector2(halfWidth);
            Maximum = points.Aggregate(Vector2.Max) + new Vector2(halfWidth);
        }

        public Vector2[] Points { get; }
        public float HalfWidth { get; }
        public float SurfaceHeight { get; }
        public bool IsCorridor { get; }
        public Vector2 Minimum { get; }
        public Vector2 Maximum { get; }
    }
}

/// <summary>Fine BAI graph used only while building compact route corridors.</summary>
public sealed class OfflineBaiGraph
{
    private readonly List<OfflineBaiNode> _nodes = [];
    private readonly List<List<OfflineBaiLink>> _adjacency = [];

    public IReadOnlyList<OfflineBaiNode> Nodes => _nodes;
    public long SourceLinkCount { get; private set; }
    public long StitchLinkCount { get; private set; }

    public void AddTile(
        int tileX,
        int tileY,
        IEnumerable<OfflineBaiSourceNode> nodes,
        IEnumerable<OfflineBaiSourceLink> links)
    {
        var local = new Dictionary<long, int>();
        foreach (var source in nodes ?? throw new ArgumentNullException(nameof(nodes)))
        {
            if (!IsFinite(source.Position) || !local.TryAdd(source.Id, _nodes.Count))
                continue;
            _nodes.Add(new OfflineBaiNode(source.Position, source.Direction, tileX, tileY, source.Id));
            _adjacency.Add([]);
        }

        foreach (var source in links ?? throw new ArgumentNullException(nameof(links)))
        {
            if (!local.TryGetValue(source.FromId, out var from) || !local.TryGetValue(source.ToId, out var to) ||
                from == to)
            {
                continue;
            }

            _adjacency[from].Add(new OfflineBaiLink(
                from,
                to,
                source.Length,
                source.MaxPassRadius,
                source.Exposure,
                source.MinWaterDepth,
                source.MaxWaterDepth));
            SourceLinkCount++;
        }
    }

    public long StitchTileBoundaries(float tolerance = 1.25f, float boundaryDistance = 2f)
    {
        if (!float.IsFinite(tolerance) || tolerance <= 0f)
            throw new ArgumentOutOfRangeException(nameof(tolerance));
        if (!float.IsFinite(boundaryDistance) || boundaryDistance <= 0f)
            throw new ArgumentOutOfRangeException(nameof(boundaryDistance));

        var buckets = new Dictionary<(int X, int Y, int Z), List<int>>();
        for (var index = 0; index < _nodes.Count; index++)
        {
            var node = _nodes[index];
            if (!NearTileBoundary(node.Position.X, boundaryDistance) &&
                !NearTileBoundary(node.Position.Y, boundaryDistance))
            {
                continue;
            }

            var key = Quantize(node.Position, tolerance);
            for (var dx = -1; dx <= 1; dx++)
            for (var dy = -1; dy <= 1; dy++)
            for (var dz = -1; dz <= 1; dz++)
            {
                if (!buckets.TryGetValue((key.X + dx, key.Y + dy, key.Z + dz), out var candidates))
                    continue;
                foreach (var candidate in candidates)
                {
                    var other = _nodes[candidate];
                    if (other.TileX == node.TileX && other.TileY == node.TileY)
                        continue;
                    var distance = Vector3.Distance(other.Position, node.Position);
                    if (!float.IsFinite(distance) || distance > tolerance)
                        continue;
                    AddStitch(index, candidate, distance);
                    AddStitch(candidate, index, distance);
                }
            }

            if (!buckets.TryGetValue(key, out var own))
                buckets[key] = own = [];
            own.Add(index);
        }
        return StitchLinkCount;
    }

    public int FindNearest(Vector3 target)
    {
        var best = -1;
        var bestDistance = float.PositiveInfinity;
        for (var index = 0; index < _nodes.Count; index++)
        {
            var distance = Vector3.DistanceSquared(_nodes[index].Position, target);
            if (distance >= bestDistance)
                continue;
            bestDistance = distance;
            best = index;
        }
        return best;
    }

    public OfflineBaiRoute FindRoute(
        int start,
        int goal,
        RoadPriorIndex roadPrior = null,
        double offRoadPenalty = 0d,
        WaterTraversalIndex water = null,
        OfflineTraversalCostProfile costProfile = null)
    {
        if (start < 0 || start >= _nodes.Count || goal < 0 || goal >= _nodes.Count)
            return new OfflineBaiRoute(false, [], 0d, 0, "endpoint_not_found");
        costProfile ??= new OfflineTraversalCostProfile();
        costProfile.Validate();

        var queue = new PriorityQueue<int, double>();
        var distances = Enumerable.Repeat(double.PositiveInfinity, _nodes.Count).ToArray();
        var previous = Enumerable.Repeat(-1, _nodes.Count).ToArray();
        var visited = 0;
        distances[start] = 0d;
        queue.Enqueue(start, Heuristic(start, goal));

        while (queue.TryDequeue(out var current, out var priority))
        {
            if (priority > distances[current] + Heuristic(current, goal) + 0.001d)
                continue;
            visited++;
            if (current == goal)
                break;

            foreach (var link in _adjacency[current])
            {
                var from = _nodes[current].Position;
                var to = _nodes[link.To].Position;
                var geometric = Vector3.Distance(from, to);
                if (!float.IsFinite(geometric) || geometric <= 0f || geometric > 100f)
                    continue;
                var baseCost = double.IsFinite(link.AuthoredLength) && link.AuthoredLength > 0d
                    ? Math.Max(geometric, link.AuthoredLength)
                    : geometric;
                var midpoint = Vector3.Lerp(from, to, 0.5f);
                var planar = Vector2.Distance(new Vector2(from.X, from.Y), new Vector2(to.X, to.Y));
                var grade = Math.Abs(to.Z - from.Z) / Math.Max(planar, 0.25f);
                var traversalMultiplier = 1d +
                    Math.Max(0d, grade - costProfile.PreferredGrade) * costProfile.GradePenalty;
                if (water?.Contains(midpoint) == true)
                    traversalMultiplier += costProfile.WaterPenalty;
                var multiplier = traversalMultiplier *
                                 (roadPrior?.CostMultiplier(midpoint, offRoadPenalty) ?? 1d);
                var candidate = distances[current] + baseCost * multiplier;
                if (candidate >= distances[link.To])
                    continue;
                distances[link.To] = candidate;
                previous[link.To] = current;
                queue.Enqueue(link.To, candidate + Heuristic(link.To, goal));
            }
        }

        if (start != goal && previous[goal] < 0)
            return new OfflineBaiRoute(false, [], 0d, visited, "disconnected");

        var points = new List<Vector3>();
        for (var current = goal; current >= 0; current = previous[current])
        {
            points.Add(_nodes[current].Position);
            if (current == start)
                break;
        }
        points.Reverse();
        return new OfflineBaiRoute(true, points, distances[goal], visited, string.Empty);
    }

    private void AddStitch(int from, int to, float distance)
    {
        _adjacency[from].Add(new OfflineBaiLink(from, to, Math.Max(distance, 0.01f), 0d, 0d, 0d, 0d, true));
        StitchLinkCount++;
    }

    private double Heuristic(int from, int to) => Vector3.Distance(_nodes[from].Position, _nodes[to].Position);

    private static bool NearTileBoundary(float value, float distance)
    {
        var remainder = MathF.Abs(value / NavigationCoordinates.PathTileSize -
                                  MathF.Round(value / NavigationCoordinates.PathTileSize)) *
                        NavigationCoordinates.PathTileSize;
        return remainder <= distance;
    }

    private static (int X, int Y, int Z) Quantize(Vector3 point, float tolerance) =>
        ((int)MathF.Floor(point.X / tolerance),
         (int)MathF.Floor(point.Y / tolerance),
         (int)MathF.Floor(point.Z / tolerance));

    private static bool IsFinite(Vector3 point) =>
        float.IsFinite(point.X) && float.IsFinite(point.Y) && float.IsFinite(point.Z);
}

public static class OfflineRouteGeometry
{
    public static IReadOnlyList<Vector3> Simplify(IReadOnlyList<Vector3> points, float tolerance)
    {
        if (points == null)
            throw new ArgumentNullException(nameof(points));
        if (!float.IsFinite(tolerance) || tolerance < 0f)
            throw new ArgumentOutOfRangeException(nameof(tolerance));
        if (points.Count <= 2)
            return points.ToArray();

        var keep = new bool[points.Count];
        keep[0] = keep[^1] = true;
        Simplify(points, 0, points.Count - 1, tolerance * tolerance, keep);
        return points.Where((_, index) => keep[index]).ToArray();
    }

    public static double Length(IReadOnlyList<Vector3> points)
    {
        var length = 0d;
        for (var index = 1; index < points.Count; index++)
            length += Vector3.Distance(points[index - 1], points[index]);
        return length;
    }

    public static RouteDeviation MeasureDeviation(
        IReadOnlyList<Vector3> samples,
        IReadOnlyList<Vector3> route)
    {
        if (samples is not { Count: > 0 } || route is not { Count: >= 2 })
            throw new ArgumentException("Deviation requires samples and a route.");
        var distances = samples.Select(sample => DistanceToPolyline(sample, route)).Order().ToArray();
        return new RouteDeviation(
            distances.Average(),
            Percentile(distances, 0.5d),
            Percentile(distances, 0.95d),
            distances[^1]);
    }

    public static RouteDeviation MeasureDeviation(
        IReadOnlyList<Vector3> samples,
        IReadOnlyList<IReadOnlyList<Vector3>> routes)
    {
        if (samples is not { Count: > 0 } || routes is not { Count: > 0 } ||
            routes.Any(route => route is not { Count: >= 2 }))
        {
            throw new ArgumentException("Deviation requires samples and one or more complete route segments.");
        }

        var distances = samples.Select(sample => routes.Min(route => DistanceToPolyline(sample, route)))
            .Order()
            .ToArray();
        return new RouteDeviation(
            distances.Average(),
            Percentile(distances, 0.5d),
            Percentile(distances, 0.95d),
            distances[^1]);
    }

    private static void Simplify(
        IReadOnlyList<Vector3> points,
        int first,
        int last,
        float toleranceSquared,
        bool[] keep)
    {
        var best = toleranceSquared;
        var bestIndex = -1;
        for (var index = first + 1; index < last; index++)
        {
            var distance = DistanceToSegmentSquared(points[index], points[first], points[last]);
            if (distance <= best)
                continue;
            best = distance;
            bestIndex = index;
        }
        if (bestIndex < 0)
            return;
        keep[bestIndex] = true;
        Simplify(points, first, bestIndex, toleranceSquared, keep);
        Simplify(points, bestIndex, last, toleranceSquared, keep);
    }

    private static double DistanceToPolyline(Vector3 point, IReadOnlyList<Vector3> route)
    {
        var best = double.PositiveInfinity;
        for (var index = 1; index < route.Count; index++)
            best = Math.Min(best, Math.Sqrt(DistanceToSegmentSquared(point, route[index - 1], route[index])));
        return best;
    }

    private static float DistanceToSegmentSquared(Vector3 point, Vector3 start, Vector3 end)
    {
        var target = new Vector2(point.X, point.Y);
        var a = new Vector2(start.X, start.Y);
        var delta = new Vector2(end.X - start.X, end.Y - start.Y);
        var lengthSquared = delta.LengthSquared();
        if (lengthSquared <= float.Epsilon)
            return Vector2.DistanceSquared(target, a);
        var projection = Math.Clamp(Vector2.Dot(target - a, delta) / lengthSquared, 0f, 1f);
        return Vector2.DistanceSquared(target, a + projection * delta);
    }

    private static double Percentile(IReadOnlyList<double> sorted, double percentile) =>
        sorted[(int)Math.Floor((sorted.Count - 1) * percentile)];
}

public readonly record struct RouteDeviation(double Mean, double Median, double P95, double Maximum);

using System.Diagnostics;
using System.Globalization;
using System.Numerics;
using System.Security.Cryptography;
using System.Text.Json;
using System.Text.RegularExpressions;

using AAEmu.Game.Bots.Navigation;
using AAEmu.Game.Bots.Navigation.Offline;
using AAEmu.Game.IO;
using AAEmu.Game.Models.CryEngine.Objects;
using AAEmu.Game.Models.CryEngine.Readers;

return NavigationExtractor.Run(args);

internal static class NavigationExtractor
{
    private static readonly JsonSerializerOptions OutputJson = new()
    {
        WriteIndented = true,
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase
    };

    public static int Run(string[] args)
    {
        if (args.Length == 0 || args is ["--help"] or ["-h"])
        {
            Console.WriteLine(ExtractorOptions.Usage);
            return 0;
        }
        try
        {
            var options = ExtractorOptions.Parse(args);
            if (File.Exists(options.Output))
                throw new InvalidOperationException($"Output already exists: {options.Output}");

            var externalRoadPrior = options.RoadPrior == null
                ? null
                : ReadRecordedRoute(options.RoadPrior, options.WorldId, options.Bounds, "road prior");
            var trace = options.Trace == null
                ? null
                : ReadRecordedRoute(options.Trace, options.WorldId, options.Bounds, "validation trace");

            var total = Stopwatch.StartNew();
            if (!ClientFileManager.AddSource(options.GamePak))
                throw new InvalidOperationException("The ArcheAge game_pak could not be opened.");

            try
            {
                var source = ClientFileManager.Sources.Single().GamePak;
                var pathFiles = IndexPathFiles(source.pakFiles.Keys, options);
                var graph = new OfflineBaiGraph();
                var priors = new RoadPriorIndex(options.RoadInfluence);
                var water = new WaterTraversalIndex();
                var inventory = new SourceInventory();
                var load = Stopwatch.StartNew();

                foreach (var tile in pathFiles.OrderBy(pair => pair.Key.X).ThenBy(pair => pair.Key.Y))
                    ReadPathTile(source, tile.Key.X, tile.Key.Y, tile.Value, graph, priors, inventory);

                ReadWorldObjects(options, priors, water, inventory);
                if (externalRoadPrior != null)
                {
                    foreach (var segment in externalRoadPrior.Segments)
                        priors.Add(segment);
                    inventory.ExternalRoadPriorSegments = externalRoadPrior.Segments.Count;
                    inventory.ExternalRoadPriorPoints = externalRoadPrior.PointCount;
                }
                graph.StitchTileBoundaries();
                load.Stop();

                if (inventory.DecodeFailures > 0)
                {
                    throw new InvalidDataException(
                        $"Navigation source decoding failed {inventory.DecodeFailures} time(s); " +
                        $"no output was written. First failure: {inventory.Failures[0]}");
                }

                if (graph.Nodes.Count == 0)
                    throw new InvalidOperationException("No mission BAI nodes were decoded inside the requested bounds.");

                var startNode = graph.FindNearest(options.Start);
                var endNode = graph.FindNearest(options.End);
                var startSnap = Vector3.Distance(options.Start, graph.Nodes[startNode].Position);
                var endSnap = Vector3.Distance(options.End, graph.Nodes[endNode].Position);
                if (startSnap > options.MaximumSnap || endSnap > options.MaximumSnap)
                {
                    throw new InvalidOperationException(
                        $"BAI endpoint snap exceeded {options.MaximumSnap:R} m " +
                        $"(start={startSnap:R}, end={endSnap:R}).");
                }

                var query = Stopwatch.StartNew();
                var route = graph.FindRoute(startNode, endNode, priors, options.OffRoadPenalty, water);
                query.Stop();
                if (!route.IsSuccess)
                    throw new InvalidOperationException($"No directed BAI route was found: {route.Failure}.");

                var simplified = OfflineRouteGeometry.Simplify(route.Points, options.SimplificationTolerance);
                var grades = route.Points.Zip(route.Points.Skip(1), (from, to) =>
                {
                    var planar = Vector2.Distance(new Vector2(from.X, from.Y), new Vector2(to.X, to.Y));
                    return Math.Abs(to.Z - from.Z) / Math.Max(planar, 0.25f);
                }).ToArray();
                var roadDistances = priors.SegmentCount == 0
                    ? []
                    : route.Points.Select(point => (double)priors.Distance(point)).Order().ToArray();
                double? meanRoadDistance = roadDistances.Length == 0 ? null : roadDistances.Average();
                double? p95RoadDistance = roadDistances.Length == 0
                    ? null
                    : roadDistances[(int)Math.Floor((roadDistances.Length - 1) * 0.95d)];
                var tracePoints = trace?.Segments.SelectMany(segment => segment).ToArray() ?? [];
                RouteDeviation? traceToRoute = trace == null
                    ? null
                    : OfflineRouteGeometry.MeasureDeviation(tracePoints, simplified);
                RouteDeviation? routeToTrace = trace == null
                    ? null
                    : OfflineRouteGeometry.MeasureDeviation(simplified, trace.Segments);
                total.Stop();

                var payload = new
                {
                    schemaVersion = 1,
                    build = new
                    {
                        source = "AAEmu 1.2 mission BAI",
                        world = options.World,
                        bounds = options.Bounds,
                        coordinateTransform = "world = local + (pathTile * 256)",
                        recordedRouteInputs = new
                        {
                            roadPrior = externalRoadPrior?.Provenance,
                            validationTrace = trace?.Provenance
                        },
                        inventory = new
                        {
                            pathTiles = pathFiles.Count,
                            inventory.NetFiles,
                            inventory.RoadFiles,
                            inventory.AreaFiles,
                            inventory.ObjectFiles,
                            inventory.ObjectRoads,
                            inventory.ObjectRoadPoints,
                            inventory.ObjectRoadPointsInsideBounds,
                            inventory.ObjectRoadPointsAlreadyWorldSpace,
                            objectRoadPriorEnabled = options.UseObjectRoadPrior,
                            inventory.ExternalRoadPriorSegments,
                            inventory.ExternalRoadPriorPoints,
                            inventory.WaterBodies,
                            waterPenaltyShapes = water.ShapeCount,
                            inventory.ForbiddenAreas,
                            inventory.NavigationModifiers,
                            inventory.NonZeroPassRadiusLinks,
                            inventory.NonZeroWaterDepthValues,
                            inventory.PassRadiusMinimum,
                            inventory.PassRadiusMaximum,
                            inventory.WaterDepthMinimum,
                            inventory.WaterDepthMaximum,
                            inventory.DecodeFailures,
                            graphNodes = graph.Nodes.Count,
                            directedSourceLinks = graph.SourceLinkCount,
                            directedStitchLinks = graph.StitchLinkCount,
                            roadPriorSegments = priors.SegmentCount
                        },
                        timingMs = new
                        {
                            preprocessing = Math.Round(load.Elapsed.TotalMilliseconds, 3),
                            routeQuery = Math.Round(query.Elapsed.TotalMilliseconds, 3),
                            total = Math.Round(total.Elapsed.TotalMilliseconds, 3)
                        },
                        route = new
                        {
                            startSnapMeters = Math.Round(startSnap, 3),
                            endSnapMeters = Math.Round(endSnap, 3),
                            visitedNodes = route.VisitedNodes,
                            rawPoints = route.Points.Count,
                            simplifiedPoints = simplified.Count,
                            lengthMeters = Math.Round(OfflineRouteGeometry.Length(route.Points), 3),
                            directMeters = Math.Round(Vector3.Distance(options.Start, options.End), 3),
                            route.Cost,
                            maximumGrade = Math.Round(grades.DefaultIfEmpty(0f).Max(), 3),
                            waterRoutePoints = route.Points.Count(water.Contains),
                            meanRoadDistanceMeters = meanRoadDistance.HasValue
                                ? Math.Round(meanRoadDistance.Value, 3)
                                : (double?)null,
                            p95RoadDistanceMeters = p95RoadDistance.HasValue
                                ? Math.Round(p95RoadDistance.Value, 3)
                                : (double?)null,
                            direction = "source-to-target links preserved",
                            traceSegments = trace?.Segments.Count ?? 0,
                            tracePoints = tracePoints.Length,
                            traceToRoute,
                            routeToTrace
                        },
                        failures = inventory.Failures
                    },
                    routes = new[]
                    {
                        new
                        {
                            worldId = options.WorldId,
                            zoneId = options.ZoneId,
                            pathName = options.RouteId,
                            pathType = 100,
                            cellX = -1,
                            cellY = -1,
                            direction = RoadTravelDirection.ForwardOnly.ToString(),
                            points = simplified.Select(point => new
                            {
                                x = Math.Round(point.X, 3),
                                y = Math.Round(point.Y, 3),
                                z = Math.Round(point.Z, 3),
                                surfaceId = 0
                            })
                        }
                    }
                };

                var outputDirectory = Path.GetDirectoryName(options.Output);
                if (!string.IsNullOrEmpty(outputDirectory))
                    Directory.CreateDirectory(outputDirectory);
                var outputJson = JsonSerializer.Serialize(payload, OutputJson) + "\n";
                using (var stream = new FileStream(options.Output, FileMode.CreateNew, FileAccess.Write, FileShare.None))
                using (var writer = new StreamWriter(stream, new System.Text.UTF8Encoding(false)))
                    writer.Write(outputJson);
                Console.WriteLine(JsonSerializer.Serialize(payload.build, OutputJson));
                Console.WriteLine($"Wrote {options.Output}");
                return 0;
            }
            finally
            {
                ClientFileManager.ClearSources();
            }
        }
        catch (Exception exception)
        {
            Console.Error.WriteLine(exception.Message);
            Console.Error.WriteLine(ExtractorOptions.Usage);
            return 2;
        }
    }

    private static Dictionary<(int X, int Y), List<string>> IndexPathFiles(
        IEnumerable<string> names,
        ExtractorOptions options)
    {
        var result = new Dictionary<(int X, int Y), List<string>>();
        var expression = new Regex(
            $"^game/worlds/{Regex.Escape(options.World)}/paths/(?<x>\\d+)_(?<y>\\d+)/(?<file>[^/]+\\.bai)$",
            RegexOptions.IgnoreCase | RegexOptions.CultureInvariant);
        var minimumX = NavigationCoordinates.PathTile(options.Bounds.MinimumX);
        var minimumY = NavigationCoordinates.PathTile(options.Bounds.MinimumY);
        var maximumX = NavigationCoordinates.PathTile(options.Bounds.MaximumX);
        var maximumY = NavigationCoordinates.PathTile(options.Bounds.MaximumY);

        foreach (var rawName in names)
        {
            var name = rawName.Replace('\\', '/');
            var match = expression.Match(name);
            if (!match.Success ||
                !int.TryParse(match.Groups["x"].Value, out var x) ||
                !int.TryParse(match.Groups["y"].Value, out var y) ||
                x < minimumX || x > maximumX || y < minimumY || y > maximumY)
            {
                continue;
            }

            if (!result.TryGetValue((x, y), out var files))
                result[(x, y)] = files = [];
            files.Add(rawName);
        }
        return result;
    }

    private static void ReadPathTile(
        AAEmu.Commons.Utils.AAPak.AAPak source,
        int tileX,
        int tileY,
        IEnumerable<string> files,
        OfflineBaiGraph graph,
        RoadPriorIndex priors,
        SourceInventory inventory)
    {
        var offset = new Vector3(tileX * NavigationCoordinates.PathTileSize,
            tileY * NavigationCoordinates.PathTileSize, 0f);
        foreach (var file in files.Order(StringComparer.OrdinalIgnoreCase))
        {
            var name = Path.GetFileName(file);
            try
            {
                using var stream = source.ExportFileAsStreamCloned(file);
                if (name.StartsWith("netmission", StringComparison.OrdinalIgnoreCase))
                {
                    var reader = new NetMissionReader(stream, 0) { ReaderPointOffset = offset };
                    reader.ReadFile();
                    var links = reader.LinkDescriptorList.Select(link => new OfflineBaiSourceLink(
                        link.SourceNode,
                        link.TargetNode,
                        link.Length,
                        link.MaxPassRadius,
                        link.Exposure,
                        link.MinWaterDepth,
                        link.MaxWaterDepth)).ToArray();
                    foreach (var link in links)
                        inventory.RecordLink(link);
                    graph.AddTile(
                        tileX,
                        tileY,
                        reader.NodeDescriptorList.Values.Select(node =>
                            new OfflineBaiSourceNode(node.Id, node.Pos, node.Dir)),
                        links);
                    inventory.NetFiles++;
                }
                else if (name.Contains("road", StringComparison.OrdinalIgnoreCase))
                {
                    var reader = new RoadMissionReader(stream, 0) { ReaderPointOffset = offset };
                    reader.ReadFile();
                    foreach (var road in reader.RoadList)
                        priors.Add(road.RoadNodeList.Select(node => node.Pos));
                    inventory.RoadFiles++;
                }
                else if (name.StartsWith("areasmission", StringComparison.OrdinalIgnoreCase))
                {
                    var previous = BaiReader.IgnoreDuplicateAreaNames;
                    BaiReader.IgnoreDuplicateAreaNames = false;
                    try
                    {
                        var reader = new AreasMissionReader(stream, 0) { ReaderPointOffset = offset };
                        reader.ReadFile();
                        inventory.ForbiddenAreas += reader.ForbiddenAreasList.Count +
                                                    reader.DesignerForbiddenAreasList.Count +
                                                    reader.ForbiddenBoundariesList.Count;
                        inventory.NavigationModifiers += reader.NavigationModifiers.Count;
                        inventory.AreaFiles++;
                    }
                    finally
                    {
                        BaiReader.IgnoreDuplicateAreaNames = previous;
                    }
                }
            }
            catch (Exception exception)
            {
                inventory.RecordFailure(file, exception.Message);
            }
        }
    }

    private static void ReadWorldObjects(
        ExtractorOptions options,
        RoadPriorIndex priors,
        WaterTraversalIndex waterIndex,
        SourceInventory inventory)
    {
        var minimumX = (int)MathF.Floor(options.Bounds.MinimumX / NavigationCoordinates.WorldCellSize);
        var minimumY = (int)MathF.Floor(options.Bounds.MinimumY / NavigationCoordinates.WorldCellSize);
        var maximumX = (int)MathF.Floor(options.Bounds.MaximumX / NavigationCoordinates.WorldCellSize);
        var maximumY = (int)MathF.Floor(options.Bounds.MaximumY / NavigationCoordinates.WorldCellSize);
        for (var cellX = minimumX; cellX <= maximumX; cellX++)
        for (var cellY = minimumY; cellY <= maximumY; cellY++)
        {
            var file = $"game/worlds/{options.World}/cells/{cellX:000}_{cellY:000}/client/object.dat";
            if (!ClientFileManager.FileExists(file))
                continue;
            try
            {
                var objects = new ObjectsFile(file);
                if (!objects.ReadFile())
                    inventory.RecordFailure(file, "object.dat was only partially decoded");
                inventory.ObjectFiles++;
                foreach (var value in objects.PrefabsList)
                {
                    switch (value)
                    {
                        case ObjectDataType13Road road:
                            var points = road.PointsList.Select(point =>
                            {
                                var world = ObjectPointToWorld(cellX, cellY, point);
                                inventory.ObjectRoadPoints++;
                                if (world.X >= options.Bounds.MinimumX && world.X <= options.Bounds.MaximumX &&
                                    world.Y >= options.Bounds.MinimumY && world.Y <= options.Bounds.MaximumY)
                                {
                                    inventory.ObjectRoadPointsInsideBounds++;
                                }
                                if (world == point)
                                    inventory.ObjectRoadPointsAlreadyWorldSpace++;
                                return world;
                            }).ToArray();
                            if (options.UseObjectRoadPrior)
                                priors.Add(points);
                            inventory.ObjectRoads++;
                            break;
                        case ObjectDataType11Water water:
                            waterIndex.AddPolygon(
                                water.PhysicsContourPointsList.Select(point =>
                                    ObjectPointToWorld(cellX, cellY, point) with { Z = water.SurfaceHeight }),
                                water.SurfaceHeight);
                            if (water.VolumeType == WaterObjectVolumeType.River)
                            {
                                var centerline = water.ShapePointsList.Count >= 2
                                    ? water.ShapePointsList
                                    : new List<Vector3> { water.StartPos, water.EndPos };
                                waterIndex.AddCorridor(
                                    centerline.Select(point =>
                                        ObjectPointToWorld(cellX, cellY, point) with { Z = water.SurfaceHeight }),
                                    Math.Clamp(water.Depth * 2f, 4f, 40f),
                                    water.SurfaceHeight);
                            }
                            inventory.WaterBodies++;
                            break;
                    }
                }
            }
            catch (Exception exception)
            {
                inventory.RecordFailure(file, exception.Message);
            }
        }
    }

    private static Vector3 ObjectPointToWorld(int cellX, int cellY, Vector3 point)
    {
        var local = point.X is >= -512f and <= NavigationCoordinates.WorldCellSize * 2f &&
                    point.Y is >= -512f and <= NavigationCoordinates.WorldCellSize * 2f;
        return local
            ? point + new Vector3(cellX * NavigationCoordinates.WorldCellSize,
                cellY * NavigationCoordinates.WorldCellSize, 0f)
            : point;
    }

    private static RecordedRouteInput ReadRecordedRoute(
        string path,
        uint expectedWorldId,
        WorldBounds extractionBounds,
        string role)
    {
        var bytes = File.ReadAllBytes(path);
        using var document = JsonDocument.Parse(bytes);
        var root = document.RootElement;
        if (root.ValueKind != JsonValueKind.Object)
            throw new InvalidDataException($"The {role} JSON root must be an object.");

        var segments = new List<Vector3[]>();
        var declaredWorldIds = new HashSet<uint>();
        var declaredDirections = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        if (root.TryGetProperty("points", out var points))
        {
            RecordMetadata(root, expectedWorldId, declaredWorldIds, declaredDirections, role, "root");
            segments.Add(ReadPoints(points, role, "points"));
        }
        else if (TryGetProperty(root, "edges", "routes", out var routeSegments) &&
                 routeSegments.ValueKind == JsonValueKind.Array)
        {
            var index = 0;
            foreach (var segment in routeSegments.EnumerateArray())
            {
                if (segment.ValueKind != JsonValueKind.Object ||
                    !segment.TryGetProperty("points", out points))
                {
                    throw new InvalidDataException($"The {role} segment {index} must contain points.");
                }

                RecordMetadata(segment, expectedWorldId, declaredWorldIds, declaredDirections, role, $"segment {index}");
                segments.Add(ReadPoints(points, role, $"segment {index}.points"));
                index++;
            }
        }
        else
        {
            throw new InvalidDataException($"The {role} JSON must contain points, edges[].points, or routes[].points.");
        }

        if (segments.Count == 0)
            throw new InvalidDataException($"The {role} contains no route segments.");
        if (segments.Sum(segment => segment.Length) > 1_000_000)
            throw new InvalidDataException($"The {role} exceeds the 1,000,000-point safety limit.");

        var allPoints = segments.SelectMany(segment => segment).ToArray();
        var pointsInsideBounds = allPoints.Count(point =>
            point.X >= extractionBounds.MinimumX && point.X <= extractionBounds.MaximumX &&
            point.Y >= extractionBounds.MinimumY && point.Y <= extractionBounds.MaximumY);
        if (pointsInsideBounds == 0)
        {
            throw new InvalidDataException(
                $"The {role} has no points inside the requested extraction bounds; check its coordinate registration.");
        }

        var mapRegistrationValidated = ValidateMapRegistration(root, allPoints, role);
        var schemaVersion = ReadString(root, "schema_version", "schemaVersion");
        var routeId = ReadString(root, "route_id", "routeId");
        string? sourceKind = null;
        if (root.TryGetProperty("source", out var source) && source.ValueKind == JsonValueKind.Object)
        {
            sourceKind = ReadString(source, "kind");
            var sourceDirection = ReadString(source, "direction");
            if (!string.IsNullOrWhiteSpace(sourceDirection))
                declaredDirections.Add(sourceDirection);
        }

        var provenance = new RecordedRouteProvenance(
            role,
            Path.GetFileName(path),
            Convert.ToHexString(SHA256.HashData(bytes)).ToLowerInvariant(),
            schemaVersion,
            routeId,
            sourceKind,
            segments.Count,
            allPoints.Length,
            pointsInsideBounds,
            declaredWorldIds.Order().ToArray(),
            declaredDirections.Order(StringComparer.OrdinalIgnoreCase).ToArray(),
            mapRegistrationValidated,
            role == "road prior"
                ? "symmetric distance-to-centerline cost prior; no graph links or direction inferred"
                : "planar deviation reference; discontinuity-separated segments remain separate");
        return new RecordedRouteInput(segments.Cast<IReadOnlyList<Vector3>>().ToArray(), provenance);
    }

    private static Vector3[] ReadPoints(JsonElement points, string role, string location)
    {
        if (points.ValueKind != JsonValueKind.Array)
            throw new InvalidDataException($"The {role} {location} value must be an array.");

        var result = points.EnumerateArray().Select((point, index) =>
        {
            if (point.ValueKind != JsonValueKind.Object)
                throw new InvalidDataException($"The {role} {location}[{index}] must be an object.");
            var value = new Vector3(
                point.GetProperty("x").GetSingle(),
                point.GetProperty("y").GetSingle(),
                point.GetProperty("z").GetSingle());
            if (!float.IsFinite(value.X) || !float.IsFinite(value.Y) || !float.IsFinite(value.Z))
                throw new InvalidDataException($"The {role} {location}[{index}] has non-finite coordinates.");
            return value;
        }).ToArray();

        if (result.Length < 2)
            throw new InvalidDataException($"The {role} {location} requires at least two points.");
        return result;
    }

    private static void RecordMetadata(
        JsonElement value,
        uint expectedWorldId,
        ISet<uint> declaredWorldIds,
        ISet<string> declaredDirections,
        string role,
        string location)
    {
        if (TryGetProperty(value, "world_id", "worldId", out var world))
        {
            if (world.ValueKind != JsonValueKind.Number)
                throw new InvalidDataException($"The {role} {location} world id must be an unsigned integer.");
            var worldId = world.GetUInt32();
            if (worldId != expectedWorldId)
                throw new InvalidDataException($"The {role} {location} declares world {worldId}, expected {expectedWorldId}.");
            declaredWorldIds.Add(worldId);
        }

        if (value.TryGetProperty("direction", out var direction))
        {
            if (direction.ValueKind != JsonValueKind.String)
                throw new InvalidDataException($"The {role} {location} direction must be a string.");
            var directionText = direction.GetString();
            if (!string.IsNullOrWhiteSpace(directionText))
                declaredDirections.Add(directionText);
        }
    }

    private static bool ValidateMapRegistration(JsonElement root, IReadOnlyList<Vector3> points, string role)
    {
        if (!TryGetProperty(root, "map_registration", "mapRegistration", out var registration))
            return false;
        if (registration.ValueKind != JsonValueKind.Object)
            throw new InvalidDataException($"The {role} map registration must be an object.");

        var x = registration.GetProperty("x").GetSingle();
        var y = registration.GetProperty("y").GetSingle();
        var width = registration.GetProperty("width").GetSingle();
        var height = registration.GetProperty("height").GetSingle();
        if (!float.IsFinite(x) || !float.IsFinite(y) || !float.IsFinite(width) || !float.IsFinite(height) ||
            width <= 0f || height <= 0f)
        {
            throw new InvalidDataException($"The {role} map registration is invalid.");
        }

        if (points.Any(point => point.X < x || point.X > x + width || point.Y < y || point.Y > y + height))
            throw new InvalidDataException($"The {role} contains points outside its declared map registration.");
        return true;
    }

    private static bool TryGetProperty(JsonElement value, string first, string second, out JsonElement result) =>
        value.TryGetProperty(first, out result) || value.TryGetProperty(second, out result);

    private static string? ReadString(JsonElement value, string name, string? alternate = null)
    {
        if (value.TryGetProperty(name, out var property) ||
            (alternate != null && value.TryGetProperty(alternate, out property)))
        {
            return property.ValueKind == JsonValueKind.String ? property.GetString() : null;
        }
        return null;
    }

    private sealed record RecordedRouteInput(
        IReadOnlyList<IReadOnlyList<Vector3>> Segments,
        RecordedRouteProvenance Provenance)
    {
        public int PointCount => Segments.Sum(segment => segment.Count);
    }

    private sealed record RecordedRouteProvenance(
        string Role,
        string File,
        string Sha256,
        string? SchemaVersion,
        string? RouteId,
        string? SourceKind,
        int Segments,
        int Points,
        int PointsInsideExtractionBounds,
        uint[] DeclaredWorldIds,
        string[] DeclaredDirections,
        bool MapRegistrationValidated,
        string Interpretation);

    private sealed class SourceInventory
    {
        public int NetFiles { get; set; }
        public int RoadFiles { get; set; }
        public int AreaFiles { get; set; }
        public int ObjectFiles { get; set; }
        public int ObjectRoads { get; set; }
        public int ObjectRoadPoints { get; set; }
        public int ObjectRoadPointsInsideBounds { get; set; }
        public int ObjectRoadPointsAlreadyWorldSpace { get; set; }
        public int ExternalRoadPriorSegments { get; set; }
        public int ExternalRoadPriorPoints { get; set; }
        public int WaterBodies { get; set; }
        public int ForbiddenAreas { get; set; }
        public int NavigationModifiers { get; set; }
        public long NonZeroPassRadiusLinks { get; private set; }
        public long NonZeroWaterDepthValues { get; private set; }
        public double? PassRadiusMinimum { get; private set; }
        public double? PassRadiusMaximum { get; private set; }
        public double? WaterDepthMinimum { get; private set; }
        public double? WaterDepthMaximum { get; private set; }
        public int DecodeFailures { get; private set; }
        public List<string> Failures { get; } = [];

        public void RecordFailure(string file, string detail)
        {
            DecodeFailures++;
            if (Failures.Count < 20)
                Failures.Add($"{file.Replace('\\', '/')}: {detail}");
        }

        public void RecordLink(OfflineBaiSourceLink link)
        {
            if (Math.Abs(link.MaxPassRadius) > 0.0001d)
            {
                NonZeroPassRadiusLinks++;
                PassRadiusMinimum = Math.Min(PassRadiusMinimum ?? link.MaxPassRadius, link.MaxPassRadius);
                PassRadiusMaximum = Math.Max(PassRadiusMaximum ?? link.MaxPassRadius, link.MaxPassRadius);
            }

            foreach (var depth in new[] { link.MinWaterDepth, link.MaxWaterDepth })
            {
                if (Math.Abs(depth) <= 0.0001d)
                    continue;
                NonZeroWaterDepthValues++;
                WaterDepthMinimum = Math.Min(WaterDepthMinimum ?? depth, depth);
                WaterDepthMaximum = Math.Max(WaterDepthMaximum ?? depth, depth);
            }
        }
    }
}

internal readonly record struct WorldBounds(float MinimumX, float MinimumY, float MaximumX, float MaximumY);

internal sealed record ExtractorOptions(
    string GamePak,
    string Output,
    string World,
    uint WorldId,
    uint ZoneId,
    string RouteId,
    WorldBounds Bounds,
    Vector3 Start,
    Vector3 End,
    string? Trace,
    string? RoadPrior,
    bool UseObjectRoadPrior,
    float RoadInfluence,
    double OffRoadPenalty,
    float SimplificationTolerance,
    float MaximumSnap)
{
    public const string Usage =
        "Usage: --game-pak <file> --output <json> --bounds <minX,minY,maxX,maxY> " +
        "--start <x,y,z> --end <x,y,z> --route-id <id> [--world main_world] " +
        "[--world-id 1] [--zone-id 0] [--trace <json>] [--road-prior <json>] " +
        "[--object-road-prior false] [--road-influence 24] [--off-road-penalty 5] " +
        "[--simplify 2] [--max-snap 8]";

    public static ExtractorOptions Parse(string[] args)
    {
        var values = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        for (var index = 0; index < args.Length; index += 2)
        {
            if (index + 1 >= args.Length || !args[index].StartsWith("--", StringComparison.Ordinal))
                throw new ArgumentException("Arguments must be --name value pairs.");
            values[args[index][2..]] = args[index + 1];
        }

        string Required(string name) => values.TryGetValue(name, out var value) && !string.IsNullOrWhiteSpace(value)
            ? value
            : throw new ArgumentException($"Missing --{name}.");
        string Optional(string name, string fallback) => values.GetValueOrDefault(name, fallback);
        float Float(string name, float fallback) => float.Parse(Optional(name, fallback.ToString("R", CultureInfo.InvariantCulture)), CultureInfo.InvariantCulture);
        bool Bool(string name, bool fallback) => bool.Parse(Optional(name, fallback.ToString(CultureInfo.InvariantCulture)));

        var routeId = Required("route-id");
        if (!Regex.IsMatch(routeId, "^[A-Za-z0-9._/-]+$", RegexOptions.CultureInvariant))
            throw new ArgumentException("--route-id contains unsupported characters.");
        var trace = values.GetValueOrDefault("trace");
        var roadPrior = values.GetValueOrDefault("road-prior");
        return new ExtractorOptions(
            Path.GetFullPath(Required("game-pak")),
            Path.GetFullPath(Required("output")),
            Optional("world", "main_world"),
            uint.Parse(Optional("world-id", "1"), CultureInfo.InvariantCulture),
            uint.Parse(Optional("zone-id", "0"), CultureInfo.InvariantCulture),
            routeId,
            ParseBounds(Required("bounds")),
            ParseVector(Required("start")),
            ParseVector(Required("end")),
            string.IsNullOrWhiteSpace(trace) ? null : Path.GetFullPath(trace),
            string.IsNullOrWhiteSpace(roadPrior) ? null : Path.GetFullPath(roadPrior),
            Bool("object-road-prior", false),
            Float("road-influence", 24f),
            double.Parse(Optional("off-road-penalty", "5"), CultureInfo.InvariantCulture),
            Float("simplify", 2f),
            Float("max-snap", 8f));
    }

    private static WorldBounds ParseBounds(string value)
    {
        var parts = Parse(value, 4);
        if (parts[2] <= parts[0] || parts[3] <= parts[1])
            throw new ArgumentException("--bounds maximums must exceed minimums.");
        return new WorldBounds(parts[0], parts[1], parts[2], parts[3]);
    }

    private static Vector3 ParseVector(string value)
    {
        var parts = Parse(value, 3);
        return new Vector3(parts[0], parts[1], parts[2]);
    }

    private static float[] Parse(string value, int count)
    {
        var parts = value.Split(',');
        if (parts.Length != count)
            throw new ArgumentException($"Expected {count} comma-separated numbers, got '{value}'.");
        return parts.Select(part => float.Parse(part, CultureInfo.InvariantCulture)).ToArray();
    }
}

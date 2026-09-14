using System.Numerics;
using System.Security.Cryptography;
using System.Text.Json;
using System.Text.Json.Serialization;

using NLog;

namespace AAEmu.Game.Bots.Navigation;

public sealed class PrebuiltRoadDocument
{
    public int SchemaVersion { get; set; }
    public List<PrebuiltRoadRoute> Routes { get; set; } = [];
}

public sealed class PrebuiltRoadRoute
{
    public uint WorldId { get; set; }
    public uint ZoneId { get; set; }
    public string PathName { get; set; } = string.Empty;
    public int PathType { get; set; }
    public int CellX { get; set; } = -1;
    public int CellY { get; set; } = -1;
    public RoadTravelDirection Direction { get; set; } = RoadTravelDirection.ForwardOnly;
    public List<PrebuiltRoadPoint> Points { get; set; } = [];
}

public sealed class PrebuiltRoadPoint
{
    public float X { get; set; }
    public float Y { get; set; }
    public float Z { get; set; }
    public int SurfaceId { get; set; }
}

public static class PrebuiltRoadSnapshotReader
{
    public const int MaximumRoutes = 10_000;
    public const int MaximumPoints = 2_000_000;

    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNameCaseInsensitive = true,
        Converters = { new JsonStringEnumConverter() }
    };

    public static TransferRoadNetworkSnapshot Read(string json, long revision = 1)
    {
        if (string.IsNullOrWhiteSpace(json))
            throw new InvalidDataException("The prebuilt road document is empty.");

        var document = JsonSerializer.Deserialize<PrebuiltRoadDocument>(json, JsonOptions) ??
                       throw new InvalidDataException("The prebuilt road document could not be decoded.");
        if (document.SchemaVersion != 1)
            throw new InvalidDataException($"Unsupported prebuilt road schema {document.SchemaVersion}.");

        if (document.Routes == null)
            throw new InvalidDataException("The prebuilt road document has no routes collection.");
        if (document.Routes.Count > MaximumRoutes)
            throw new InvalidDataException($"The prebuilt road document exceeds {MaximumRoutes} routes.");

        var routes = new List<RoadPolylineSnapshot>(document.Routes.Count);
        var totalPoints = 0;
        for (var index = 0; index < document.Routes.Count; index++)
        {
            var route = document.Routes[index] ??
                        throw new InvalidDataException($"Route {index} is null.");
            if (string.IsNullOrWhiteSpace(route.PathName))
                throw new InvalidDataException($"Route {index} has no path name.");
            if (!Enum.IsDefined(route.Direction))
                throw new InvalidDataException($"Route {index} has an invalid direction.");
            if (route.Points is not { Count: >= 2 })
                throw new InvalidDataException($"Route {index} requires at least two points.");
            totalPoints = checked(totalPoints + route.Points.Count);
            if (totalPoints > MaximumPoints)
                throw new InvalidDataException($"The prebuilt road document exceeds {MaximumPoints} points.");

            var points = route.Points.Select((point, pointIndex) =>
            {
                if (point == null || !float.IsFinite(point.X) || !float.IsFinite(point.Y) ||
                    !float.IsFinite(point.Z))
                {
                    throw new InvalidDataException($"Route {index} point {pointIndex} is invalid.");
                }

                return new RoadPoint(point.X, point.Y, point.Z, point.SurfaceId);
            }).ToArray();

            routes.Add(new RoadPolylineSnapshot(
                route.WorldId,
                route.ZoneId,
                route.PathName,
                route.PathType,
                route.CellX,
                route.CellY,
                route.Direction,
                points));
        }

        return new TransferRoadNetworkSnapshot(revision, routes);
    }

    public static long Revision(ReadOnlySpan<byte> bytes)
    {
        Span<byte> hash = stackalloc byte[32];
        SHA256.HashData(bytes, hash);
        return BitConverter.ToInt64(hash);
    }
}

/// <summary>Loads one immutable, optional local route file per server process.</summary>
public sealed class PrebuiltRoadSnapshotProvider : ITransferRoadSnapshotProvider
{
    private const long MaximumFileBytes = 32L * 1024L * 1024L;
    private static readonly Logger Logger = LogManager.GetCurrentClassLogger();
    private readonly string _path;
    private readonly Lazy<TransferRoadNetworkSnapshot> _snapshot;

    public PrebuiltRoadSnapshotProvider(string path)
    {
        _path = string.IsNullOrWhiteSpace(path)
            ? throw new ArgumentException("A route data path is required.", nameof(path))
            : Path.GetFullPath(path);
        _snapshot = new Lazy<TransferRoadNetworkSnapshot>(Load, LazyThreadSafetyMode.ExecutionAndPublication);
    }

    public TransferRoadNetworkSnapshot Capture() => _snapshot.Value;

    private TransferRoadNetworkSnapshot Load()
    {
        if (!File.Exists(_path))
            return new TransferRoadNetworkSnapshot(0, []);

        try
        {
            if (new FileInfo(_path).Length > MaximumFileBytes)
                throw new InvalidDataException($"Prebuilt road data exceeds {MaximumFileBytes} bytes.");
            var bytes = File.ReadAllBytes(_path);
            return PrebuiltRoadSnapshotReader.Read(
                System.Text.Encoding.UTF8.GetString(bytes),
                PrebuiltRoadSnapshotReader.Revision(bytes));
        }
        catch (Exception exception)
        {
            Logger.Warn(exception, "Ignoring invalid PlayerBots route data at {0}.", _path);
            return new TransferRoadNetworkSnapshot(0, []);
        }
    }
}

public sealed class CompositeRoadSnapshotProvider(params ITransferRoadSnapshotProvider[] providers)
    : ITransferRoadSnapshotProvider
{
    private readonly IReadOnlyList<ITransferRoadSnapshotProvider> _providers =
        providers?.Where(provider => provider != null).ToArray() ??
        throw new ArgumentNullException(nameof(providers));

    public TransferRoadNetworkSnapshot Capture()
    {
        var roads = new List<RoadPolylineSnapshot>();
        var revision = 1469598103934665603L;
        foreach (var provider in _providers)
        {
            var snapshot = provider.Capture() ??
                           throw new InvalidOperationException("A road provider returned no snapshot.");
            roads.AddRange(snapshot.Roads);
            revision = unchecked((revision ^ snapshot.Revision) * 1099511628211L);
        }

        return new TransferRoadNetworkSnapshot(revision, roads);
    }
}

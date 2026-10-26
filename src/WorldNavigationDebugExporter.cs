using System.Text.Json;
using System.Text.Json.Serialization;

namespace AAEmu.Game.Bots.Navigation;

/// <summary>
/// Produces a deterministic read-only JSON representation. Deliberately exposes no file
/// writer, so callers choose a server-side diagnostics destination and cannot mutate client data.
/// </summary>
public static class WorldNavigationDebugExporter
{
    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
        WriteIndented = true
    };

    public static string Serialize(WorldRoadGraph graph, RoadRouteResult? route = null)
    {
        ArgumentNullException.ThrowIfNull(graph);
        var document = new
        {
            SourceRevision = graph.SourceRevision,
            GenerationId = graph.GenerationId,
            ComponentCount = graph.ComponentCount,
            Nodes = graph.Nodes.OrderBy(node => node.Id).Select(node => new
            {
                Id = node.Id,
                WorldId = node.WorldId,
                ComponentId = node.ComponentId,
                SurfaceId = node.SurfaceId,
                Position = new DebugPoint(node.Position.X, node.Position.Y, node.Position.Z),
                Endpoints = node.Endpoints.Select(endpoint => new
                {
                    EdgeId = endpoint.EdgeId,
                    IsStart = endpoint.IsStart,
                    ZoneId = endpoint.ZoneId,
                    PathName = endpoint.PathName
                })
            }),
            Edges = graph.Edges.OrderBy(edge => edge.Id, StringComparer.Ordinal).Select(edge => new
            {
                Id = edge.Id,
                StartNodeId = edge.StartNodeId,
                EndNodeId = edge.EndNodeId,
                WorldId = edge.WorldId,
                ZoneId = edge.ZoneId,
                PathName = edge.PathName,
                PathType = edge.PathType,
                CellX = edge.CellX,
                CellY = edge.CellY,
                Direction = edge.Direction.ToString(),
                Weight = edge.Weight,
                Points = edge.Points.Select(point => new DebugPoint(point.X, point.Y, point.Z))
            }),
            Issues = graph.Issues.Select(issue => new
            {
                Code = issue.Code.ToString(),
                Severity = issue.Severity.ToString(),
                RoadKey = issue.RoadKey,
                Detail = issue.Detail
            }),
            Route = route == null ? null : new
            {
                Status = route.Status.ToString(),
                Detail = route.Detail,
                GraphGenerationId = route.GraphGenerationId,
                ComponentId = route.ComponentId,
                TotalWeight = route.TotalWeight,
                Waypoints = route.Waypoints.Select(point => new DebugPoint(point.X, point.Y, point.Z)),
                Steps = route.Steps.Select(step => new
                {
                    EdgeId = step.EdgeId,
                    ZoneId = step.ZoneId,
                    PathName = step.PathName,
                    PathType = step.PathType,
                    Forward = step.Forward,
                    FromDistance = step.FromDistance,
                    ToDistance = step.ToDistance,
                    Weight = step.Weight
                })
            }
        };

        return JsonSerializer.Serialize(document, JsonOptions).ReplaceLineEndings("\n") + "\n";
    }

    private sealed record DebugPoint(float X, float Y, float Z);
}

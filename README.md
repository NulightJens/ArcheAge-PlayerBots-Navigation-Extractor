# ArcheAge Navigation Extractor

[Download this preview](https://github.com/NulightJens/ArcheAge-PlayerBots-Navigation-Extractor/releases/tag/v0.1.0-alpha.1) · [Tool family](FAMILY.md)

Build navigation maps and plan routes from your local ArcheAge `game_pak`.
Optionally use recorded walks to help choose a route. Extraction runs offline;
you supply the matching game data.

**Preview 0.1.0-alpha.1 · ArcheAge 1.2 r208022 · .NET 10**

Build PlayerBots with its pinned AAEmu 1.2 host, then build this tool against that
host's output directory:

```powershell
dotnet build extractor/AAEmu.NavigationExtractor -p:AAEmuOutput=C:/AAEmu/AAEmu.Game/bin/Debug/net10.0
dotnet run --project extractor/AAEmu.NavigationExtractor -p:AAEmuOutput=C:/AAEmu/AAEmu.Game/bin/Debug/net10.0 -- --help
```

Provide `--game-pak`, `--output`, `--bounds minX,minY,maxX,maxY`, `--start x,y,z`,
`--end x,y,z` and a `--route-id`. Optional `--road-prior` accepts a recorded route.
Choose a new output file; existing output is retained. Use a bounded region so the
extraction fits your memory budget.

The output contains a navigation graph and a planned route. Check the route
in game before using it for bots: obstacles, water and height changes can make
a planned path impassable.

[Changes](CHANGELOG.md) · [Tool family](FAMILY.md)

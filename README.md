# ArcheAge Navigation Extractor

[Download this preview](https://github.com/NulightJens/ArcheAge-PlayerBots-Navigation-Extractor/releases/tag/v0.1.0-alpha.1) · [Tool family](FAMILY.md)

Create an offline navigation snapshot from your local ArcheAge `game_pak` and optional
recorded routes. It does not run a game client or server and does not redistribute game data.

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

The output describes the extracted graph and planned route. Successful extraction
does not establish that a character can traverse every edge in the live game.
The tool builds against the matching host and has extracted a bounded region
from local r208022 data. Planned routes still require review in the live game.

[Changes](CHANGELOG.md) · [Tool family](FAMILY.md)

# Globe view (not built yet)

The goal is a 3D globe in the browser that you can spin and zoom like Google Earth, with all of skybag's aviation data on it: every airport, runways once you're close enough, NOTAM closures, the planned route at its planned altitude, and the ownship during a replay.

## Why not osgEarth

osgEarth is the obvious engine for this, and terra is laid out the same way (its tile pyramid is osgEarth's "global-geodetic" profile). But osgEarth can't run where skybag has to run. It needs OpenSceneGraph, GDAL, PROJ and GEOS built from source, an OpenGL context (there is no GPU and no native GUI toolkit on the target machine), and no package manager is available. So the plan is to split it: terra stays the C++ globe engine and the browser draws with WebGL. The desktop's Chrome has WebGL, with software rendering if there's no GPU.

## What already exists

- `terra::build_mesh` (include/terra/mesh.hpp) builds one tile of the global-geodetic pyramid: a (grid+1)^2 lattice with float positions relative to a double-precision tile centre, normals, albedo from height and slope, and skirts against cracks between levels.
- `terra::bounds`, `child`, `parent`, `key_for` (include/terra/tiles.hpp) give the tile layout.
- `terra::select_tiles` (include/terra/select.hpp) does screen-space-error LOD with frustum and horizon culling. Today it runs on the server for the synthetic view.
- `skybag::Terrain::height_m` gives the terrain height. It clamps the sea to 0 m, and `raw_height_m` has the procedural height.
- The API already serves airports (`/api/search`, `/api/near`, `/api/airport`), runways with NOTAM closures (`/api/runways`), NOTAMs (`/api/notams`), the plan path and profile (`/api/plan`) and the replay track (`/api/track`).
- `web/` is plain JavaScript with no dependencies, so it works offline.

## What it should do

- A globe tab in the web UI. Drag rotates, the wheel zooms, a modifier or right-drag tilts and turns, double-click flies to a point, and the search box flies to an airport.
- Terrain tiles from terra, served by a new endpoint, refined by screen-space error as you zoom, and cached in the browser. Coarse tiles stay on screen while finer ones load. No cracks, no z-fighting, and no float jitter at street level. This needs relative-to-centre rendering, because float32 ECEF coordinates are only good to about 0.5 m.
- Airports drawn by importance (large, medium, then small as you zoom in), with labels that don't pile up. Clicking one opens its runways, closures and active NOTAMs.
- Runways drawn from their threshold coordinates once you're close enough. A runway closed by an active NOTAM at the demo clock looks different.
- The planned route as a 3D line at its planned altitudes, with the terrain profile under it. The ownship moves along it during a replay.
- Correct across the antimeridian and at the poles. Those are where a hand-written globe usually breaks.

## How we'll know it works

- Tile edges are seamless. Shared edges between neighbouring tiles match exactly (terra already checks this on the server side, see `terra --check`), and no background shows through at level changes.
- An airport marker sits on its runway at every zoom. The screen position of each runway threshold matches the projected lat/lon within 1 px at close range.
- Clicking hits the right airport: picking agrees with a brute-force nearest-on-screen search.
- It stays usable on the target machine with software WebGL. Measure frame times and tile requests per second, then set a target from the numbers.
- A Playwright script searches KSFO, flies there, clicks it and sees 10L/28R marked closed at the demo clock.

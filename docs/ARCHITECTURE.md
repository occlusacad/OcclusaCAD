# OcclusaCAD architecture

## Modules

```
        OcclusaCADDB                     OcclusaCAD
       (apps/casedb)                  (apps/designer)
             │                               │
             ├──────────────┬────────────────┤
             ▼              ▼                ▼
        occlusa_ui ───► occlusa_gfx      occlusa_db
      (GLFW, ImGui,    (OpenGL 3.3)     (repository,
     themes, dialogs)       │           SQLite, files)
             │              ▼                │
             └───────► occlusa_core ◄────────┘
              (DICOM, STL, volume, registration,
               workflows, platform, config)
```

* **occlusa_core** has no GUI or GL dependency and is fully unit-tested. Every domain algorithm lives here, so it can later be reused by a headless server or batch tool.
* **occlusa_db** depends only on core. The apps talk to `ICaseRepository` and `ICaseFileStore`, never to SQLite directly.
* **occlusa_gfx** holds OpenGL resources and shaders, but no ImGui.
* **occlusa_ui** is the shared application shell. `GuiApp` owns the window, the GL context, Dear ImGui, themes and the headless capture mode.

## Coordinate systems

* **World space = DICOM patient coordinates (LPS, millimetres)**: +X is patient left, +Y posterior, +Z superior.
* A `Volume` stores its voxel grid with `origin`, `spacing` and a `direction` matrix (columns are the i/j/k axes, which may be oblique or sheared). **Volumes are never transformed.**
* A surface scan is stored untouched in its own (scanner) coordinates. `ScanObject::transform` maps scan coordinates into world space, and registration only ever changes that matrix.
* Aligned scans are exported with the transform baked in (`design/<name>_cbct_aligned.stl`), so downstream tools see them in CBCT space.
* Slice views use radiological conventions. Axial is seen from the feet with anterior up; coronal is seen from the front; sagittal is seen from the patient's left with anterior on the left.

## DICOM

`core/dicom` is an in-house reader with no third-party DICOM library:

* `DicomDataset`: Part 10 parser covering the preamble, the meta group, implicit/explicit VR, both endiannesses, nested sequences (defined and undefined length) and encapsulated pixel data. A header-only mode stops before the pixel data and reads only a 256 KB prefix, so folder scans stay fast on network shares.
* `DicomSeries`: groups images into stacks keyed by series UID, orientation and matrix size. It sorts slices by their position projected on the slice normal, derives the slice vector from the first and last positions (which handles gantry tilt), checks spacing uniformity, applies per-frame rescale, and supports enhanced multi-frame objects through the functional groups.
* `DicomCodecs`: RLE Lossless and JPEG Lossless (process 14, including SV1 with restart markers).
  * TODO: JPEG baseline, JPEG-LS (CharLS, BSD-3) and JPEG 2000 (OpenJPEG, BSD-2).
* Voxels are stored as `int16` with a global slope and intercept. Unsigned 16-bit data is shifted losslessly.

## Rendering

OpenGL 3.3 core profile, which macOS provides as 4.1 core. Each viewport renders into a `RenderTarget`, made of a 4× MSAA colour+depth buffer, a resolved colour texture shown by ImGui, and a single-sample depth copy:

1. Gradient background, then opaque scans.
2. The depth buffer is copied, and a full-screen pass ray-casts the CBCT 3D texture (`GL_R16`). Rays stop at the copied depth. Iso-surface hits write `gl_FragDepth`, so meshes and volume occlude each other correctly. The modes are threshold surface (bisection-refined, gradient shaded), MIP and front-to-back compositing.
3. Transparent scans, then the MSAA resolve.

MPR slices draw a world-space quad through the crosshair and sample the same 3D texture, which works for oblique volumes. Overlays (crosshairs, scan outlines from mesh/plane intersection, landmarks, labels) are drawn with ImGui's draw lists, so they stay crisp at any DPI. Views re-render only when a hash of their inputs changes.

**Future**: macOS deprecates OpenGL. `occlusa_gfx` is the only module that touches GL, so a Metal/Vulkan backend can replace it behind the same `SceneRenderer` interface. Candidates are a thin RHI over Metal plus Vulkan (via MoltenVK), or bgfx/Diligent (BSD-2/Apache-2.0).

## Registration

1. **Point pairs**: the technician picks matching features on the scan (left view, scan coordinates) and on the CBCT (right view, CPU ray march to the threshold, respecting the crop box). Horn's quaternion method gives the least-squares rigid transform; collinear picks are rejected.
2. **ICP refinement**:
   * The target is a naive-surface-nets iso-surface extracted only inside a region around the scan, in a kd-tree, and cached per threshold and region.
   * Each iteration builds point-to-plane correspondences, rejects pairs whose normals disagree, and keeps the closest 80%.
   * **Coarse-to-fine**: after each convergence the search radius halves, down to 0.4 mm. This removes the systematic bias from gingiva on the scan sitting close to bone in the CBCT. On the phantom it cuts the error from about 0.3 mm to about 0.09 mm.
3. **Manual**: an ImGuizmo world-space delta about the scan centre, or millimetre/degree nudges. Undo is kept per scan.

## Crown & bridge

All geometry lives in `core/crown` (no GUI), in the coordinates of the scan containing the preparation. The scan's transform only places the design on screen, so the crown is exported in the scan's own coordinates.

1. **Scan analysis** (`PrepScan`): vertex adjacency, a signed curvature estimate per vertex (convex > 0), a kd-tree and a triangle BVH (`core/MeshBvh`). This is computed once per scan in the background.
2. **Margin detection** (`detectMargin`), from one click on the preparation:
   * The occlusal direction is the mean normal around the click. Vertices within 7.5 mm of the preparation axis are binned into 72 angular sectors and walked outwards by surface (graph) distance from the click.
   * In each sector the margin is the first strong convex edge **after** the first concave one: the inner angle of the chamfer or shoulder comes before the outer rim. This skips the convex rounded occlusal edge of the preparation and stops before the gingiva and neighbouring teeth.
   * Sectors that disagree with their neighbours (median of height and radius) are rejected.
   * 24 control points are traced into a closed loop with curvature-weighted shortest paths, so segments follow the convex ridge. Detours that would make the loop touch itself are removed.
   * Manual drawing and editing use the same tracing.
3. **Die** (`extractDie`): the scan is cut along the margin edges by a face flood fill from the preparation point. A margin that does not separate the preparation is detected (the fill escapes) and reported.
4. **Insertion axis**: the default minimises the area-weighted squared undercut (faces facing away from the axis) within 25° of the margin normal. **Blockout** works in cylindrical coordinates around the axis: a die point is undercut when material higher up along the axis sticks out further. Points closer than 0.05 mm in height are ignored, so flat shoulders and scan noise are not blocked out.
5. **Intaglio**: the die is offset along its normals by the cement gap. The gap ramps in over the distance to the margin, and an extra gap is added 1 mm further up. The result is then blocked out along the axis. The margin itself stays on a smoothed margin curve.
6. **Outer shell** (`buildCrown`): a structured grid whose bottom ring **is** the margin. The anatomy comes from a tooth library (`ToothLibraryRegistry`; see [TOOTH_LIBRARIES.md](TOOTH_LIBRARIES.md)) as azimuthal (radius, height) profiles from the height of contour to the axis.
   * Each margin vertex starts a column. Its profile runs from the margin to the height of contour along an emergence curve (a cubic Bézier), then follows the library profile.
   * The library profile is taken at the azimuth that lands on the column after anisotropic scaling to the crown's mesial/distal/buccal/lingual half-widths and height. Cusp scaling exaggerates or flattens the relief around the tooth's occlusal level.
   * **Minimum thickness**: walls are pushed out radially and the occlusal surface vertically. Both supports are dilated by the required thickness: the widest wall within that distance above, the highest point within it inwards. That way the shell also clears the edges of the die.
   * The radius may not grow again between the contour and the apex, so the pushes cannot fold the surface. Thickness ramps from the margin thickness over the first 1.2 mm.
   * Crown = reversed die faces + shell + centre fan, oriented by the die's margin edge. It is **closed and consistently oriented by construction**, and the tests check this.
7. **Fitting and contacts**:
   * Neighbour widths come from ray casts at the height of contour against the scan without the preparation. The height is a bisection on the clearance to the antagonist.
   * `adaptContacts` moves the shell along its normals. It pulls back where the signed distance is below target and extends facing areas towards proximal contacts. The correction is spread, then smoothed, and the final rounds only pull back.
   * The displacement field (one value per vertex) also stores free-form edits. It survives parameter changes and is saved with the design.
8. **Orientation**: neighbouring crowns form two opposite lobes around the preparation, which gives the mesio-distal line; the arch centroid gives mesial and lingual. Both can be flipped in the UI.

### Bridges (`core/crown/Bridge`)

* **Grouping**: `findBridges` turns the case's restorations into bridges. A bridge is a run of adjacent teeth in arch order (`archIndex`) holding abutments (crowns, copings) and at least one pontic. Splinted crowns without a pontic are not grouped.
* **Common axis**: `optimizeCommonAxis` runs the undercut minimisation on all abutment dies together. The UI shows each abutment's divergence from its own best axis. Changing the axis on any abutment moves the whole bridge.
* **Pontics**: `layoutPontics` places them between the abutments' margin centroids, in proportion to the template widths; cantilevers continue along the arch. `makePonticBase` projects an elliptical footprint onto the ridge along the axis (offset by the ridge contact). That patch acts as the die, so the pontic is built by the same crown builder, without cement gap and with a rounded basal edge.
* **Fitting**: `fitBridgeWidths` makes facing units meet: each half width is half the distance between the unit centres plus 0.15 mm. Outer sides keep the scan-based proximal fit. Contact scenes exclude the other units of the bridge.
* **Connectors** (`placeConnector`): ellipsoids centred between neighbouring units, with the requested cross-section (inflated so the tessellated section is not smaller). They sit above the higher margin or ridge plus the embrasure clearance and below the lower cusp tips minus 1.5 mm. A warning is raised when they don't fit.
* **Connector edits** (`ConnectorEdit`): the technician's changes are stored per connector, relative to the automatic placement and in the connector's own frame (along the span, bucco-lingual, occlusal). They hold an offset and optional area, height/width ratio and length, so they survive regenerated units and a moved axis. `checkConnectorFit` re-checks the edited connector against the embrasure and marginal-ridge limits. In the designer, a connector is selected from the list or by clicking it in the 3D view, and moved with a translate-only gizmo (`View3D::gizmoPivot`) or sliders.
* **Merge** (`uniteBridge`, using [Manifold](https://github.com/elalish/manifold)): every unit and connector is clipped by the preparation spaces of the *other* abutments. A preparation space (`makeCavity`) is the intaglio grown by 10 µm and closed by a skirt below the margin. The clipped parts are then united. The result is checked for watertightness, and each connector's cross-section is measured on the merged mesh (`crossSectionArea`).

The `RestorationDesign` state (margin vertex ids, axis, parameters, displacement) is part of the design JSON, as are the bridges' connector settings and common axis. Derived geometry is rebuilt on load once the scan analysis is ready. Saved vertex ids are only reused if the scan's vertex count matches.

## Custom abutments (`core/implant`)

* **Libraries** (`ImplantLibrary`): `ImplantLibraryRegistry` holds the built-in generic library (generated with Manifold) and library folders. Connections are loaded on first use and cached. The interface's top circle (`measureInterfaceTop`: highest level, outer radius there) is where the abutment starts. The format is described in [IMPLANT_LIBRARIES.md](IMPLANT_LIBRARIES.md).
* **Scan body matching** (`ScanBodyFit`):
  1. The library scan body is sampled evenly over its surface; only the top 6 mm is used, since the rest is hidden in the gingiva.
  2. It is placed at the clicked point along the mean surface normal there, in 12 rotations about the axis.
  3. Each placement is refined by point-to-plane ICP (`refineIcp`) against the scan vertices within 13 mm.
  4. Fits are compared by point-to-plane distance. Point-to-point distance would mostly measure the scan's vertex spacing.
* **Abutment geometry** (`Abutment`): `AbutmentShape` holds per control point (9, evenly spaced) the margin radius and height, the mid offset, and the core outline (free when unlocked). `buildAbutment` evaluates closed Catmull-Rom splines at 108 azimuths. Each column is a profile:
  1. a quadratic Bézier through the mid point, from the interface's top circle to the margin;
  2. the shoulder with a rounded inner corner;
  3. the core wall;
  4. a rounded top down to the screw channel;
  5. the channel down into the interface, closing inside it.

  The grid is a closed, consistently oriented solid; tests check this for edited shapes too. `finishAbutment` unites it with the library interface (Manifold), so the interface stays exactly as provided, and checks the optional minimum-thickness and blank solids by ray parity.
* **Designer** (`AbutmentSteps`):
  * `ImplantRestoration` holds the connection, the implant position (implant frame → scan) and the shape. It is saved in the design JSON with the tooth in the configured numbering.
  * Handles are dragged in the 3D view. The step sets `View3D::blockOrbit` while a drag is on, so the camera stays still.
  * The scan is shown without the scanned scan body (triangles within 0.12 mm of the matched library scan body are left out), as an overlay. The scan itself is hidden only for display (`ScanObject::stepHidden`, not saved).

## Workflow engine

* `core/Workflow` declares every **step** (id, title, group, guidance, implemented flag) and the **workflows** as ordered step lists: `implant_planning`, `surgical_guide`, `custom_abutment` and `crown_bridge`.
* OcclusaCAD DB picks the workflow from the restoration types; the technician can override it.
* In the designer, each step is a `Step` subclass with these hooks:
  * `drawPanel`
  * `blocker` (why the wizard can't advance yet)
  * `preferredLayout`
  * viewport `overlay` / `onViewEvent`
  * `onEnter` / `onLeave`

  Tools that aren't implemented yet are `PlaceholderStep`s, so workflows are already complete end to end.
* **Wizard mode** shows the workflow's steps. Completed steps and the next one can be clicked, and Next is gated by `blocker`. **Expert mode** lists every step grouped by category and can open any of them, including steps that aren't in the case's workflow.
* Adding a tool means implementing a `Step`, registering it in `DesignerApp::onStart`, and flipping `implemented` in `core/Workflow.cpp`.

## Case database

`ICaseRepository` covers cases, restorations and files, plus the design state (JSON), advisory locks and an `ICaseFileStore`.

### SQLite backend (network share)

* One `occlusacad.sqlite3` file under the data root. Schema versioning uses `PRAGMA user_version`, with migrations in `SqliteCaseRepository::migrate`.
* **Network share safety**: `journal_mode=DELETE`, because WAL needs shared memory and breaks on SMB/NFS. `synchronous=FULL`, a 15 s busy timeout, short `BEGIN IMMEDIATE` write transactions, and no long-lived read transactions.
* **Optimistic concurrency**: every case row has a `revision`. `updateCase` fails with `ConcurrencyError` if another workstation saved first. When the designer saves, it re-reads the record and merges its changes so edits made meanwhile in OcclusaCAD DB (notes, status) survive.
* **Advisory locks** (`locked_by = user@host`): The designer takes the lock when it opens a case. Other users get a read-only view or can take over a stale lock. OcclusaCAD DB can release locks.
* **File store**: `<dataRoot>/cases/<caseNumber>_<uuid8>/{dicom,scans,design}`. Paths are stored **relative to the case folder**, so `\\server\share`, `Z:\` and `/Volumes/share` all resolve.
* Known SQLite-on-share caveats, documented for IT: use a reliable SMB3/NFSv4 server with working byte-range locks. Don't use sync tools such as Dropbox or OneDrive for the data folder. Keep backups at the file level.

### Cloud backend (stub)

`CloudCaseRepository` and its file store throw `NotImplementedError`. The planned design, documented in the header:

* REST/JSON API mirroring `ICaseRepository`, with ETag/If-Match for revisions and server-side lock leases.
* Object storage with pre-signed URLs, plus a local cache.
* OAuth2 device-code sign-in, with tokens in the OS keychain.
* HTTP via a permissively licensed client.

`AppConfig::databaseBackend` selects the backend.

## Cross-platform notes

* All strings are UTF-8. Paths cross the boundary through `platform::pathFromUtf8` / `pathToUtf8`, and the Windows command line is re-read as UTF-16.
* Platform specifics live in `core/Platform.cpp`: config and documents folders, executable path, user and host names, dark-mode detection (registry, `defaults`, `gsettings`), detached process launch (`CreateProcessW` / `posix_spawn`) and opening a folder.
* HiDPI: on Windows and Linux the UI scale follows the monitor content scale (configurable through `uiScale`). On macOS the framebuffer scale handles Retina.
* Windows builds are GUI-subsystem executables with a `main` entry point. The dark title bar follows the theme through DWM.
* CI builds and tests on Ubuntu, Windows (MSVC) and macOS (AppleClang). See `.github/workflows/ci.yml`.

## Testing

* `occlusa_tests` (doctest) covers:
  * STL round trips; the DICOM writer → reader round trip (explicit, implicit, RLE, oblique geometry, shuffled file names); JPEG Lossless with all 7 predictors and restart markers.
  * Horn's method, ICP convergence, iso-surface accuracy, watertightness and consistent orientation.
  * BVH queries against brute force; crown & bridge on a synthetic shoulder preparation with neighbours and an antagonist:
    * margin accuracy against the analytic margin;
    * die extraction, insertion axis and undercuts;
    * cement gap, proximal and occlusal fitting, contact adaptation;
    * minimum thickness, no folded faces, watertight crowns and copings.
  * Bridges on a synthetic two-preparation site with a ridge:
    * grouping and pontic layout, including cantilevers;
    * the common axis and the pontic resting on the ridge;
    * connector areas (exact on the mesh, and measured on the merge);
    * one watertight solid when the units only touch through the connectors;
    * the intaglios untouched by connectors.
  * Ray casting, plane slicing, workflows, tooth numbering.
  * The SQLite repository: CRUD, concurrency conflicts, locks, two connections sharing one file, and file import.
* `designer_alignment_e2e` (opt-in, `-DOCCLUSACAD_HEADLESS_TESTS=ON`, Linux + EGL) generates the phantom, runs the designer headless with simulated landmark picks, and fails if the error against ground truth exceeds 0.25 mm.
* `designer_crown_e2e` (same option) opens the phantom crown case. It detects the margin from one click, sets the axis, designs and saves the crown. It fails if the margin deviates more than 0.3 mm from the analytic margin, or if the crown is not watertight or not saved to the case.

  It then does the same for the bridge case 35-36-37: two margins, the common axis, the pontic, connectors, and the merged bridge saved as one STL.
* `designer_abutment_e2e` (same option) opens the phantom abutment case, chooses the generic RP 4.1 connection and matches the scan body from a click. It fails if the implant platform is off by more than 0.05 mm (or the axis by more than 1°), or if the default abutment is not united with the interface and saved. It then repeats this with the library exported to a lab library folder by `occlusa_implantlib`.
* Unit tests for implant libraries (closed generic geometry, the top circle, the `library.json` round trip), abutment shapes (default margin 0.5 mm wider, mid points controlling convexity, the locked core following the margin, unlocking, thin-wall warnings, the union with the interface) and scan body matching on a rotated synthetic scan.

## Roadmap

1. Implant restorations: exocad and 3Shape implant library import, crowns on custom abutments and screw-retained crowns, angled screw channels, abutment emergence fitted to the gingiva scan.
2. Crown & bridge: inlays/onlays and veneers, per-connector editing and pontic shape options (ovate, sanitary), a larger tooth library (morphable library teeth from scanned shapes), a dynamic occlusion check, and margin re-meshing for scans coarser than the margin.
3. Panoramic curve and reconstruction, then nerve canal tracing.
4. Implant placement on the CBCT with the implant libraries, with safety distances; virtual teeth.
5. Sleeve setup and surgical guide design (offset surface from the scan, sleeve holes, Boolean operations).
6. The cloud backend and user accounts.
7. JPEG 2000 / JPEG-LS DICOM, and DICOMDIR browsing.
8. Packaging: macOS app bundles and notarisation, a Windows MSI, Linux AppImage/Flatpak, file associations, and an icon set.
9. A Metal/Vulkan renderer backend.

# VehDeformIII archive study — 2026-08-17

Source archive: `vehdeform 3 test.zip`

## Inventory

- 59 files total
- 57 C++ snapshots + 2 text/code notes
- Several exact duplicate snapshots
- Useful chronology is preserved by filenames and ZIP timestamps (June 15–22, 2026)

## Evolution visible in the archive

### 1. June 15 — beam/lattice experiments

Representative files:

- `SoftVehicleBeamNGish_GTAIII_SmartImpact.cpp`
- `SoftVehicleBeamNGish_GTAIII_AnatomyPanels.cpp`
- `SoftVehicleBeamNGish_GTAIII_DamageRigGroups.cpp`
- `SoftVehicleBeamNGish_GTAIII_StableDamageFields.cpp`

The important endpoint is `StableDamageFields.cpp`. Its header explicitly documents why the live solver was removed:

- live/oscillating lattice could blink/stretch;
- GTA OK/DAM atomics could toggle while only one variant was deformed;
- per-frame solver output could fight RenderWare/GTA damage swaps.

The replacement was persistent per-vertex offsets written as `original + offset`.

The stable dent itself was still fairly complicated (axial/tangent gates, scrape stretch, role classification, buckle ring), but its core invariants were good:

- collision point and push are shared in vehicle space;
- vertices are evaluated from their original/rest position;
- accumulated deformation is stored separately from the original mesh;
- the written mesh is always original vertex + accumulated offset;
- opposite-side bleed is explicitly limited;
- repeated tiny signals are not summed into a giant hit.

### 2. June 19 — private geometry + event-field transition

Representative files:

- `vehdeform_iii_privategeom_lowpoly.cpp`
- `vehdeform_iii_saresource_field.cpp`
- `vehdeform_iii_vcproper_field.cpp`
- `vehdeform_iii_vcproper_field2.cpp`
- `vehdeform_iii_component_field3..10`

This era establishes several things that kept surviving later:

- per-vehicle private geometry is necessary for traffic/all-cars support;
- cached bind/rest data is more stable than deforming shared model resources;
- damage should be event-driven, not continuously solved every frame;
- collision trace memory becomes increasingly important;
- component damage status becomes context/fallback, not the primary geometric selector.

The SA/VC-style event fields often measured distance from the **current deformed vertex** (`base + offset`). This allows the active deformation region to move with the already-bent surface, but it also introduces the later-described “dent chasing itself” failure mode under repeated impacts.

`vcproper_field2.cpp` also applies panel-cell, mesh-neighbor, and broad low-poly smoothing together. Later branches progressively disable the broad versions.

### 3. June 20 — component-field complexity, then practical reset

Representative files:

- `component_field12_axisfix.cpp`
- `component_field13_panelcontrol.cpp`
- `component_field14_cage.cpp`
- `component_field15_partaware_cage.cpp`
- `component_field16_practical_reset.cpp`
- `component_field17_asifield.cpp`

`field16_practical_reset` is a major signal. Its header explicitly says:

- no structural cage;
- no OK/DAM follower lattice;
- clone render mesh per vehicle;
- deform owned geometry only;
- optionally suppress stock damaged-part swapping so GTA cannot flicker between differently-deformed variants.

`field17_asifield` keeps the event architecture but moves back toward a simple radius-gated field. It still uses current deformed vertex position for distance and allows small smoothing/noise, so it is not the final stable form.

### 4. June 22 — direct-field / proper-port convergence

Representative sequence:

- `field19_directfield.cpp`
- `field20_properport.cpp`
- `field21_livefallback.cpp`
- `field23_safe_fullfeatures.cpp`
- `field24_compilefix.cpp`
- `field25_frogfix.cpp`
- `wipvehdeform.cpp`
- archive root `Main.cpp`

This is the most relevant lineage for the next implementation.

The repeated description is essentially:

`collision trace -> VehicleDamage/fallback -> radius-gated ORIGINAL vertex dent`

The deformation loop converges on these rules:

1. Distance is measured from `v.baseVehicle`, not `base + offset`.
2. Linear spherical falloff: `1 - distance/radius`.
3. Amount is derived from impact impulse and falloff, then capped per vertex.
4. A second simple center-resistance term reduces outer-radius movement.
5. Hinged/bumper/structure layers only apply hard safety caps.
6. Delta is simply `push * amount`.
7. Accumulated offset is clamped and stored persistently.
8. Broad panel/zone smoothing is disabled by default.
9. Small triangle-neighbor smoothing survives (typically 0.12) because it is topology-local.
10. Crinkle is disabled in the “proper port” defaults.

The comments explicitly explain the original-position rule as preventing the dent from “chasing itself” and turning into spikes/sails across repeated impacts.

## Important correction to the current 2026-08-17 direction

The late branches define a helper named `StableDentDirection()` with optional inward-normal and radial blending. However, in:

- `field19_directfield.cpp`
- `field20_properport.cpp`
- `field21_livefallback.cpp`
- `field23_safe_fullfeatures.cpp`
- `field24_compilefix.cpp`
- `field25_frogfix.cpp`
- `wipvehdeform.cpp`
- root `Main.cpp`

it is defined but not actually called by `ApplyDamageEvent`.

The real late-branch deformation line is `delta = push * amount`.

Therefore the recently proposed `.65 impact / .20 surface-normal / .15 radial` reconstruction should NOT be treated as the proven III baseline. That weighting may be useful as an optional experiment later, but the archive does not support it as the behavior the III iterations converged on.

## Smoothing lesson

Late defaults strongly favor:

- `LowPolySmoothing = 0`
- `PanelCellSmoothing = 0`
- `PanelNeighborSmoothing = 0`
- `MeshNeighborSmoothing ~= 0.12`
- one pass

This is important. Seam coordination should come from a common impact coordinate/event, not from averaging offsets across unrelated atomics/panels.

## Noise / crinkle lesson

The later code still contains crease/noise machinery, but the proper-port default sets `DirectCrinkleScale = 0`.

This means crinkle/noise should not be part of the baseline deformation core. It can be reintroduced only after the basic crush is stable.

## OK/DAM / flicker lesson

This problem appears repeatedly and is explicitly documented by the old source.

Two historically attempted solutions:

1. deform both OK and DAM atomics consistently;
2. suppress stock damaged-part swaps and keep one visual mesh family authoritative.

The next GTA III-specific implementation should not choose between these by heuristic. Inspect `CAutomobile::SetComponentVisibility` / damage-manager callers in the GTA III IDB and implement the smallest deterministic policy that matches III.

The current clean 2026-08-17 refactor clones all non-wheel atomics, which is useful, but simply cloning both variants is not enough if their base topology/positions differ and visibility swaps occur asynchronously.

## RenderWare refresh lesson

`field20_properport` reintroduced manual RenderWare repEntry invalidation because the reversed working mod did it.

Later `frogfix`/root Main progressively guard that code:

- atomic repEntry considered safer;
- geometry repEntry marked RW-build-sensitive and disabled unless needed;
- many pointer/readability/finite checks added after crash reports.

This history supports keeping manual repEntry destruction OUT of the clean baseline unless the GTA III RW path proves normal `RpGeometryLock/Unlock` does not re-instance changed vertices.

## Collision / “frog” lesson

`field25_frogfix` and root `Main.cpp` identify repeated low-normal-speed building contacts as the source of a sidewalk/building “frog” crash/distortion pattern.

Important rules from that branch:

- do not use total vehicle speed as deformation force;
- prefer normal collision speed, stock damage impulse, health loss, and actual damage changes;
- ignore low-normal-speed building chatter unless there is a real damage/force signal;
- collision trace should cache exact point/direction first, then VehicleDamage consumes it.

The new III-native max-impulse collision-sample selection is an improvement over these old approximations and should stay. A building-contact chatter gate may still be needed around it.

## Geometry ownership lesson

Private per-vehicle geometry is one of the most persistent successful ideas across the archive.

Keep:

- deep/private geometry per vehicle;
- immutable original/rest vertices;
- persistent accumulated offset separate from rest data;
- repair writes rest geometry back;
- model-change/destructor cleanup is explicit.

Do not go back to deforming model-shared geometry.

## Component-frame lesson

Most old event-field branches cached `baseVehicle` once. This is problematic for an opened/moving door, bonnet, or boot.

The current clean refactor's idea of transforming the component's immutable atomic-local rest vertex through the component's **current frame** at impact time is an improvement and should be retained.

The key is to combine it with the old stable rule:

- immutable local rest vertex;
- current frame only determines where that rest vertex is **now** for distance tests;
- the stored plastic offset remains in the component's local mesh coordinates / a stable basis;
- never use the already-deformed vertex as the next impact's distance center.

## Wheel lesson

The archive mostly avoids wheel-area deformation rather than solving wheel/body synchronization.

The new render-scoped wheel-frame correction is therefore genuinely new work, not something validated by these snapshots. It should be isolated from body deformation during testing:

1. body deformation stable with WheelSync off;
2. then enable wheel visual offset and validate independently.

## Recommended next baseline

Do NOT port the archived 3900-line root `Main.cpp` wholesale.

Use the current clean III-native hook/lifetime shell, but replace its ellipsoidal crush field with the late direct-field core:

1. exact GTA III max-impulse collision sample;
2. immutable original atomic-local vertex;
3. transform rest vertex through current component frame for the impact distance test;
4. spherical radius around exact contact;
5. linear falloff from rest/current-frame position, NOT already-deformed position;
6. pure collision push direction for baseline;
7. simple per-vertex impulse amount + per-hit cap;
8. simple vehicle-envelope center resistance;
9. layer-specific hard caps only;
10. accumulate persistent offset;
11. write `rest + accumulated offset`;
12. optional triangle-neighbor smoothing only, default low/off;
13. recalculate normals from changed triangles for lighting, but normals do not drive the next dent;
14. no noise/crinkle baseline;
15. no broad zone/panel smoothing;
16. no beam/cage solver;
17. no manual repEntry invalidation unless proven necessary;
18. verify deterministic OK/DAM policy against GTA III IDB;
19. add building-contact chatter rejection if exact max-impulse selection alone still allows scrape spam;
20. test body first with wheel sync disabled.

## Candidate historical tuning to use only as reference

From the late proper-port/root lineage, not as authoritative constants:

- radius around 1.75 vehicle-local units
- per-hit vertex cap around 0.30–0.45
- direct center resistance around 0.35
- broad smoothing = 0
- mesh-neighbor smoothing ~0.12
- direct crinkle = 0

The old `DirectImpactMult=0.015` must NOT be copied blindly because the old hook/event impulse domain was inconsistent across revisions (`MinImpulse` even reached 0.05 in field20). The current GTA III IDB-grounded `m_fDamageImpulse`/VehicleDamage path should define the new scale.

## Suggested runtime test ladder

1. Moderate straight front collision into wall.
2. Hard straight front collision.
3. Rear collision.
4. Side collision at door/fender seam.
5. Repeated impacts at the same exact area (look for chasing/spikes).
6. Scrape along a building with little normal speed (look for frog/distortion spam).
7. Car-to-car collision with multiple contact points (verify chosen max-impulse contact).
8. Bonnet/boot/door opened, then impact that component (verify moving-frame behavior).
9. Trigger stock damaged-part visibility changes (look for OK/DAM flicker/discontinuity).
10. Repair vehicle (full rest restoration).
11. Only after body is stable: enable WheelSync and test suspension/steering while damaged.

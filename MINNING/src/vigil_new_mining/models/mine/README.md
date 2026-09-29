# Licensed mine assets

This package expects the **Old underground mine excavation 06** asset by
[`archiwum_xyz`](https://sketchfab.com/3d-models/old-underground-mine-excavation-06-d020ab9eb98e416da9fd3ccdf88e43d2).
The model is sold under the Sketchfab Standard license and its API marks it as
non-downloadable, so the geometry and textures are deliberately not stored in
this public repository.

Licensed owners should export the six UDIM sections into `meshes/` as:

```text
mine_u1_v1.obj  mine_u1_v2.obj
mine_u2_v1.obj  mine_u2_v2.obj
mine_u3_v1.obj  mine_u3_v2.obj
mine_collision.stl
```

Place the matching 4096px diffuse and normal maps in `materials/textures/`:

```text
tex_u1_v1_diffuse.png  tex_u1_v1_normal.png
...
tex_u3_v2_diffuse.png  tex_u3_v2_normal.png
```

The checked-in world preserves the six material assignments and resolves these
paths at runtime. `run.sh` stops with a clear message when the licensed files are
missing.

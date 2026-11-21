# 149 · Scene replay render

Renders scenes that were simulated **outside** RNE with RNE's wgpu PBR renderer.
The external simulator writes a JSON script of primitives (boxes, spheres and
cylinders) and camera views. This example draws every view and writes PNG files.

```bash
cargo run --release -p scene_replay_render --example 149_scene_replay_render -- script.json out_dir
RNE_SKIP_GPU=1 cargo run -p scene_replay_render --example 149_scene_replay_render   # validate only
```

The schema is documented at the top of [`main.rs`](main.rs). Coordinates are
metres, the Y axis points up, and quaternions are `[x, y, z, w]`. Each camera
looks down its local −Z axis.

It is used to render the hero GIFs of the
[navigation-foundation-model](https://github.com/rsasaki0109/navigation-foundation-model)
sandbox, which compares VLM + planning, VLA, world-model planning and
world-action models. Without a GPU it renders through llvmpipe (Mesa's
software Vulkan driver).

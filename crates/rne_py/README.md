# rne_py

Python bindings for [Robot Native Engine](https://github.com/rsasaki0109/RobotNativeEngine).

```bash
pip install rne_py
```

Installing from source needs a Rust toolchain (see `rust-toolchain.toml`):

```bash
pip install "git+https://github.com/rsasaki0109/RobotNativeEngine#subdirectory=crates/rne_py"
# or, from a checkout
pip install ./crates/rne_py
```

```python
import rne_py

episode = rne_py.DiffDriveEpisode(goal_x_m=1.0, max_steps=300)
step = episode.reset()
while not step.done:
    step = episode.step(6.0, 6.0)
```

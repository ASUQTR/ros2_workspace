# Where rosbags are stored on the Jetson

The Jetson has two storage devices:

| Device | Mount point | Size | Used for |
|---|---|---|---|
| eMMC | `/` | ~28 GB | OS, this repo, docker images |
| NVMe SSD | `/mnt/nvme` | ~477 GB | rosbags |

`workspace/bags` should be a symlink to a directory on `/mnt/nvme` (e.g.
`/mnt/nvme/rosbags`), not a real directory on the eMMC. The eMMC fills up
fast — a handful of `ros2 bag record` sessions is enough to leave the Jetson
without disk space to boot or build.

`env/jetson/start.sh` bind-mounts `/mnt/nvme` into the container at the same
path (`-v /mnt/nvme:/mnt/nvme`) so the symlink resolves correctly whether
you're on the host or inside the container.

## Recording a bag

Always record from inside `workspace/bags` so the output lands on the NVMe:

```bash
cd /workspace/bags
ros2 bag record -o my_bag_name --storage mcap -a
```

`ros2 bag record -o <name>` creates `<name>` relative to the current working
directory — running it from anywhere else (e.g. `/workspace`) creates the bag
there instead, on the eMMC.

## One-time setup on a Jetson

If `workspace/bags` isn't already a symlink:

```bash
mkdir -p /mnt/nvme/rosbags
mv workspace/bags/* /mnt/nvme/rosbags/ 2>/dev/null
rmdir workspace/bags
ln -s /mnt/nvme/rosbags workspace/bags
```

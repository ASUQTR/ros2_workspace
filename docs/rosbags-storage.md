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
sudo mkdir -p /mnt/nvme/rosbags
sudo mv workspace/bags/* /mnt/nvme/rosbags/ 2>/dev/null
sudo rmdir workspace/bags
sudo ln -s /mnt/nvme/rosbags workspace/bags
```

(`sudo` is needed because bag files are written as root inside the
container, so the existing `workspace/bags` contents are root-owned.)

## Running your own container alongside someone else's

`env/jetson/start.sh` and `connect-jetson.sh` both default to the container
name `asuqtr_ros2_manual_assisted`. If a teammate already has a container
running under that name (check with `docker ps`), starting or connecting
with the default name will either fail (`name already in use`) or drop you
into *their* container — which may not even be mounting this checkout, so
`/workspace/bags` can appear missing there.

Use a distinct name via `ASUQTR_CONTAINER_NAME` to avoid this:

```bash
ASUQTR_CONTAINER_NAME=asuqtr_ros2_<yourname> bash env/jetson/start.sh
# in another terminal, to get a second shell in the same container:
ASUQTR_CONTAINER_NAME=asuqtr_ros2_<yourname> ./connect-jetson.sh
```

## Recording real data (two terminals)

`ros2 bag record -a` only captures topics that are actively being published.
To record real sub data, start the sub nodes in one terminal and record in
another shell of the *same* container:

```bash
# terminal 1 — inside the container
bash /workspace/start-sub.sh

# terminal 2 — docker exec -it asuqtr_ros2_<yourname> bash
cd /workspace/bags
ros2 bag record -o my_bag_name --storage mcap -a
# Ctrl+C to stop
```

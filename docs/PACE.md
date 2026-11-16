# Running isaacdrones on Georgia Tech PACE ICE

ICE is the only machine we have that runs Isaac Sim, so the code in this
repository is verified there. Every step goes through `scripts/pace.sh`, run
from the clone on a login node, on the campus network or VPN.

Isaac Sim runs inside an Apptainer container. ICE nodes run RHEL 9 with glibc
2.34 and Isaac Sim 6.2's wheels require 2.35, so `uv sync --extra sim` fails
on a node directly. The container is NVIDIA's Isaac Sim 6.2 image, which
supplies Ubuntu 24.04 and the Vulkan libraries. We do not use the Isaac Sim
inside the image. The script runs `uv sync --extra sim` in the container, so
ICE gets the versions in `uv.lock`, as any Linux machine would. The cost is
that Isaac Sim is stored twice, in the image and in the environment.

On 2026-11-16 a fresh `setup` took 16 minutes, and `uv run pytest` then
reported 7 passed on a Blackwell and on an H100. The tests
cover a headless boot at one PhysX step per frame, `reset()` returning
simulation time to its start, an RTX camera producing an image, and the pose
of a falling cube. Startup takes about 10 s with our app file,
`src/isaacdrones/sim/isaacdrones.kit`, against one to three minutes for Isaac
Sim's stock app. Every command in the script has run. `stream` and `--video`
last ran on 2026-11-14, before the camera code was last changed. RTX lidar and
radar are untried.

## Commands

The first time, on a login node:

```bash
ssh YOUR_USERNAME@login-ice.pace.gatech.edu
git clone git@github.com:EthanMBoos/isaacdrones.git ~/isaacdrones  # needs a key
cd ~/isaacdrones
pace-quota               # setup adds 55 GB and 245,000 files to scratch
scripts/pace.sh node     # wait for a CPU node; the prompt changes
cd ~/isaacdrones
scripts/pace.sh setup    # pulls the image and installs Isaac Sim
scripts/pace.sh run uv run pytest -m "not sim"    # expect 3 passed
exit                     # release the node
```

To queue a command on a GPU node, from the login node:

```bash
scripts/pace.sh cards                  # card types and what is busy
scripts/pace.sh batch uv run pytest    # expect 7 passed
squeue --me                            # PD = waiting, R = running
tail -60 ~/scratch/isaacdrones/logs/JOBID.log
```

To work on a GPU node interactively:

```bash
scripts/pace.sh gpu      # wait for a node
scripts/pace.sh run uv run isaacdrones --headless --steps 100
scripts/pace.sh shell    # a shell inside the container
exit                     # release the GPU
```

`gpu`, `batch` and `stream` take a card name before the command: `blackwell`
(the default), `l40s`, `a40` or `h100`, as in `scripts/pace.sh batch h100 uv
run pytest`. `scripts/pace.sh space` prints the quota and the size of `data/`
and `runs/`, and `scripts/pace.sh clean runs`, `clean data` or `clean build`
deletes one of them after asking.

**Cloning.** The repository is private, so ICE needs a key that GitHub knows.
ICE accounts come with `~/.ssh/id_rsa`, which PACE uses between cluster nodes
and which must stay as it is. A second key serves GitHub:

```bash
ssh-keygen -t rsa -b 4096 -f ~/.ssh/id_rsa_github -C "ice-github"
printf 'Host github.com\n  User git\n' >> ~/.ssh/config
printf '  IdentityFile ~/.ssh/id_rsa_github\n' >> ~/.ssh/config
printf '  IdentitiesOnly yes\n' >> ~/.ssh/config
chmod 600 ~/.ssh/config
cat ~/.ssh/id_rsa_github.pub
```

Add the printed line at <https://github.com/settings/keys>. `ssh -T
git@github.com` then answers with your GitHub username.

**Setup.** Setup needs no GPU, so it runs on a CPU node. Most of its time is
the image pull and the Isaac Sim download. While unpacking, Apptainer prints
one warning per file ending `ignoring (usually) harmless EPERM on setxattr`,
and then builds the `.sif` file for several minutes without output. Setup ends
with `Setup done`. It skips whatever is already present, so it can be rerun,
and it is also the repair: `clean build` followed by `setup` rebuilds
everything and leaves `data/` and `runs/` alone. After pulling commits that
change dependencies, run `scripts/pace.sh run uv sync --extra sim`.

**Hub.** The image points Isaac Sim at Hub, Omniverse's asset cache, which
does not run here. Each launch then spends about ten seconds retrying and logs
dozens of `Hub failed to launch` warnings. The script therefore sets
`OMNICLIENT_HUB_MODE=disabled`. Assets fetched from NVIDIA's servers are then
not cached between runs, which costs nothing while we fetch none. Setting
`OMNICLIENT_HUB_MODE=shared` before calling the script keeps Hub on.

**Denoising.** Apptainer's `--nv` passes the NVIDIA driver into the container
but omits `libnvidia-ngx`, the library behind DLSS, with which Isaac Sim's
default renderer denoises. Without it the log shows `Failed to create NGX
context` and camera images are grainy. The script therefore binds the node's
copy to `/usr/local/lib/libnvidia-ngx.so.1`. On an H100 the log then warns
that DLSS-RR is unsupported, with no visible effect: a still image and a
two-second video of the falling cube are as clean as a Blackwell's.

**The first image.** On a node that has not rendered recently, the first
camera image takes a minute or more while shaders compile. Later images arrive
at once.

## Storage

Home on ICE is 30 GB, and scratch is 300 GB and 1,000,000 files. The clone
therefore goes in home and everything large in scratch.

```text
~/isaacdrones/                  the git clone; code only
~/scratch/isaacdrones/
  data/                         yours: downloaded assets
  runs/                         yours: outputs, a folder per run
  images/isaac-sim-6.2.0.sif    the container image
  apptainer-cache/              layers kept from the image pull
  apptainer-tmp/                used when a job has no $TMPDIR
  uv/                           uv, the Python it downloads, its wheel cache
  venv/                         the project environment, Isaac Sim included
  home/                         the home directory seen inside the container
  logs/                         output of batch jobs, one file per job
```

Isaac Sim writes its shader cache and user data under `home/` and `venv/`, so
shaders compile once and persist between jobs. Inside the container the clone
is at `/workspace/isaacdrones` and scratch is at its outside path.
`ISAACDRONES_SCRATCH` moves the scratch folder.

Scratch has three limits. It is not backed up, and PACE clears old files from
it, so a run that matters must be copied off. The file limit binds before the
space limit: the build uses 245,000 of the 1,000,000 files, and `--video`
writes one PNG per frame, which is 36,000 files for ten minutes at 60 Hz. And
a job's working folder is the clone, so a relative path such as `--camera
out.png` writes into home; outputs belong under
`~/scratch/isaacdrones/runs/`.

**Copying files off ICE.** ICE cannot reach your computer, so every copy
starts from your computer, in a terminal not logged in to ICE. `scp` asks for
the same password and Duo prompt as `ssh`. A path without a leading slash
starts in your ICE home directory.

```bash
scp YOUR_USERNAME@login-ice.pace.gatech.edu:isaacdrones/out.png .
scp -r YOUR_USERNAME@login-ice.pace.gatech.edu:scratch/isaacdrones/runs/NAME .
scp notes.txt YOUR_USERNAME@login-ice.pace.gatech.edu:
```

## GPU nodes

The script knows four card types. The counts are from `sinfo` on 2026-11-14.

| Name | Card | Partition | Cards | RT cores |
|---|---|---|---|---|
| `blackwell` | RTX Pro 6000 Blackwell | `ice-bw-gpu` | 4 nodes of 16 | yes |
| `l40s` | L40S | `ice-gpu` | 4 nodes of 8 | yes |
| `a40` | A40 | `ice-gpu` | 2 nodes of 2 | yes |
| `h100` | H100 | `coe-gpu` | 19 nodes of 8 | no |

`card_request` in `scripts/pace.sh` holds this table, and `scripts/pace.sh
cards` prints it with the current node states. An `idle` or `mixed` node may
have a free card, and an `allocated` node has none. ICE also has two nodes of
Quadro RTX 6000 cards, tagged `cuda12`, which suggests a driver older than the
595.58.03 that Isaac Sim 6.2 requires; the script omits them. The Blackwell
and H100 nodes ran driver 615.71.09 on 2026-11-14.

`batch` is the reliable way to get a node. A batch job waits in the queue
unattended and writes to `~/scratch/isaacdrones/logs/JOBID.log`, whereas an
interactive request made with `gpu` is lost if the terminal closes while it
waits. Both get four hours.

The script names the partition explicitly. Otherwise ICE assigns the job to
several partitions, including ones with no matching card, and the job stays at
`PD` with reason `BadConstraints` even when a card is free. A healthy waiting
job shows `(Priority)` or `(Resources)`.

**H100s.** ICE has 19 H100 nodes and 10 RTX nodes, so an H100 job usually
starts within seconds where an RTX job waits. NVIDIA does not support Isaac
Sim on a GPU without RT cores, so what works on an H100 is what we have
measured, and it may break in a later release. Physics works: the test suite
gives the same result as on a Blackwell. Mesh rendering works in the one scene
tried: a 640x480 image of the cube matches the Blackwell's, and 120 frames of
the cube falling differ by 0.3 out of 255 per pixel on average, with equal
grain. Textures, reflections and fast camera motion are untried. Gaussian
splats do not draw, and nothing in the log says so; isaacdrones has none, but
anything else that needs RT cores may fail as silently. Streaming does not
work, because an H100 has no video encoder.

**Two H100 cards crash every run.** The log shows `GPU crash is detected` and
`ERROR_DEVICE_LOST` about ten seconds in, before any isaacdrones output. On
2026-11-14 this happened on all nine runs given the card at PCI bus `BE:00.0`
on `atl1-1-03-010-10-0` or the card at `0A:00.0` on `atl1-1-03-010-25-0`,
while runs on other cards of those nodes and on four other H100 nodes passed.
Slurm tends to return the same card, so a resubmitted job fails again, and
`card_request` therefore excludes both nodes. PACE may repair the cards, so
the exclusion should be retried occasionally. On 2026-11-16 one run on a
third node, `atl1-1-03-011-3-0`, crashed the same way, and runs before and
after it on that node passed. The `Nodes:` line at the end of a log names the
node.

## The GUI on your own computer

isaacdrones can run on an ICE node with Isaac Sim's window streamed to the
Isaac Sim WebRTC Streaming Client on your computer. The client shows the full
GUI. This worked on 2026-11-14 from a Mac, with Isaac Sim on a Blackwell.

The client is installed once, from the "Isaac Sim WebRTC Streaming Client"
row of NVIDIA's download page
(<https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/download.html>),
and your computer needs a clone of this repository. Each session takes two
commands. The first runs on the ICE login node, and arguments after `stream`
go to `isaacdrones`:

```bash
scripts/pace.sh stream --scene falling-cube
```

The script submits the job, waits for a card and for Isaac Sim to load, and
prints the second command with your username and the node filled in. That one
runs on your computer, in the clone, with the VPN on:

```bash
python3 scripts/stream_relay.py connect YOUR_USERNAME NODE
```

It logs in to ICE and then tells you to open the client, enter `127.0.0.1`
and connect. The scene starts paused. To finish, close the client, stop the
relay with Ctrl-C and run `scancel JOBID` on ICE; the job otherwise ends after
two hours.

**The two waits.** `scripts/pace.sh stream` reports which one it is in. The
first is the queue. The second is Isaac Sim's startup, which takes three to
six minutes, because streaming uses the stock app: its few hundred extensions
are the GUI, and they load from 245,000 small files on shared storage. Nothing
is built or installed in that time. Headless runs avoid it because
`isaacdrones.kit` loads about a dozen extensions.

**How it works.** The stream uses TCP 49100 on the node to set up the
connection and UDP 47998 for the video. ICE passes neither from the VPN to a
compute node, and only the login node reaches them, so `stream_relay.py
connect` opens an SSH tunnel through the login node. An SSH tunnel carries
only TCP. The relay therefore wraps the UDP video in TCP, and a second copy,
started on the node by `scripts/pace.sh stream`, unwraps it.

```text
client app -> UDP 47998 -> stream_relay.py connect -> SSH tunnel, TCP 47999
           -> stream_relay.py node -> UDP 47998 -> Isaac Sim
client app -> SSH tunnel, TCP 49100 -> Isaac Sim
```

On a machine of your own with an RTX card none of this is needed: `uv run
isaacdrones --stream` and the client connected to `127.0.0.1` suffice.

**Limits.** The GPU's video encoder produces the stream, so `stream h100`
refuses. A node carries one stream and one client at a time, and a second
stream job on the same node fails to open the ports. The simulation is paced
to real time while streaming, and video through a TCP tunnel can stutter on a
poor connection. The stream has no password or encryption: between you and ICE
it travels inside SSH, but on the node the ports are open to the rest of ICE,
so another ICE user who knew the node could connect. Isaac Sim's own streaming
app (`isaacsim isaacsim.exp.full.streaming`) failed to load its extensions
here, which is why we start the stream from our code. An Open OnDemand desktop
is the other route to a window, and it is untried.

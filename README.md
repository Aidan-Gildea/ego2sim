<div align="center">

<h2>Ego2Sim: Photorealistic Contact-Enabled Simulation from Egocentric Data</h2>

<a href="https://ego2sim.xyz/ego2sim.pdf"><img src="https://img.shields.io/badge/paper-red" alt="Paper"></a>
<a href="https://ego2sim.xyz"><img src="https://img.shields.io/badge/Ego2Sim-project_page-blue" alt="Project page"></a>
<a href="https://www.youtube.com/watch?v=d8mcdGW2FAk"><img src="https://img.shields.io/badge/Ego2Sim-video-purple" alt="Video"></a>

Aidan Gildea, Sunwoong Choi

[Sensing and Robotics for Infrastructure (SRI) Lab](https://sri-lab.seas.ucla.edu/), University of California, Los Angeles

[Sim2Real2Field Workshop](https://sim2real2field-corl26.github.io/) at CoRL 2026

<p>
  <a href="https://www.ucla.edu"><picture><source media="(prefers-color-scheme: dark)" srcset="https://github.com/Aidan-Gildea/ego2sim/raw/main/media/ucla_logo_white.png"><img src="media/ucla_logo_blue.png" alt="UCLA" height="50"></picture></a>
  &nbsp;&nbsp;&nbsp;&nbsp;
  <a href="https://www.corl.org"><img src="media/corl2026_logo.png" alt="CoRL 2026" height="50"></a>
</p>

</div>

<p align="center"><img src="media/teaser.png" alt="An egocentric walk becomes a hybrid scene of a Gaussian splat and a collider, in which Spot robots make contact with a car and a hedge" width="100%"></p>

## Overview


Ego2Sim converts data from a single capture using Meta's [Project Aria Gen 2](https://www.projectaria.com/) glasses into a scene that a robot can walk through in NVIDIA Isaac Sim. Each scene is composed of two parts:

- **A Gaussian splat for appearance**, trained on the glasses' RGB video.
- **A collision mesh for contact**, built from the glasses' own SLAM points, which come from Aria's Machine Perception Services (MPS).

Both come out in the same metric & gravity-aligned frame, so they line up with no registration step, and they are packed into a single `.usdz` file. Against same-day iPad LiDAR scans, the collision mesh reaches an F-score of 0.84–0.94 at 10 cm within 1 m of the ground, and it takes 51–160× less time to build than a multi-view stereo mesh. The frame extraction and splat training come from Meta's [egocentric_splats](https://github.com/facebookresearch/egocentric_splats).

## Quick start

System Requirements:
- **Linux with an NVIDIA GPU.** Recommended 16+ GB VRAM
- **Python 3.10 with CUDA PyTorch 2.6 or newer**, plus the CUDA toolkit (`nvcc`) of the same CUDA version.
- **[COLMAP](https://colmap.github.io/install.html) on your `PATH`**, built with CGAL, which the collider needs. The conda-forge and `apt` packages have it.
- **Isaac Sim 5.1** to open the result.

<details>
<summary><b>Setting up the environment</b></summary>

`nvidia-smi` shows the newest CUDA version your driver supports, in the top right. The commands below use CUDA 12.8. If yours shows lower, change `12.8.1` to `12.6.3` and `cu128` to `cu126`.

```bash
nvidia-smi
conda create -n ego2sim python=3.10 -y
conda activate ego2sim
conda install -y -c "nvidia/label/cuda-12.8.1" cuda-toolkit
conda env config vars set CUDA_HOME="$CONDA_PREFIX"
conda activate ego2sim
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
conda install -y -c conda-forge colmap
```

`nvcc` should show the same CUDA version as PyTorch, and `colmap delaunay_mesher -h` should list its options, not a CGAL error.

```bash
nvcc --version
python -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())"
colmap delaunay_mesher -h
```

</details>

```bash
git clone https://github.com/Aidan-Gildea/ego2sim.git
cd ego2sim
pip install -r requirements.txt
```

## Run on your own recording

### 1. Get the MPS output

[Machine Perception Services](https://facebookresearch.github.io/projectaria_tools/docs/ARK/mps) gives the recording's SLAM trajectory and points. `aria_mps` asks you to sign in with your Project Aria account, uploads the recording and downloads the result next to it. Install it with `pipx`, because its pinned packages clash with the training packages.

```bash
pipx install projectaria-mps
aria_mps single -i ~/recordings/Outside_20260812_141244.vrs --features SLAM
```

The pipeline looks for the output in `mps_<recording>_vrs/slam/` next to the `.vrs`. If yours is somewhere else, set `MPS_FOLDER` to its `slam/` folder. [Sample recordings](https://drive.google.com/drive/folders/1INGZFiCD6qYtHSzbXo4Qegmj-flwikv0) with their MPS output are on Google Drive.

The defaults assume recording profile `profile10` (RGB 2016×1512 at 30 Hz). For any other profile, see [tools/README.md](tools/README.md#debug).

### 2. Run the pipeline

```bash
bash run_ego2sim.sh ~/recordings/Outside_20260812_141244.vrs
```

A first run takes a few hours: extracting the frames and training the splat each take one to three hours, and the collision mesh takes a few minutes. If you stop and rerun it, stages that already finished are skipped. A stopped training run starts over.

| Stage | Script | What it makes |
| --- | --- | --- |
| 1.&nbsp;Frames | `frames/run_gen2_outside.sh` | Undistorted RGB frames, each with its camera pose from MPS |
| 2.&nbsp;Collider | `mesh/mps-mesh/run_mps_mesh.sh` | A mesh from the MPS points, simplified for collisions |
| 3.&nbsp;Splat | `splat/train_gen2_outside.sh` | The Gaussian splat, trained for 30,000 steps |
| 4.&nbsp;Scene | `isaacsim/splat_and_mesh_to_usdz.py` | One `.usdz` with the visible splat and the invisible collider |

```text
~/recordings/processed/<recording>/camera-rgb-rectified-1008-h1512/   1. frames
ego2sim/output/
├── mps-mesh/<recording>/camera-rgb-rectified-1008-h1512/
│   ├── mesh_delaunay_decimated.ply                                   2. collider
│   └── report_decimated.md                                           collider score
└── <recording>/camera-rgb-rectified-1008-h1512/
    ├── point_cloud/iteration_30000/point_cloud.ply                   3. splat
    ├── test_logs.json                                                splat score
    └── isaacsim/<recording>_with_collider.usdz                       4. scene
```

`report_decimated.md` checks the collider without a reference scan: how much of the walked floor it covers, its largest hole and its floor height error. `test_logs.json` scores the splat on frames held out from training.

### 3. Open it in Isaac Sim

Open the `.usdz` with File > Open and press Play to turn on collisions. The splat is what you see; the mesh is invisible and only collides.

<p align="center"><img src="media/isaacsim_drive.gif" alt="Spot walking into a parked car in an Ego2Sim scene in Isaac Sim" width="100%"></p>

<details>
<summary><b>Settings</b></summary>

Training settings are in `splat/conf/config.yaml`, with a note on each. The ones you are most likely to change are `opt.iterations`, `opt.mcmc_strategy.cap_max` (most Gaussians, which sets GPU memory), `opt.handle_rolling_shutter`, `opt.depth_loss` and `scene.train_split`. The shell scripts pass training only the input and output paths, so the YAML is what runs.

The other settings are at the top of `run_ego2sim.sh`. Edit them there, or set them before the command, for example `MESH_SOURCE=mvs bash run_ego2sim.sh ...`.

| Setting | Default | Effect |
| --- | --- | --- |
| `MESH_SOURCE` | `mps` | `mvs` builds the collider with COLMAP multi-view stereo instead. That takes hours and fills outdoor floors only slightly better. Known bug: frames get the wrong poses when their timestamps change length partway through a recording |
| `DECIMATE` | `1` | `0` uses the full mesh as the collider |
| `EYE_HEIGHT` | `1.6683` | Height of the glasses above the floor in metres. Only used to score the mesh |
| `RGB_FOCAL`, `RGB_HEIGHT` | `1008`, `1512` | Rectified frame size. Change for recording profiles other than `profile10` |

A changed setting only affects stages that have not run yet. To redo the mesh, add `FORCE=1`. To retrain the splat, delete `output/<recording>/<rect>/point_cloud/`.

</details>

## Visualize the splat

To look at a trained splat on its own in a browser (port 8080):

```bash
python splat/launch_viewer.py model_root=output/<recording>
```

## Recording tips

- Walk through the scene. Turning in place gives no depth.
- Record for 2 to 5 minutes, and pass earlier spots again so MPS can close loops.
- Turn your head slowly and avoid dim places. Indoors the camera exposure is long, so fast motion blurs the frames.

## Repository layout

| Path | What it does |
| --- | --- |
| `run_ego2sim.sh` | Runs the whole pipeline and holds the frame, mesh and scene settings |
| `frames/` | Stage 1: reads the `.vrs` and the MPS output, writes rectified, posed frames |
| `mesh/mps-mesh/` | Stage 2: collision mesh from the MPS points |
| `mesh/photogrammetry/` | Mesh scoring and simplification used by stage 2, and the optional multi-view stereo mesh |
| `splat/` | Stage 3: splat training, rendering and the viewer (from egocentric_splats). Every training setting is in `splat/conf/config.yaml` |
| `isaacsim/` | Stage 4: the Isaac Sim scene |
| `tools/` | Checks for recordings and setups, and the paper's evaluation against iPad LiDAR scans |
| `media/` | Images for this README |

The code used for the paper is tagged [`srf-2026`](https://github.com/Aidan-Gildea/ego2sim/tree/srf-2026).

## Citation

```bibtex
@inproceedings{gildea2026ego2sim,
    title={Ego2Sim: Photorealistic Contact-Enabled Simulation from Egocentric Data},
    author={Gildea, Aidan and Choi, Sunwoong},
    booktitle={Sim2Real2Field Workshop at the Conference on Robot Learning (CoRL)},
    year={2026}
}
```

Ego2Sim builds on egocentric_splats:

```bibtex
@inproceedings{lv2025egosplats,
    title={Photoreal Scene Reconstruction from an Egocentric Device},
    author={Lv, Zhaoyang and Monge, Maurizio and Chen, Ka and Zhu, Yufeng and Goesele, Michael and Engel, Jakob and Dong, Zhao and Newcombe, Richard},
    booktitle={ACM SIGGRAPH},
    year={2025}
}
```

## License

[CC BY-NC 4.0](LICENSE), inherited from [egocentric_splats](https://github.com/facebookresearch/egocentric_splats). Ego2Sim also builds on:

* [COLMAP](https://colmap.github.io/) for meshing, BSD
* [gsplat](https://github.com/nerfstudio-project/gsplat) for splat training, Apache 2.0
* [Project Aria Tools](https://github.com/facebookresearch/projectaria_tools) for reading Aria recordings, Apache 2.0
* [3dgrut](https://github.com/nv-tlabs/3dgrut), whose USDZ writer is adapted here, Apache 2.0

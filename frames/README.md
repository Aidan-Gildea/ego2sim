# frames

Stage 1 of the pipeline: reads the `.vrs` recording and its MPS output, and writes undistorted RGB frames with a camera pose for each, plus sparse depth.

```
frames/
├── run_gen2_outside.sh   # What run_ego2sim.sh runs for this stage
├── extract_aria_vrs.py   # Frame extraction, from egocentric_splats
├── aria_utils.py         # Aria calibration and undistortion helpers
└── data/                 # Lens-shading images for Aria Gen 1
```

The frames are written to `<recordings>/processed/<recording>/camera-rgb-rectified-1008-h1512/`.

## Recording tips

- Walk through the scene. Turning in place gives no depth.
- Record for 2 to 5 minutes, and pass earlier spots again so MPS can close loops.
- Turn your head slowly and avoid dim places. Indoors the exposure is long, so fast motion blurs the frames.

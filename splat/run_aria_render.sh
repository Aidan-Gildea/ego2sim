#!/bin/bash

# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

# SCENE=<recording> RECT=camera-rgb-rectified-1008-h1512 bash splat/run_aria_render.sh

# trained model folder, with point_cloud/ and cameras.json; output/$SCENE/$RECT unless RESULTS_FOLDER is set
results_folder="${RESULTS_FOLDER:-output/${SCENE:?set RESULTS_FOLDER, or SCENE and RECT}/${RECT:?set RESULTS_FOLDER, or SCENE and RECT}}"

gain_amplify=${GAIN_AMPLIFY:-1}  # analog gain multiplier, relative to the original video
render_fps=${RENDER_FPS:-10}     # output video fps; poses are interpolated between keyframes

# PLY of the highest iteration
ply_file=$(ls "$results_folder"/point_cloud/iteration_*/point_cloud.ply | sort -V | tail -n 1)
json_file=$results_folder/cameras.json
render_output=$results_folder/traj_render_fps_"$render_fps"_gain_"$gain_amplify"

python splat/render_lightning.py \
    scene.load_ply="$ply_file" \
    render.render_json="$json_file" \
    render.render_output="$render_output" \
    render.render_fps=$render_fps \
    render.gain_amplify=$gain_amplify

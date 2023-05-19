#pragma once

#include "common.h"
#include "vec.h"
#include "quat.h"
#include "array.h"
#include "nnet.h"

// This function uses the decompressor network
// to generate the pose of the character. It 
// requires as input the feature values and latent 
// values as well as a current root position and 
// rotation.
void decompressor_evaluate(
    slice1d<vec3> bone_positions,
    slice1d<vec3> bone_velocities,
    slice1d<quat> bone_rotations,
    slice1d<vec3> bone_angular_velocities,
    slice1d<bool> bone_contacts,
    nnet_evaluation& evaluation,
    const slice1d<float> features,
    const slice1d<float> latent,
    const vec3 root_position,
    const quat root_rotation,
    const nnet& nn,
    const float dt = 1.0f / 60.0f);

// This function updates the feature and latent values
// using the stepper network and a given dt.
void stepper_evaluate(
    slice1d<float> features,
    slice1d<float> latent,
    nnet_evaluation& evaluation,
    const nnet& nn,
    const float dt = 1.0f / 60.0f);

// This function projects a set of feature values onto
// the nearest in the trained database, also outputting the 
// associated latent values. It also produces the matching 
// cost using the distance of the projection, and detects 
// transitions for a given transition cost by measuring the 
// distance between the projected result and the current
// feature values
void projector_evaluate(
    bool& transition,
    float& best_cost,
    slice1d<float> proj_features,
    slice1d<float> proj_latent,
    nnet_evaluation& evaluation,
    const slice1d<float> query,
    const slice1d<float> features_offset,
    const slice1d<float> features_scale,
    const slice1d<float> curr_features,
    const nnet& nn,
    const float transition_cost = 0.0f);

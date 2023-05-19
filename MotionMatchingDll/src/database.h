#pragma once

#include "common.h"
#include "vec.h"
#include "quat.h"
#include "array.h"

#include <assert.h>
#include <float.h>
#include <stdio.h>
#include <math.h>

//--------------------------------------

enum
{
    BOUND_SM_SIZE = 16,
    BOUND_LR_SIZE = 64,
};

struct database
{
    array2d<vec3> bone_positions;
    array2d<vec3> bone_velocities;
    array2d<quat> bone_rotations;
    array2d<vec3> bone_angular_velocities;
    array1d<int> bone_parents;
    
    array1d<int> range_starts;
    array1d<int> range_stops;
    
    array2d<float> features;
    array1d<float> features_offset;
    array1d<float> features_scale;
    
    array2d<bool> contact_states;
    
    array2d<float> bound_sm_min;
    array2d<float> bound_sm_max;
    array2d<float> bound_lr_min;
    array2d<float> bound_lr_max;
    
    int nframes() const { return bone_positions.rows; }
    int nbones() const { return bone_positions.cols; }
    int nranges() const { return range_starts.size; }
    int nfeatures() const { return features.cols; }
    int ncontacts() const { return contact_states.cols; }
};

void database_load(database& db, const char* filename);

void database_save_matching_features(const database& db, const char* filename);

// When we add an offset to a frame in the database there is a chance
// it will go out of the relevant range so here we can clamp it to 
// the last frame of that range.
int database_trajectory_index_clamp(database& db, int frame, int offset);

//--------------------------------------

void normalize_feature(
    slice2d<float> features,
    slice1d<float> features_offset,
    slice1d<float> features_scale,
    const int offset, 
    const int size, 
    const float weight = 1.0f);

void denormalize_features(
    slice1d<float> features,
    const slice1d<float> features_offset,
    const slice1d<float> features_scale);

//--------------------------------------

// Here I am using a simple recursive version of forward kinematics
void forward_kinematics(
    vec3& bone_position,
    quat& bone_rotation,
    const slice1d<vec3> bone_positions,
    const slice1d<quat> bone_rotations,
    const slice1d<int> bone_parents,
    const int bone);

// Forward kinematics but also compute the velocities
void forward_kinematics_velocity(
    vec3& bone_position,
    vec3& bone_velocity,
    quat& bone_rotation,
    vec3& bone_angular_velocity,
    const slice1d<vec3> bone_positions,
    const slice1d<vec3> bone_velocities,
    const slice1d<quat> bone_rotations,
    const slice1d<vec3> bone_angular_velocities,
    const slice1d<int> bone_parents,
    const int bone);

// Compute forward kinematics for all joints
void forward_kinematics_full(
    slice1d<vec3> global_bone_positions,
    slice1d<quat> global_bone_rotations,
    const slice1d<vec3> local_bone_positions,
    const slice1d<quat> local_bone_rotations,
    const slice1d<int> bone_parents);

// Compute forward kinematics of just some joints using a
// mask to indicate which joints are already computed
void forward_kinematics_partial(
    slice1d<vec3> global_bone_positions,
    slice1d<quat> global_bone_rotations,
    slice1d<bool> global_bone_computed,
    const slice1d<vec3> local_bone_positions,
    const slice1d<quat> local_bone_rotations,
    const slice1d<int> bone_parents,
    int bone);

// Same but including velocity
void forward_kinematics_velocity_partial(
    slice1d<vec3> global_bone_positions,
    slice1d<vec3> global_bone_velocities,
    slice1d<quat> global_bone_rotations,
    slice1d<vec3> global_bone_angular_velocities,
    slice1d<bool> global_bone_computed,
    const slice1d<vec3> local_bone_positions,
    const slice1d<vec3> local_bone_velocities,
    const slice1d<quat> local_bone_rotations,
    const slice1d<vec3> local_bone_angular_velocities,
    const slice1d<int> bone_parents,
    int bone);

//--------------------------------------

// Compute a feature for the position of a bone relative to the simulation/root bone
void compute_bone_position_feature(database& db, int& offset, int bone, float weight = 1.0f);

// Similar but for a bone's velocity
void compute_bone_velocity_feature(database& db, int& offset, int bone, float weight = 1.0f);

// Compute the trajectory at 20, 40, and 60 frames in the future
void compute_trajectory_position_feature(database& db, int& offset, float weight = 1.0f);

// Same for direction
void compute_trajectory_direction_feature(database& db, int& offset, float weight = 1.0f);

// Build the Motion Matching search acceleration structure. Here we
// just use axis aligned bounding boxes regularly spaced at BOUND_SM_SIZE
// and BOUND_LR_SIZE frames
void database_build_bounds(database& db);

// Build all motion matching features and acceleration structure
void database_build_matching_features(
    database& db,
    const float feature_weight_foot_position,
    const float feature_weight_foot_velocity,
    const float feature_weight_hip_velocity,
    const float feature_weight_trajectory_positions,
    const float feature_weight_trajectory_directions);

// Motion Matching search function essentially consists
// of comparing every feature vector in the database, 
// against the query feature vector, first checking the 
// query distance to the axis aligned bounding boxes used 
// for the acceleration structure.
void motion_matching_search(
    int& best_index, 
    float& best_cost, 
    const slice1d<int> range_starts,
    const slice1d<int> range_stops,
    const slice2d<float> features,
    const slice1d<float> features_offset,
    const slice1d<float> features_scale,
    const slice2d<float> bound_sm_min,
    const slice2d<float> bound_sm_max,
    const slice2d<float> bound_lr_min,
    const slice2d<float> bound_lr_max,
    const slice1d<float> query_normalized,
    const float transition_cost,
    const int ignore_range_end,
    const int ignore_surrounding);

// Search database
void database_search(
    int& best_index, 
    float& best_cost, 
    const database& db, 
    const slice1d<float> query,
    const float transition_cost = 0.0f,
    const int ignore_range_end = 20,
    const int ignore_surrounding = 20);

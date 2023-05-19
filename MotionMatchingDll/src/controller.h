#pragma once

#include "common.h"
#include "vec.h"
#include "quat.h"
#include "spring.h"
#include "array.h"
#include "character.h"
#include "database.h"
#include "nnet.h"
#include "lmm.h"

class controller
{
public:
    
    // Character
    character character_data;    
    // Matching Database    
    database db;
    
    // Camera    
    float camera_azimuth = 0.0f;
    float camera_altitude = 0.4f;
    float camera_distance = 4.0f;

    
    // Scene Obstacles
    array1d<vec3> obstacles_positions;
    array1d<vec3> obstacles_scales;
    
    float feature_weight_foot_position = 0.75f;
    float feature_weight_foot_velocity = 1.0f;
    float feature_weight_hip_velocity = 1.0f;
    float feature_weight_trajectory_positions = 1.0f;
    float feature_weight_trajectory_directions = 1.5f;
    
    int frame_index = 0;
    float inertialize_blending_halflife = 0.1f;

    // Pose Data
    array1d<vec3> curr_bone_positions;
    array1d<vec3> curr_bone_velocities;
    array1d<quat> curr_bone_rotations;
    array1d<vec3> curr_bone_angular_velocities;
    array1d<bool> curr_bone_contacts;

    array1d<vec3> trns_bone_positions;
    array1d<vec3> trns_bone_velocities;
    array1d<quat> trns_bone_rotations;
    array1d<vec3> trns_bone_angular_velocities;
    array1d<bool> trns_bone_contacts;

    array1d<vec3> bone_positions;
    array1d<vec3> bone_velocities;
    array1d<quat> bone_rotations;
    array1d<vec3> bone_angular_velocities;
    
    array1d<vec3> bone_offset_positions;
    array1d<vec3> bone_offset_velocities;
    array1d<quat> bone_offset_rotations;
    array1d<vec3> bone_offset_angular_velocities;
    
    array1d<vec3> global_bone_positions;
    array1d<vec3> global_bone_velocities;
    array1d<quat> global_bone_rotations;
    array1d<vec3> global_bone_angular_velocities;
    array1d<bool> global_bone_computed;
    
    vec3 transition_src_position;
    quat transition_src_rotation;
    vec3 transition_dst_position;
    quat transition_dst_rotation;
    
    // Trajectory & Gameplay Data
    
    float search_time = 0.1f;
    float search_timer = search_time;
    float force_search_timer = search_time;
    
    vec3 desired_velocity;
    vec3 desired_velocity_change_curr;
    vec3 desired_velocity_change_prev;
    float desired_velocity_change_threshold = 50.0;
    
    quat desired_rotation;
    vec3 desired_rotation_change_curr;
    vec3 desired_rotation_change_prev;
    float desired_rotation_change_threshold = 50.0;
    
    float desired_gait = 0.0f;
    float desired_gait_velocity = 0.0f;
    
    vec3 simulation_position;
    vec3 simulation_velocity;
    vec3 simulation_acceleration;
    quat simulation_rotation;
    vec3 simulation_angular_velocity;
    
    float simulation_velocity_halflife = 0.27f;
    float simulation_rotation_halflife = 0.27f;
    
    // All speeds in m/s
    float simulation_run_fwrd_speed = 4.0f;
    float simulation_run_side_speed = 3.0f;
    float simulation_run_back_speed = 2.5f;
    
    float simulation_walk_fwrd_speed = 1.75f;
    float simulation_walk_side_speed = 1.5f;
    float simulation_walk_back_speed = 1.25f;
    
    array1d<vec3> trajectory_desired_velocities;
    array1d<quat> trajectory_desired_rotations;
    array1d<vec3> trajectory_positions;
    array1d<vec3> trajectory_velocities;
    array1d<vec3> trajectory_accelerations;
    array1d<quat> trajectory_rotations;
    array1d<vec3> trajectory_angular_velocities;
    
    // Synchronization
    
    bool synchronization_enabled = false;
    float synchronization_data_factor = 1.0f;
    
    // Adjustment
    
    bool adjustment_enabled = true;
    bool adjustment_by_velocity_enabled = true;
    float adjustment_position_halflife = 0.1f;
    float adjustment_rotation_halflife = 0.2f;
    float adjustment_position_max_ratio = 0.5f;
    float adjustment_rotation_max_ratio = 0.5f;
    
    // Clamping
    
    bool clamping_enabled = true;
    float clamping_max_distance = 0.15f;
    float clamping_max_angle = 0.5f * PIf;
    
    // IK
    
    bool ik_enabled = true;
    bool ik_enabled_prev = false;
    float ik_max_length_buffer = 0.015f;
    float ik_foot_height = 0.02f;
    float ik_toe_length = 0.15f;
    float ik_unlock_radius = 0.2f;
    float ik_blending_halflife = 0.1f;
    
    // Contact and Foot Locking data
    
    array1d<int> contact_bones;    
    array1d<vec3> adjusted_bone_positions;
    array1d<quat> adjusted_bone_rotations;

    array1d<bool> contact_states;
    array1d<bool> contact_locks;
    array1d<vec3> contact_positions;
    array1d<vec3> contact_velocities;
    array1d<vec3> contact_points;
    array1d<vec3> contact_targets;
    array1d<vec3> contact_offset_positions;
    array1d<vec3> contact_offset_velocities;
    
    
    // Learned Motion Matching
    
    bool lmm_enabled = false;
    nnet decompressor, stepper, projector;    
    nnet_evaluation decompressor_evaluation, stepper_evaluation, projector_evaluation;

    array1d<float> features_proj;
    array1d<float> features_curr;
    array1d<float> latent_proj;
    array1d<float> latent_curr;

    vec3 camera_data;
    vec3 movement_data;
    bool strafe_data = false;
public:
    void rebuild_database();
    bool init(char* path);

    void update(float dt);

    
    void setCameraData(vec3 v) { camera_data = v; }
    void move(vec3 v) { movement_data = v; }
    
};

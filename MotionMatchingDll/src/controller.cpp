
#include "controller.h"


#include <initializer_list>
#include <functional>
#include <string>


//--------------------------------------

float orbit_camera_update_azimuth(
    const float azimuth, 
    const vec3 gamepadstick_right,
    const bool desired_strafe,
    const float dt)
{
    vec3 gamepadaxis = desired_strafe ? vec3() : gamepadstick_right;
    return azimuth + 2.0f * dt * -gamepadaxis.x;
}

float orbit_camera_update_altitude(
    const float altitude, 
    const vec3 gamepadstick_right,
    const bool desired_strafe,
    const float dt)
{
    vec3 gamepadaxis = desired_strafe ? vec3() : gamepadstick_right;
    return clampf(altitude + 2.0f * dt * gamepadaxis.z, 0.0, 0.4f * PIf);
}

float orbit_camera_update_distance(
    const float distance, 
    const float dt)
{
    float gamepadzoom = 0.0f;
        // IsGamepadButtonDown(GAMEPAD_PLAYER, GAMEPAD_BUTTON_LEFT_TRIGGER_1)  ? +1.0f :
        // IsGamepadButtonDown(GAMEPAD_PLAYER, GAMEPAD_BUTTON_RIGHT_TRIGGER_1) ? -1.0f : 0.0f;
        
    return clampf(distance +  10.0f * dt * gamepadzoom, 0.1f, 100.0f);
}

// Updates the camera using the orbit cam controls
void orbit_camera_update(
    float& camera_azimuth,
    float& camera_altitude,
    float& camera_distance,
    const vec3 gamepadstick_right,
    const bool desired_strafe,
    const float dt)
{
    camera_azimuth = orbit_camera_update_azimuth(camera_azimuth, gamepadstick_right, desired_strafe, dt);
    camera_altitude = orbit_camera_update_altitude(camera_altitude, gamepadstick_right, desired_strafe, dt);
    camera_distance = orbit_camera_update_distance(camera_distance, dt);
}

void desired_gait_update(
    float& desired_gait, 
    float& desired_gait_velocity,
    const float dt,
    const float gait_change_halflife = 0.1f,
    const float x_goal = 0.0f)
{
    simple_spring_damper_exact(
        desired_gait, 
        desired_gait_velocity,
        x_goal,
        gait_change_halflife,
        dt);
}

vec3 desired_velocity_update(
    const vec3 gamepadstick_left,
    const float camera_azimuth,
    const quat simulation_rotation,
    const float fwrd_speed,
    const float side_speed,
    const float back_speed)
{
    // Find stick position in world space by rotating using camera azimuth
    vec3 global_stick_direction = quat_mul_vec3(
        quat_from_angle_axis(camera_azimuth, vec3(0, 1, 0)), gamepadstick_left);
    
    // Find stick position local to current facing direction
    vec3 local_stick_direction = quat_inv_mul_vec3(
        simulation_rotation, global_stick_direction);
    
    // Scale stick by forward, sideways and backwards speeds
    vec3 local_desired_velocity = local_stick_direction.z > 0.0 ?
        vec3(side_speed, 0.0f, fwrd_speed) * local_stick_direction :
        vec3(side_speed, 0.0f, back_speed) * local_stick_direction;
    
    // Re-orientate into the world space
    return quat_mul_vec3(simulation_rotation, local_desired_velocity);
}

quat desired_rotation_update(
    const quat desired_rotation,
    const vec3 gamepadstick_left,
    const vec3 gamepadstick_right,
    const float camera_azimuth,
    const bool desired_strafe,
    const vec3 desired_velocity)
{
    quat desired_rotation_curr = desired_rotation;
    
    // If strafe is active then desired direction is coming from right
    // stick as long as that stick is being used, otherwise we assume
    // forward facing
    if (desired_strafe)
    {
        vec3 desired_direction = quat_mul_vec3(quat_from_angle_axis(camera_azimuth, vec3(0, 1, 0)), vec3(0, 0, -1));

        if (length(gamepadstick_right) > 0.01f)
        {
            desired_direction = quat_mul_vec3(quat_from_angle_axis(camera_azimuth, vec3(0, 1, 0)), normalize(gamepadstick_right));
        }
        
        return quat_from_angle_axis(atan2f(desired_direction.x, desired_direction.z), vec3(0, 1, 0));            
    }
    
    // If strafe is not active the desired direction comes from the left 
    // stick as long as that stick is being used
    else if (length(gamepadstick_left) > 0.01f)
    {
        
        vec3 desired_direction = normalize(desired_velocity);
        return quat_from_angle_axis(atan2f(desired_direction.x, desired_direction.z), vec3(0, 1, 0));
    }
    
    // Otherwise desired direction remains the same
    else
    {
        return desired_rotation_curr;
    }
}

//--------------------------------------

// Moving the root is a little bit difficult when we have the
// inertializer set up in the way we do. Essentially we need
// to also make sure to adjust all of the locations where 
// we are transforming the data to and from as well as the 
// offsets being blended out
void inertialize_root_adjust(
    vec3& offset_position,
    vec3& transition_src_position,
    quat& transition_src_rotation,
    vec3& transition_dst_position,
    quat& transition_dst_rotation,
    vec3& position,
    quat& rotation,
    const vec3 input_position,
    const quat input_rotation)
{
    // Find the position difference and add it to the state and transition location
    vec3 position_difference = input_position - position;
    position = position_difference + position;
    transition_dst_position = position_difference + transition_dst_position;
    
    // Find the point at which we want to now transition from in the src data
    transition_src_position = transition_src_position + quat_mul_vec3(transition_src_rotation,
        quat_inv_mul_vec3(transition_dst_rotation, position - offset_position - transition_dst_position));
    transition_dst_position = position;
    offset_position = vec3();
    
    // Find the rotation difference. We need to normalize here or some error can accumulate 
    // over time during adjustment.
    quat rotation_difference = quat_normalize(quat_mul_inv(input_rotation, rotation));
    
    // Apply the rotation difference to the current rotation and transition location
    rotation = quat_mul(rotation_difference, rotation);
    transition_dst_rotation = quat_mul(rotation_difference, transition_dst_rotation);
}

void inertialize_pose_reset(
    slice1d<vec3> bone_offset_positions,
    slice1d<vec3> bone_offset_velocities,
    slice1d<quat> bone_offset_rotations,
    slice1d<vec3> bone_offset_angular_velocities,
    vec3& transition_src_position,
    quat& transition_src_rotation,
    vec3& transition_dst_position,
    quat& transition_dst_rotation,
    const vec3 root_position,
    const quat root_rotation)
{
    bone_offset_positions.zero();
    bone_offset_velocities.zero();
    bone_offset_rotations.set(quat());
    bone_offset_angular_velocities.zero();
    
    transition_src_position = root_position;
    transition_src_rotation = root_rotation;
    transition_dst_position = vec3();
    transition_dst_rotation = quat();
}

// This function transitions the inertializer for 
// the full character. It takes as input the current 
// offsets, as well as the root transition locations,
// current root state, and the full pose information 
// for the pose being transitioned from (src) as well 
// as the pose being transitioned to (dst) in their
// own animation spaces.
void inertialize_pose_transition(
    slice1d<vec3> bone_offset_positions,
    slice1d<vec3> bone_offset_velocities,
    slice1d<quat> bone_offset_rotations,
    slice1d<vec3> bone_offset_angular_velocities,
    vec3& transition_src_position,
    quat& transition_src_rotation,
    vec3& transition_dst_position,
    quat& transition_dst_rotation,
    const vec3 root_position,
    const vec3 root_velocity,
    const quat root_rotation,
    const vec3 root_angular_velocity,
    const slice1d<vec3> bone_src_positions,
    const slice1d<vec3> bone_src_velocities,
    const slice1d<quat> bone_src_rotations,
    const slice1d<vec3> bone_src_angular_velocities,
    const slice1d<vec3> bone_dst_positions,
    const slice1d<vec3> bone_dst_velocities,
    const slice1d<quat> bone_dst_rotations,
    const slice1d<vec3> bone_dst_angular_velocities)
{
    // First we record the root position and rotation
    // in the animation data for the source and destination
    // animation
    transition_dst_position = root_position;
    transition_dst_rotation = root_rotation;
    transition_src_position = bone_dst_positions(0);
    transition_src_rotation = bone_dst_rotations(0);
    
    // We then find the velocities so we can transition the 
    // root inertiaizers
    vec3 world_space_dst_velocity = quat_mul_vec3(transition_dst_rotation, 
        quat_inv_mul_vec3(transition_src_rotation, bone_dst_velocities(0)));
    
    vec3 world_space_dst_angular_velocity = quat_mul_vec3(transition_dst_rotation, 
        quat_inv_mul_vec3(transition_src_rotation, bone_dst_angular_velocities(0)));
    
    // Transition inertializers recording the offsets for 
    // the root joint
    inertialize_transition(
        bone_offset_positions(0),
        bone_offset_velocities(0),
        root_position,
        root_velocity,
        root_position,
        world_space_dst_velocity);
        
    inertialize_transition(
        bone_offset_rotations(0),
        bone_offset_angular_velocities(0),
        root_rotation,
        root_angular_velocity,
        root_rotation,
        world_space_dst_angular_velocity);
    
    // Transition all the inertializers for each other bone
    for (int i = 1; i < bone_offset_positions.size; i++)
    {
        inertialize_transition(
            bone_offset_positions(i),
            bone_offset_velocities(i),
            bone_src_positions(i),
            bone_src_velocities(i),
            bone_dst_positions(i),
            bone_dst_velocities(i));
            
        inertialize_transition(
            bone_offset_rotations(i),
            bone_offset_angular_velocities(i),
            bone_src_rotations(i),
            bone_src_angular_velocities(i),
            bone_dst_rotations(i),
            bone_dst_angular_velocities(i));
    }
}

// This function updates the inertializer states. Here 
// it outputs the smoothed animation (input plus offset) 
// as well as updating the offsets themselves. It takes 
// as input the current playing animation as well as the 
// root transition locations, a halflife, and a dt
void inertialize_pose_update(
    slice1d<vec3> bone_positions,
    slice1d<vec3> bone_velocities,
    slice1d<quat> bone_rotations,
    slice1d<vec3> bone_angular_velocities,
    slice1d<vec3> bone_offset_positions,
    slice1d<vec3> bone_offset_velocities,
    slice1d<quat> bone_offset_rotations,
    slice1d<vec3> bone_offset_angular_velocities,
    const slice1d<vec3> bone_input_positions,
    const slice1d<vec3> bone_input_velocities,
    const slice1d<quat> bone_input_rotations,
    const slice1d<vec3> bone_input_angular_velocities,
    const vec3 transition_src_position,
    const quat transition_src_rotation,
    const vec3 transition_dst_position,
    const quat transition_dst_rotation,
    const float halflife,
    const float dt)
{
    // First we find the next root position, velocity, rotation
    // and rotational velocity in the world space by transforming 
    // the input animation from it's animation space into the 
    // space of the currently playing animation.
    vec3 world_space_position = quat_mul_vec3(transition_dst_rotation, 
        quat_inv_mul_vec3(transition_src_rotation, 
            bone_input_positions(0) - transition_src_position)) + transition_dst_position;
    
    vec3 world_space_velocity = quat_mul_vec3(transition_dst_rotation, 
        quat_inv_mul_vec3(transition_src_rotation, bone_input_velocities(0)));
    
    // Normalize here because quat inv mul can sometimes produce 
    // unstable returns when the two rotations are very close.
    quat world_space_rotation = quat_normalize(quat_mul(transition_dst_rotation, 
        quat_inv_mul(transition_src_rotation, bone_input_rotations(0))));
    
    vec3 world_space_angular_velocity = quat_mul_vec3(transition_dst_rotation, 
        quat_inv_mul_vec3(transition_src_rotation, bone_input_angular_velocities(0)));
    
    // Then we update these two inertializers with these new world space inputs
    inertialize_update(
        bone_positions(0),
        bone_velocities(0),
        bone_offset_positions(0),
        bone_offset_velocities(0),
        world_space_position,
        world_space_velocity,
        halflife,
        dt);
        
    inertialize_update(
        bone_rotations(0),
        bone_angular_velocities(0),
        bone_offset_rotations(0),
        bone_offset_angular_velocities(0),
        world_space_rotation,
        world_space_angular_velocity,
        halflife,
        dt);        
    
    // Then we update the inertializers for the rest of the bones
    for (int i = 1; i < bone_positions.size; i++)
    {
        inertialize_update(
            bone_positions(i),
            bone_velocities(i),
            bone_offset_positions(i),
            bone_offset_velocities(i),
            bone_input_positions(i),
            bone_input_velocities(i),
            halflife,
            dt);
            
        inertialize_update(
            bone_rotations(i),
            bone_angular_velocities(i),
            bone_offset_rotations(i),
            bone_offset_angular_velocities(i),
            bone_input_rotations(i),
            bone_input_angular_velocities(i),
            halflife,
            dt);
    }
}

//--------------------------------------

// Copy a part of a feature vector from the 
// matching database into the query feature vector
void query_copy_denormalized_feature(
    slice1d<float> query, 
    int& offset, 
    const int size, 
    const slice1d<float> features,
    const slice1d<float> features_offset,
    const slice1d<float> features_scale)
{
    for (int i = 0; i < size; i++)
    {
        query(offset + i) = features(offset + i) * features_scale(offset + i) + features_offset(offset + i);
    }
    
    offset += size;
}

// Compute the query feature vector for the current 
// trajectory controlled by the gamepad.
void query_compute_trajectory_position_feature(
    slice1d<float> query, 
    int& offset, 
    const vec3 root_position, 
    const quat root_rotation, 
    const slice1d<vec3> trajectory_positions)
{
    vec3 traj0 = quat_inv_mul_vec3(root_rotation, trajectory_positions(1) - root_position);
    vec3 traj1 = quat_inv_mul_vec3(root_rotation, trajectory_positions(2) - root_position);
    vec3 traj2 = quat_inv_mul_vec3(root_rotation, trajectory_positions(3) - root_position);
    
    query(offset + 0) = traj0.x;
    query(offset + 1) = traj0.z;
    query(offset + 2) = traj1.x;
    query(offset + 3) = traj1.z;
    query(offset + 4) = traj2.x;
    query(offset + 5) = traj2.z;
    
    offset += 6;
}

// Same but for the trajectory direction
void query_compute_trajectory_direction_feature(
    slice1d<float> query, 
    int& offset, 
    const quat root_rotation, 
    const slice1d<quat> trajectory_rotations)
{
    vec3 traj0 = quat_inv_mul_vec3(root_rotation, quat_mul_vec3(trajectory_rotations(1), vec3(0, 0, 1)));
    vec3 traj1 = quat_inv_mul_vec3(root_rotation, quat_mul_vec3(trajectory_rotations(2), vec3(0, 0, 1)));
    vec3 traj2 = quat_inv_mul_vec3(root_rotation, quat_mul_vec3(trajectory_rotations(3), vec3(0, 0, 1)));
    
    query(offset + 0) = traj0.x;
    query(offset + 1) = traj0.z;
    query(offset + 2) = traj1.x;
    query(offset + 3) = traj1.z;
    query(offset + 4) = traj2.x;
    query(offset + 5) = traj2.z;
    
    offset += 6;
}

//--------------------------------------

// Collide against the obscales which are
// essentially bounding boxes of a given size
vec3 simulation_collide_obstacles(
    const vec3 prev_pos,
    const vec3 next_pos,
    const slice1d<vec3> obstacles_positions,
    const slice1d<vec3> obstacles_scales,
    const float radius = 0.6f)
{
    vec3 dx = next_pos - prev_pos;
    vec3 proj_pos = prev_pos;
    
    // Substep because I'm too lazy to implement CCD
    int substeps = 1 + (int)(length(dx) * 5.0f);
    
    for (int j = 0; j < substeps; j++)
    {
        proj_pos = proj_pos + dx / substeps;
        
        for (int i = 0; i < obstacles_positions.size; i++)
        {
            // Find nearest point inside obscale and push out
            vec3 nearest = clamp(proj_pos, 
              obstacles_positions(i) - 0.5f * obstacles_scales(i),
              obstacles_positions(i) + 0.5f * obstacles_scales(i));

            if (length(nearest - proj_pos) < radius)
            {
                proj_pos = radius * normalize(proj_pos - nearest) + nearest;
            }
        }
    } 
    
    return proj_pos;
}

// Taken from https://theorangeduck.com/page/spring-roll-call#controllers
void simulation_positions_update(
    vec3& position, 
    vec3& velocity, 
    vec3& acceleration, 
    const vec3 desired_velocity, 
    const float halflife, 
    const float dt,
    const slice1d<vec3> obstacles_positions,
    const slice1d<vec3> obstacles_scales)
{
    float y = halflife_to_damping(halflife) / 2.0f; 
    vec3 j0 = velocity - desired_velocity;
    vec3 j1 = acceleration + j0*y;
    float eydt = fast_negexpf(y*dt);
    
    vec3 position_prev = position;

    position = eydt*(((-j1)/(y*y)) + ((-j0 - j1*dt)/y)) + 
        (j1/(y*y)) + j0/y + desired_velocity * dt + position_prev;
    velocity = eydt*(j0 + j1*dt) + desired_velocity;
    acceleration = eydt*(acceleration - j1*y*dt);
    
    position = simulation_collide_obstacles(
        position_prev, 
        position,
        obstacles_positions,
        obstacles_scales);
}

void simulation_rotations_update(
    quat& rotation, 
    vec3& angular_velocity, 
    const quat desired_rotation, 
    const float halflife, 
    const float dt)
{
    simple_spring_damper_exact(
        rotation, 
        angular_velocity, 
        desired_rotation, 
        halflife, dt);
}

// Predict what the desired velocity will be in the 
// future. Here we need to use the future trajectory 
// rotation as well as predicted future camera 
// position to find an accurate desired velocity in 
// the world space
void trajectory_desired_velocities_predict(
  slice1d<vec3> desired_velocities,
  const slice1d<quat> trajectory_rotations,
  const vec3 desired_velocity,
  const float camera_azimuth,
  const vec3 gamepadstick_left,
  const vec3 gamepadstick_right,
  const bool desired_strafe,
  const float fwrd_speed,
  const float side_speed,
  const float back_speed,
  const float dt)
{
    desired_velocities(0) = desired_velocity;
    
    for (int i = 1; i < desired_velocities.size; i++)
    {
        desired_velocities(i) = desired_velocity_update(
            gamepadstick_left,
            orbit_camera_update_azimuth(
                camera_azimuth, gamepadstick_right, desired_strafe, i * dt),
            trajectory_rotations(i),
            fwrd_speed,
            side_speed,
            back_speed);
    }
}

void trajectory_positions_predict(
    slice1d<vec3> positions, 
    slice1d<vec3> velocities, 
    slice1d<vec3> accelerations, 
    const vec3 position, 
    const vec3 velocity, 
    const vec3 acceleration, 
    const slice1d<vec3> desired_velocities, 
    const float halflife,
    const float dt,
    const slice1d<vec3> obstacles_positions,
    const slice1d<vec3> obstacles_scales)
{
    positions(0) = position;
    velocities(0) = velocity;
    accelerations(0) = acceleration;
    
    for (int i = 1; i < positions.size; i++)
    {
        positions(i) = positions(i-1);
        velocities(i) = velocities(i-1);
        accelerations(i) = accelerations(i-1);
        
        simulation_positions_update(
            positions(i), 
            velocities(i), 
            accelerations(i), 
            desired_velocities(i), 
            halflife, 
            dt, 
            obstacles_positions, 
            obstacles_scales);
    }
}

// Predict desired rotations given the estimated future 
// camera rotation and other parameters
void trajectory_desired_rotations_predict(
  slice1d<quat> desired_rotations,
  const slice1d<vec3> desired_velocities,
  const quat desired_rotation,
  const float camera_azimuth,
  const vec3 gamepadstick_left,
  const vec3 gamepadstick_right,
  const bool desired_strafe,
  const float dt)
{
    desired_rotations(0) = desired_rotation;
    
    for (int i = 1; i < desired_rotations.size; i++)
    {
        desired_rotations(i) = desired_rotation_update(
            desired_rotations(i-1),
            gamepadstick_left,
            gamepadstick_right,
            orbit_camera_update_azimuth(
                camera_azimuth, gamepadstick_right, desired_strafe, i * dt),
            desired_strafe,
            desired_velocities(i));
    }
}

void trajectory_rotations_predict(
    slice1d<quat> rotations, 
    slice1d<vec3> angular_velocities, 
    const quat rotation, 
    const vec3 angular_velocity, 
    const slice1d<quat> desired_rotations, 
    const float halflife,
    const float dt)
{
    rotations.set(rotation);
    angular_velocities.set(angular_velocity);
    
    for (int i = 1; i < rotations.size; i++)
    {
        simulation_rotations_update(
            rotations(i), 
            angular_velocities(i), 
            desired_rotations(i), 
            halflife, 
            i * dt);
    }
}

//--------------------------------------

void contact_reset(
    bool& contact_state,
    bool& contact_lock,
    vec3& contact_position,
    vec3& contact_velocity,
    vec3& contact_point,
    vec3& contact_target,
    vec3& contact_offset_position,
    vec3& contact_offset_velocity,
    const vec3 input_contact_position,
    const vec3 input_contact_velocity,
    const bool input_contact_state)
{
    contact_state = false;
    contact_lock = false;
    contact_position = input_contact_position;
    contact_velocity = input_contact_velocity;
    contact_point = input_contact_position;
    contact_target = input_contact_position;
    contact_offset_position = vec3();
    contact_offset_velocity = vec3();
}

void contact_update(
    bool& contact_state,
    bool& contact_lock,
    vec3& contact_position,
    vec3& contact_velocity,
    vec3& contact_point,
    vec3& contact_target,
    vec3& contact_offset_position,
    vec3& contact_offset_velocity,
    const vec3 input_contact_position,
    const bool input_contact_state,
    const float unlock_radius,
    const float foot_height,
    const float halflife,
    const float dt,
    const float eps=1e-8)
{
    // First compute the input contact position velocity via finite difference
    vec3 input_contact_velocity = 
        (input_contact_position - contact_target) / (dt + eps);    
    contact_target = input_contact_position;
    
    // Update the inertializer to tick forward in time
    inertialize_update(
        contact_position,
        contact_velocity,
        contact_offset_position,
        contact_offset_velocity,
        // If locked we feed the contact point and zero velocity, 
        // otherwise we feed the input from the animation
        contact_lock ? contact_point : input_contact_position,
        contact_lock ?        vec3() : input_contact_velocity,
        halflife,
        dt);
    
    // If the contact point is too far from the current input position 
    // then we need to unlock the contact
    bool unlock_contact = contact_lock && (
        length(contact_point - input_contact_position) > unlock_radius);
    
    // If the contact was previously inactive but is now active we 
    // need to transition to the locked contact state
    if (!contact_state && input_contact_state)
    {
        // Contact point is given by the current position of 
        // the foot projected onto the ground plus foot height
        contact_lock = true;
        contact_point = contact_position;
        contact_point.y = foot_height;
        
        inertialize_transition(
            contact_offset_position,
            contact_offset_velocity,
            input_contact_position,
            input_contact_velocity,
            contact_point,
            vec3());
    }
    
    // Otherwise if we need to unlock or we were previously in 
    // contact but are no longer we transition to just taking 
    // the input position as-is
    else if ((contact_lock && contact_state && !input_contact_state) 
         || unlock_contact)
    {
        contact_lock = false;
        
        inertialize_transition(
            contact_offset_position,
            contact_offset_velocity,
            contact_point,
            vec3(),
            input_contact_position,
            input_contact_velocity);
    }
    
    // Update contact state
    contact_state = input_contact_state;
}

//--------------------------------------

// Rotate a joint to look toward some 
// given target position
void ik_look_at(
    quat& bone_rotation,
    const quat global_parent_rotation,
    const quat global_rotation,
    const vec3 global_position,
    const vec3 child_position,
    const vec3 target_position,
    const float eps = 1e-5f)
{
    vec3 curr_dir = normalize(child_position - global_position);
    vec3 targ_dir = normalize(target_position - global_position);

    if (fabs(1.0f - dot(curr_dir, targ_dir) > eps))
    {
        bone_rotation = quat_inv_mul(global_parent_rotation, 
            quat_mul(quat_between(curr_dir, targ_dir), global_rotation));
    }
}

// Basic two-joint IK in the style of https://theorangeduck.com/page/simple-two-joint
// Here I add a basic "forward vector" which acts like a kind of pole-vetor
// to control the bending direction
void ik_two_bone(
    quat& bone_root_lr, 
    quat& bone_mid_lr,
    const vec3 bone_root, 
    const vec3 bone_mid, 
    const vec3 bone_end, 
    const vec3 target, 
    const vec3 fwd,
    const quat bone_root_gr, 
    const quat bone_mid_gr,
    const quat bone_par_gr,
    const float max_length_buffer) {
    
    float max_extension = 
        length(bone_root - bone_mid) + 
        length(bone_mid - bone_end) - 
        max_length_buffer;
    
    vec3 target_clamp = target;
    if (length(target - bone_root) > max_extension)
    {
        target_clamp = bone_root + max_extension * normalize(target - bone_root);
    }
    
    vec3 axis_dwn = normalize(bone_end - bone_root);
    vec3 axis_rot = normalize(cross(axis_dwn, fwd));

    vec3 a = bone_root;
    vec3 b = bone_mid;
    vec3 c = bone_end;
    vec3 t = target_clamp;
    
    float lab = length(b - a);
    float lcb = length(b - c);
    float lat = length(t - a);

    float ac_ab_0 = acosf(clampf(dot(normalize(c - a), normalize(b - a)), -1.0f, 1.0f));
    float ba_bc_0 = acosf(clampf(dot(normalize(a - b), normalize(c - b)), -1.0f, 1.0f));

    float ac_ab_1 = acosf(clampf((lab * lab + lat * lat - lcb * lcb) / (2.0f * lab * lat), -1.0f, 1.0f));
    float ba_bc_1 = acosf(clampf((lab * lab + lcb * lcb - lat * lat) / (2.0f * lab * lcb), -1.0f, 1.0f));

    quat r0 = quat_from_angle_axis(ac_ab_1 - ac_ab_0, axis_rot);
    quat r1 = quat_from_angle_axis(ba_bc_1 - ba_bc_0, axis_rot);

    vec3 c_a = normalize(bone_end - bone_root);
    vec3 t_a = normalize(target_clamp - bone_root);

    quat r2 = quat_from_angle_axis(
        acosf(clampf(dot(c_a, t_a), -1.0f, 1.0f)),
        normalize(cross(c_a, t_a)));
    
    bone_root_lr = quat_inv_mul(bone_par_gr, quat_mul(r2, quat_mul(r0, bone_root_gr)));
    bone_mid_lr = quat_inv_mul(bone_root_gr, quat_mul(r1, bone_mid_gr));
}

//--------------------------------------

vec3 adjust_character_position(
    const vec3 character_position,
    const vec3 simulation_position,
    const float halflife,
    const float dt)
{
    // Find the difference in positioning
    vec3 difference_position = simulation_position - character_position;
    
    // Damp that difference using the given halflife and dt
    vec3 adjustment_position = damp_adjustment_exact(
        difference_position,
        halflife,
        dt);
    
    // Add the damped difference to move the character toward the sim
    return adjustment_position + character_position;
}

quat adjust_character_rotation(
    const quat character_rotation,
    const quat simulation_rotation,
    const float halflife,
    const float dt)
{
    // Find the difference in rotation (from character to simulation).
    // Here `quat_abs` forces the quaternion to take the shortest 
    // path and normalization is required as sometimes taking 
    // the difference between two very similar rotations can 
    // introduce numerical instability
    quat difference_rotation = quat_abs(quat_normalize(
        quat_mul_inv(simulation_rotation, character_rotation)));
    
    // Damp that difference using the given halflife and dt
    quat adjustment_rotation = damp_adjustment_exact(
        difference_rotation,
        halflife,
        dt);
    
    // Apply the damped adjustment to the character
    return quat_mul(adjustment_rotation, character_rotation);
}

vec3 adjust_character_position_by_velocity(
    const vec3 character_position,
    const vec3 character_velocity,
    const vec3 simulation_position,
    const float max_adjustment_ratio,
    const float halflife,
    const float dt)
{
    // Find and damp the desired adjustment
    vec3 adjustment_position = damp_adjustment_exact(
        simulation_position - character_position,
        halflife,
        dt);
    
    // If the length of the adjustment is greater than the character velocity 
    // multiplied by the ratio then we need to clamp it to that length
    float max_length = max_adjustment_ratio * length(character_velocity) * dt;
    
    if (length(adjustment_position) > max_length)
    {
        adjustment_position = max_length * normalize(adjustment_position);
    }
    
    // Apply the adjustment
    return adjustment_position + character_position;
}

quat adjust_character_rotation_by_velocity(
    const quat character_rotation,
    const vec3 character_angular_velocity,
    const quat simulation_rotation,
    const float max_adjustment_ratio,
    const float halflife,
    const float dt)
{
    // Find and damp the desired rotational adjustment
    quat adjustment_rotation = damp_adjustment_exact(
        quat_abs(quat_normalize(quat_mul_inv(
            simulation_rotation, character_rotation))),
        halflife,
        dt);
    
    // If the length of the adjustment is greater than the angular velocity 
    // multiplied by the ratio then we need to clamp this adjustment
    float max_length = max_adjustment_ratio *
        length(character_angular_velocity) * dt;
    
    if (length(quat_to_scaled_angle_axis(adjustment_rotation)) > max_length)
    {
        // To clamp can convert to scaled angle axis, rescale, and convert back
        adjustment_rotation = quat_from_scaled_angle_axis(max_length * 
            normalize(quat_to_scaled_angle_axis(adjustment_rotation)));
    }
    
    // Apply the adjustment
    return quat_mul(adjustment_rotation, character_rotation);
}

//--------------------------------------

vec3 clamp_character_position(
    const vec3 character_position,
    const vec3 simulation_position,
    const float max_distance)
{
    // If the character deviates too far from the simulation 
    // position we need to clamp it to within the max distance
    if (length(character_position - simulation_position) > max_distance)
    {
        return max_distance * 
            normalize(character_position - simulation_position) + 
            simulation_position;
    }
    else
    {
        return character_position;
    }
}
  
quat clamp_character_rotation(
    const quat character_rotation,
    const quat simulation_rotation,
    const float max_angle)
{
    // If the angle between the character rotation and simulation 
    // rotation exceeds the threshold we need to clamp it back
    if (quat_angle_between(character_rotation, simulation_rotation) > max_angle)
    {
        // First, find the rotational difference between the two
        quat diff = quat_abs(quat_mul_inv(
            character_rotation, simulation_rotation));
        
        // We can then decompose it into angle and axis
        float diff_angle; vec3 diff_axis;
        quat_to_angle_axis(diff, diff_angle, diff_axis);
        
        // We then clamp the angle to within our bounds
        diff_angle = clampf(diff_angle, -max_angle, max_angle);
        
        // And apply back the clamped rotation
        return quat_mul(
          quat_from_angle_axis(diff_angle, diff_axis), simulation_rotation);
    }
    else
    {
        return character_rotation;
    }
}

//--------------------------------------

void controller::rebuild_database()
{
    database_build_matching_features(
        db,
        feature_weight_foot_position,
        feature_weight_foot_velocity,
        feature_weight_hip_velocity,
        feature_weight_trajectory_positions,
        feature_weight_trajectory_directions);
}

void controller::update(float dt)
{
    vec3 gamepadstick_left = movement_data;
    vec3 gamepadstick_right = camera_data;
    bool desired_strafe = strafe_data;
    
    orbit_camera_update(
        camera_azimuth,
        camera_altitude,
        camera_distance,
        // simulation_position + vec3(0, 1, 0),
        gamepadstick_right,
        desired_strafe,
        dt);
    
    // Get the desired gait (walk / run)
    desired_gait_update(
        desired_gait,
        desired_gait_velocity,
        dt);
    
    // Get the desired simulation speeds based on the gait
    float simulation_fwrd_speed = lerpf(simulation_run_fwrd_speed, simulation_walk_fwrd_speed, desired_gait);
    float simulation_side_speed = lerpf(simulation_run_side_speed, simulation_walk_side_speed, desired_gait);
    float simulation_back_speed = lerpf(simulation_run_back_speed, simulation_walk_back_speed, desired_gait);
    
    // Get the desired velocity
    vec3 desired_velocity_curr = desired_velocity_update(
        gamepadstick_left,
        camera_azimuth,
        simulation_rotation,
        simulation_fwrd_speed,
        simulation_side_speed,
        simulation_back_speed);
        
    // Get the desired rotation/direction
    quat desired_rotation_curr = desired_rotation_update(
        desired_rotation,
        gamepadstick_left,
        gamepadstick_right,
        camera_azimuth,
        desired_strafe,
        desired_velocity_curr);
    
    // Check if we should force a search because input changed quickly
    desired_velocity_change_prev = desired_velocity_change_curr;
    desired_velocity_change_curr =  (desired_velocity_curr - desired_velocity) / dt;
    desired_velocity = desired_velocity_curr;
    
    desired_rotation_change_prev = desired_rotation_change_curr;
    desired_rotation_change_curr = quat_to_scaled_angle_axis(quat_abs(quat_mul_inv(desired_rotation_curr, desired_rotation))) / dt;
    desired_rotation =  desired_rotation_curr;
    
    bool force_search = false;

    if (force_search_timer <= 0.0f && (
        (length(desired_velocity_change_prev) >= desired_velocity_change_threshold && 
         length(desired_velocity_change_curr)  < desired_velocity_change_threshold)
    ||  (length(desired_rotation_change_prev) >= desired_rotation_change_threshold && 
         length(desired_rotation_change_curr)  < desired_rotation_change_threshold)))
    {
        force_search = true;
        force_search_timer = search_time;
    }
    else if (force_search_timer > 0)
    {
        force_search_timer -= dt;
    }
    
    // Predict Future Trajectory
    
    trajectory_desired_rotations_predict(
      trajectory_desired_rotations,
      trajectory_desired_velocities,
      desired_rotation,
      camera_azimuth,
      gamepadstick_left,
      gamepadstick_right,
      desired_strafe,
      20.0f * dt);
    
    trajectory_rotations_predict(
        trajectory_rotations,
        trajectory_angular_velocities,
        simulation_rotation,
        simulation_angular_velocity,
        trajectory_desired_rotations,
        simulation_rotation_halflife,
        20.0f * dt);
    
    trajectory_desired_velocities_predict(
      trajectory_desired_velocities,
      trajectory_rotations,
      desired_velocity,
      camera_azimuth,
      gamepadstick_left,
      gamepadstick_right,
      desired_strafe,
      simulation_fwrd_speed,
      simulation_side_speed,
      simulation_back_speed,
      20.0f * dt);
    
    trajectory_positions_predict(
        trajectory_positions,
        trajectory_velocities,
        trajectory_accelerations,
        simulation_position,
        simulation_velocity,
        simulation_acceleration,
        trajectory_desired_velocities,
        simulation_velocity_halflife,
        20.0f * dt,
        obstacles_positions,
        obstacles_scales);
       
    // Make query vector for search.
    // In theory this only needs to be done when a search is 
    // actually required however for visualization purposes it
    // can be nice to do it every frame
    array1d<float> query(db.nfeatures());
            
    // Compute the features of the query vector

    slice1d<float> query_features = lmm_enabled ? slice1d<float>(features_curr) : db.features(frame_index);

    int offset = 0;
    query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Left Foot Position
    query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Right Foot Position
    query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Left Foot Velocity
    query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Right Foot Velocity
    query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Hip Velocity
    query_compute_trajectory_position_feature(query, offset, bone_positions(0), bone_rotations(0), trajectory_positions);
    query_compute_trajectory_direction_feature(query, offset, bone_rotations(0), trajectory_rotations);
    
    assert(offset == db.nfeatures());

    // Check if we reached the end of the current anim
    bool end_of_anim = database_trajectory_index_clamp(db, frame_index, 1) == frame_index;
    
    // Do we need to search?
    if (force_search || search_timer <= 0.0f || end_of_anim)
    {
        if (lmm_enabled)
        {
            // Project query onto nearest feature vector
            
            float best_cost = FLT_MAX;
            bool transition = false;
            
            projector_evaluate(
                transition,
                best_cost,
                features_proj,
                latent_proj,
                projector_evaluation,
                query,
                db.features_offset,
                db.features_scale,
                features_curr,
                projector);
            
            // If projection is sufficiently different from current
            if (transition)
            {   
                // Evaluate pose for projected features
                decompressor_evaluate(
                    trns_bone_positions,
                    trns_bone_velocities,
                    trns_bone_rotations,
                    trns_bone_angular_velocities,
                    trns_bone_contacts,
                    decompressor_evaluation,
                    features_proj,
                    latent_proj,
                    curr_bone_positions(0),
                    curr_bone_rotations(0),
                    decompressor,
                    dt);
                
                // Transition inertializer to this pose
                inertialize_pose_transition(
                    bone_offset_positions,
                    bone_offset_velocities,
                    bone_offset_rotations,
                    bone_offset_angular_velocities,
                    transition_src_position,
                    transition_src_rotation,
                    transition_dst_position,
                    transition_dst_rotation,
                    bone_positions(0),
                    bone_velocities(0),
                    bone_rotations(0),
                    bone_angular_velocities(0),
                    curr_bone_positions,
                    curr_bone_velocities,
                    curr_bone_rotations,
                    curr_bone_angular_velocities,
                    trns_bone_positions,
                    trns_bone_velocities,
                    trns_bone_rotations,
                    trns_bone_angular_velocities);
                
                // Update current features and latents
                features_curr = features_proj;
                latent_curr = latent_proj;
            }
        }
        else
        {
            // Search
            
            int best_index = end_of_anim ? -1 : frame_index;
            float best_cost = FLT_MAX;
            
            database_search(
                best_index,
                best_cost,
                db,
                query);
            
            // Transition if better frame found
            
            if (best_index != frame_index)
            {
                trns_bone_positions = db.bone_positions(best_index);
                trns_bone_velocities = db.bone_velocities(best_index);
                trns_bone_rotations = db.bone_rotations(best_index);
                trns_bone_angular_velocities = db.bone_angular_velocities(best_index);
                
                inertialize_pose_transition(
                    bone_offset_positions,
                    bone_offset_velocities,
                    bone_offset_rotations,
                    bone_offset_angular_velocities,
                    transition_src_position,
                    transition_src_rotation,
                    transition_dst_position,
                    transition_dst_rotation,
                    bone_positions(0),
                    bone_velocities(0),
                    bone_rotations(0),
                    bone_angular_velocities(0),
                    curr_bone_positions,
                    curr_bone_velocities,
                    curr_bone_rotations,
                    curr_bone_angular_velocities,
                    trns_bone_positions,
                    trns_bone_velocities,
                    trns_bone_rotations,
                    trns_bone_angular_velocities);
                
                frame_index = best_index;
            }
        }

        // Reset search timer
        search_timer = search_time;
    }
    
    // Tick down search timer
    search_timer -= dt;

    if (lmm_enabled)
    {
        // Update features and latents
        stepper_evaluate(
            features_curr,
            latent_curr,
            stepper_evaluation,
            stepper,
            dt);
        
        // Decompress next pose
        decompressor_evaluate(
            curr_bone_positions,
            curr_bone_velocities,
            curr_bone_rotations,
            curr_bone_angular_velocities,
            curr_bone_contacts,
            decompressor_evaluation,
            features_curr,
            latent_curr,
            curr_bone_positions(0),
            curr_bone_rotations(0),
            decompressor,
            dt);
    }
    else
    {
        // Tick frame
        frame_index++; // Assumes dt is fixed to 60fps
        
        // Look-up Next Pose
        curr_bone_positions = db.bone_positions(frame_index);
        curr_bone_velocities = db.bone_velocities(frame_index);
        curr_bone_rotations = db.bone_rotations(frame_index);
        curr_bone_angular_velocities = db.bone_angular_velocities(frame_index);
        curr_bone_contacts = db.contact_states(frame_index);
    }
    
    // Update inertializer
    
    inertialize_pose_update(
        bone_positions,
        bone_velocities,
        bone_rotations,
        bone_angular_velocities,
        bone_offset_positions,
        bone_offset_velocities,
        bone_offset_rotations,
        bone_offset_angular_velocities,
        curr_bone_positions,
        curr_bone_velocities,
        curr_bone_rotations,
        curr_bone_angular_velocities,
        transition_src_position,
        transition_src_rotation,
        transition_dst_position,
        transition_dst_rotation,
        inertialize_blending_halflife,
        dt);
    
    // Update Simulation
    
    vec3 simulation_position_prev = simulation_position;
    
    simulation_positions_update(
        simulation_position, 
        simulation_velocity, 
        simulation_acceleration,
        desired_velocity,
        simulation_velocity_halflife,
        dt,
        obstacles_positions,
        obstacles_scales);
        
    simulation_rotations_update(
        simulation_rotation, 
        simulation_angular_velocity, 
        desired_rotation,
        simulation_rotation_halflife,
        dt);
    
    // Synchronization 
    
    if (synchronization_enabled)
    {
        vec3 synchronized_position = lerp(
            simulation_position, 
            bone_positions(0),
            synchronization_data_factor);
            
        quat synchronized_rotation = quat_nlerp_shortest(
            simulation_rotation,
            bone_rotations(0), 
            synchronization_data_factor);
      
        synchronized_position = simulation_collide_obstacles(
            simulation_position_prev,
            synchronized_position,
            obstacles_positions,
            obstacles_scales);
        
        simulation_position = synchronized_position;
        simulation_rotation = synchronized_rotation;
        
        inertialize_root_adjust(
            bone_offset_positions(0),
            transition_src_position,
            transition_src_rotation,
            transition_dst_position,
            transition_dst_rotation,
            bone_positions(0),
            bone_rotations(0),
            synchronized_position,
            synchronized_rotation);
    }
    
    // Adjustment 
    
    if (!synchronization_enabled && adjustment_enabled)
    {   
        vec3 adjusted_position = bone_positions(0);
        quat adjusted_rotation = bone_rotations(0);
        
        if (adjustment_by_velocity_enabled)
        {
            adjusted_position = adjust_character_position_by_velocity(
                bone_positions(0),
                bone_velocities(0),
                simulation_position,
                adjustment_position_max_ratio,
                adjustment_position_halflife,
                dt);
            
            adjusted_rotation = adjust_character_rotation_by_velocity(
                bone_rotations(0),
                bone_angular_velocities(0),
                simulation_rotation,
                adjustment_rotation_max_ratio,
                adjustment_rotation_halflife,
                dt);
        }
        else
        {
            adjusted_position = adjust_character_position(
                bone_positions(0),
                simulation_position,
                adjustment_position_halflife,
                dt);
            
            adjusted_rotation = adjust_character_rotation(
                bone_rotations(0),
                simulation_rotation,
                adjustment_rotation_halflife,
                dt);
        }
  
        inertialize_root_adjust(
            bone_offset_positions(0),
            transition_src_position,
            transition_src_rotation,
            transition_dst_position,
            transition_dst_rotation,
            bone_positions(0),
            bone_rotations(0),
            adjusted_position,
            adjusted_rotation);
    }
    
    // Clamping
    
    if (!synchronization_enabled && clamping_enabled)
    {
        vec3 adjusted_position = bone_positions(0);
        quat adjusted_rotation = bone_rotations(0);
        
        adjusted_position = clamp_character_position(
            adjusted_position,
            simulation_position,
            clamping_max_distance);
        
        adjusted_rotation = clamp_character_rotation(
            adjusted_rotation,
            simulation_rotation,
            clamping_max_angle);
        
        inertialize_root_adjust(
            bone_offset_positions(0),
            transition_src_position,
            transition_src_rotation,
            transition_dst_position,
            transition_dst_rotation,
            bone_positions(0),
            bone_rotations(0),
            adjusted_position,
            adjusted_rotation);
    }
    
    // Contact fixup with foot locking and IK

    adjusted_bone_positions = bone_positions;
    adjusted_bone_rotations = bone_rotations;

    if (ik_enabled)
    {
        for (int i = 0; i < contact_bones.size; i++)
        {
            // Find all the relevant bone indices
            int toe_bone = contact_bones(i);
            int heel_bone = db.bone_parents(toe_bone);
            int knee_bone = db.bone_parents(heel_bone);
            int hip_bone = db.bone_parents(knee_bone);
            int root_bone = db.bone_parents(hip_bone);
            
            // Compute the world space position for the toe
            global_bone_computed.zero();
            
            forward_kinematics_partial(
                global_bone_positions,
                global_bone_rotations,
                global_bone_computed,
                bone_positions,
                bone_rotations,
                db.bone_parents,
                toe_bone);
            
            // Update the contact state
            contact_update(
                contact_states(i),
                contact_locks(i),
                contact_positions(i),  
                contact_velocities(i),
                contact_points(i),
                contact_targets(i),
                contact_offset_positions(i),
                contact_offset_velocities(i),
                global_bone_positions(toe_bone),
                curr_bone_contacts(i),
                ik_unlock_radius,
                ik_foot_height,
                ik_blending_halflife,
                dt);
            
            // Ensure contact position never goes through floor
            vec3 contact_position_clamp = contact_positions(i);
            contact_position_clamp.y = maxf(contact_position_clamp.y, ik_foot_height);
            
            // Re-compute toe, heel, knee, hip, and root bone positions
            for (int bone : {heel_bone, knee_bone, hip_bone, root_bone})
            {
                forward_kinematics_partial(
                    global_bone_positions,
                    global_bone_rotations,
                    global_bone_computed,
                    bone_positions,
                    bone_rotations,
                    db.bone_parents,
                    bone);
            }
            
            // Perform simple two-joint IK to place heel
            ik_two_bone(
                adjusted_bone_rotations(hip_bone),
                adjusted_bone_rotations(knee_bone),
                global_bone_positions(hip_bone),
                global_bone_positions(knee_bone),
                global_bone_positions(heel_bone),
                contact_position_clamp + (global_bone_positions(heel_bone) - global_bone_positions(toe_bone)),
                quat_mul_vec3(global_bone_rotations(knee_bone), vec3(0.0f, 1.0f, 0.0f)),
                global_bone_rotations(hip_bone),
                global_bone_rotations(knee_bone),
                global_bone_rotations(root_bone),
                ik_max_length_buffer);
            
            // Re-compute toe, heel, and knee positions 
            global_bone_computed.zero();
            
            for (int bone : {toe_bone, heel_bone, knee_bone})
            {
                forward_kinematics_partial(
                    global_bone_positions,
                    global_bone_rotations,
                    global_bone_computed,
                    adjusted_bone_positions,
                    adjusted_bone_rotations,
                    db.bone_parents,
                    bone);
            }
            
            // Rotate heel so toe is facing toward contact point
            ik_look_at(
                adjusted_bone_rotations(heel_bone),
                global_bone_rotations(knee_bone),
                global_bone_rotations(heel_bone),
                global_bone_positions(heel_bone),
                global_bone_positions(toe_bone),
                contact_position_clamp);
            
            // Re-compute toe and heel positions
            global_bone_computed.zero();
            
            for (int bone : {toe_bone, heel_bone})
            {
                forward_kinematics_partial(
                    global_bone_positions,
                    global_bone_rotations,
                    global_bone_computed,
                    adjusted_bone_positions,
                    adjusted_bone_rotations,
                    db.bone_parents,
                    bone);
            }
            
            // Rotate toe bone so that the end of the toe 
            // does not intersect with the ground
            vec3 toe_end_curr = quat_mul_vec3(
                global_bone_rotations(toe_bone), vec3(ik_toe_length, 0.0f, 0.0f)) + 
                global_bone_positions(toe_bone);
                
            vec3 toe_end_targ = toe_end_curr;
            toe_end_targ.y = maxf(toe_end_targ.y, ik_foot_height);
            
            ik_look_at(
                adjusted_bone_rotations(toe_bone),
                global_bone_rotations(heel_bone),
                global_bone_rotations(toe_bone),
                global_bone_positions(toe_bone),
                toe_end_curr,
                toe_end_targ);
        }
    }
    
    // Full pass of forward kinematics to compute 
    // all bone positions and rotations in the world
    // space ready for rendering
    
    forward_kinematics_full(
        global_bone_positions,
        global_bone_rotations,
        adjusted_bone_positions,
        adjusted_bone_rotations,
        db.bone_parents);
    
    
    // Foot locking needs resetting when IK is toggled
    if (ik_enabled && !ik_enabled_prev)
    {
        ik_enabled_prev = true;
        for (int i = 0; i < contact_bones.size; i++)
        {
            vec3 bone_position;
            vec3 bone_velocity;
            quat bone_rotation;
            vec3 bone_angular_velocity;
            
            forward_kinematics_velocity(
                bone_position,
                bone_velocity,
                bone_rotation,
                bone_angular_velocity,
                bone_positions,
                bone_velocities,
                bone_rotations,
                bone_angular_velocities,
                db.bone_parents,
                contact_bones(i));
            
            contact_reset(
                contact_states(i),
                contact_locks(i),
                contact_positions(i),  
                contact_velocities(i),
                contact_points(i),
                contact_targets(i),
                contact_offset_positions(i),
                contact_offset_velocities(i),
                bone_position,
                bone_velocity,
                false);
        }
    }
    

};

bool file_exists(const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }   
}

bool controller::init(char* path)
{
    const std::string path_str(path);
    const std::string character_path = path_str + "/character.bin";
    const std::string database_path = path_str + "/database.bin";
    const std::string features_path = path_str + "/features.bin";
    const std::string decompressor_path = path_str + "/decompressor.bin";
    const std::string stepper_path = path_str + "/stepper.bin";
    const std::string projector_path = path_str + "/projector.bin";

    // if file not exist, return false
    if (!file_exists(character_path) || !file_exists(database_path)
        || !file_exists(features_path) || !file_exists(decompressor_path)
        || !file_exists(stepper_path) || !file_exists(projector_path))
    {
        return false;
    }
    
    // Character
    
    character_load(character_data, character_path.c_str());
    
    // Load Animation Data and build Matching Database
    
    database_load(db, database_path.c_str());
    
    database_build_matching_features(
        db,
        feature_weight_foot_position,
        feature_weight_foot_velocity,
        feature_weight_hip_velocity,
        feature_weight_trajectory_positions,
        feature_weight_trajectory_directions);
        
    database_save_matching_features(db, features_path.c_str());
   
    // Pose & Inertializer Data
    
    frame_index = db.range_starts(0);
    inertialize_blending_halflife = 0.1f;

    curr_bone_positions = db.bone_positions(frame_index);
    curr_bone_velocities = db.bone_velocities(frame_index);
    curr_bone_rotations = db.bone_rotations(frame_index);
    curr_bone_angular_velocities = db.bone_angular_velocities(frame_index);
    curr_bone_contacts = db.contact_states(frame_index);

    trns_bone_positions = db.bone_positions(frame_index);
    trns_bone_velocities = db.bone_velocities(frame_index);
    trns_bone_rotations = db.bone_rotations(frame_index);
    trns_bone_angular_velocities = db.bone_angular_velocities(frame_index);
    trns_bone_contacts = db.contact_states(frame_index);

    bone_positions = db.bone_positions(frame_index);
    bone_velocities = db.bone_velocities(frame_index);
    bone_rotations = db.bone_rotations(frame_index);
    bone_angular_velocities = db.bone_angular_velocities(frame_index);
    
    bone_offset_positions.resize(db.nbones());
    bone_offset_velocities.resize(db.nbones());
    bone_offset_rotations.resize(db.nbones());
    bone_offset_angular_velocities.resize(db.nbones());
    
    global_bone_positions.resize(db.nbones());
    global_bone_velocities.resize(db.nbones());
    global_bone_rotations.resize(db.nbones());
    global_bone_angular_velocities.resize(db.nbones());
    global_bone_computed.resize(db.nbones());
    
    
    inertialize_pose_reset(
        bone_offset_positions,
        bone_offset_velocities,
        bone_offset_rotations,
        bone_offset_angular_velocities,
        transition_src_position,
        transition_src_rotation,
        transition_dst_position,
        transition_dst_rotation,
        bone_positions(0),
        bone_rotations(0));
    
    inertialize_pose_update(
        bone_positions,
        bone_velocities,
        bone_rotations,
        bone_angular_velocities,
        bone_offset_positions,
        bone_offset_velocities,
        bone_offset_rotations,
        bone_offset_angular_velocities,
        db.bone_positions(frame_index),
        db.bone_velocities(frame_index),
        db.bone_rotations(frame_index),
        db.bone_angular_velocities(frame_index),
        transition_src_position,
        transition_src_rotation,
        transition_dst_position,
        transition_dst_rotation,
        inertialize_blending_halflife,
        0.0f);
        
    trajectory_desired_velocities.resize(4);
    trajectory_desired_rotations.resize(4);
    trajectory_positions.resize(4);
    trajectory_velocities.resize(4);
    trajectory_accelerations.resize(4);
    trajectory_rotations.resize(4);
    trajectory_angular_velocities.resize(4);
    
    // Contact and Foot Locking data
    
    contact_bones.resize(2);
    contact_bones(0) = Bone_LeftToe;
    contact_bones(1) = Bone_RightToe;
    
    contact_states.resize(contact_bones.size);
    contact_locks.resize(contact_bones.size);
    contact_positions.resize(contact_bones.size);
    contact_velocities.resize(contact_bones.size);
    contact_points.resize(contact_bones.size);
    contact_targets.resize(contact_bones.size);
    contact_offset_positions.resize(contact_bones.size);
    contact_offset_velocities.resize(contact_bones.size);
    
    for (int i = 0; i < contact_bones.size; i++)
    {
        vec3 bone_position;
        vec3 bone_velocity;
        quat bone_rotation;
        vec3 bone_angular_velocity;
        
        forward_kinematics_velocity(
            bone_position,
            bone_velocity,
            bone_rotation,
            bone_angular_velocity,
            bone_positions,
            bone_velocities,
            bone_rotations,
            bone_angular_velocities,
            db.bone_parents,
            contact_bones(i));
        
        contact_reset(
            contact_states(i),
            contact_locks(i),
            contact_positions(i),  
            contact_velocities(i),
            contact_points(i),
            contact_targets(i),
            contact_offset_positions(i),
            contact_offset_velocities(i),
            bone_position,
            bone_velocity,
            false);
    }
    
    adjusted_bone_positions = bone_positions;
    adjusted_bone_rotations = bone_rotations;
    
    // Learned Motion Matching    
    lmm_enabled = false;
    
    nnet_load(decompressor, decompressor_path.c_str());
    nnet_load(stepper, stepper_path.c_str());
    nnet_load(projector, projector_path.c_str());

    decompressor_evaluation.resize(decompressor);
    stepper_evaluation.resize(stepper);
    projector_evaluation.resize(projector);

    features_proj = db.features(frame_index);
    features_curr = db.features(frame_index);
    latent_proj.resize(32); latent_proj.zero();
    latent_curr.resize(32); latent_curr.zero();

    return true;
}
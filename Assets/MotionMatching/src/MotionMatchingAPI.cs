using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace FollowMe.Runtime.MotionMatching
{
// #define MAX_TRAJECTORY_LENGTH 10
// #define MAX_BONE_LENGTH 23

    public enum ControllerBones
    {
        Bone_Entity        = 0,
        
        Bone_Hips          = 1,
        Bone_LeftUpLeg     = 2,
        Bone_LeftLeg       = 3,
        Bone_LeftFoot      = 4,
        Bone_LeftToe       = 5,
        Bone_RightUpLeg    = 6,
        Bone_RightLeg      = 7,
        Bone_RightFoot     = 8,
        Bone_RightToe      = 9,
        Bone_Spine         = 10,
        Bone_Spine1        = 11,
        Bone_Spine2        = 12,
        Bone_Neck          = 13,
        Bone_Head          = 14,
        Bone_LeftShoulder  = 15,
        Bone_LeftArm       = 16,
        Bone_LeftForeArm   = 17,
        Bone_LeftHand      = 18,
        Bone_RightShoulder = 19,
        Bone_RightArm      = 20,
        Bone_RightForeArm  = 21,
        Bone_RightHand     = 22,
        
        Bone_Count         = 23,
    }
    
    [StructLayout(LayoutKind.Sequential)]
    public struct ControllerPose
    {
        int bone_length;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 23)]
        public int[] bone_parents;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 23)]
        public Vector3[] bone_rest_positions;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 23)]
        public Quaternion[] bone_rest_rotations;
    }
    
    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct ControllerInput
    {
        public bool lmm_enabled;
        public float simulation_velocity_halflife;
        public float simulation_rotation_halflife;    
        public float simulation_run_fwrd_speed;
        public float simulation_run_side_speed;
        public float simulation_run_back_speed;    
        public float simulation_walk_fwrd_speed;
        public float simulation_walk_side_speed;
        public float simulation_walk_back_speed;
        public float inertialize_blending_halflife;            
        public float feature_weight_foot_position;
        public float feature_weight_foot_velocity;
        public float feature_weight_hip_velocity;
        public float feature_weight_trajectory_positions;
        public float feature_weight_trajectory_directions;        
        public bool synchronization_enabled;
        public float synchronization_data_factor;        
        public bool adjustment_enabled;
        public bool adjustment_by_velocity_enabled;
        public float adjustment_position_halflife;
        public float adjustment_rotation_halflife;        
        public bool clamping_enabled;
        public float clamping_max_distance;
        public float clamping_max_angle;        
        public bool ik_enabled;
        public float ik_unlock_radius;
        public float ik_blending_halflife;

        public void Reset()
        {
            lmm_enabled = false;
            simulation_velocity_halflife = 0.27f;
            simulation_rotation_halflife = 0.27f;
            simulation_run_fwrd_speed = 4.0f;
            simulation_run_side_speed = 3.0f;
            simulation_run_back_speed = 2.5f;
            simulation_walk_fwrd_speed = 1.75f;
            simulation_walk_side_speed = 1.5f;
            simulation_walk_back_speed = 1.25f;
            inertialize_blending_halflife = 0.0f;
            feature_weight_foot_position = 0.75f;
            feature_weight_foot_velocity = 1.0f;
            feature_weight_hip_velocity = 1.0f;
            feature_weight_trajectory_positions = 1.0f;
            feature_weight_trajectory_directions = 1.5f;
            synchronization_enabled = false;
            synchronization_data_factor = 1.0f;
            adjustment_enabled = true;
            adjustment_by_velocity_enabled = true;
            adjustment_position_halflife = 0.1f;
            adjustment_rotation_halflife = 0.2f;
            clamping_enabled = true;
            clamping_max_distance = 0.15f;
            clamping_max_angle = 0.5f * 3.1415926f;
            ik_enabled = true;
            ik_unlock_radius = 0.2f;
            ik_blending_halflife = 0.1f;
        }
    }
    
    [StructLayout(LayoutKind.Sequential)]
    public struct ControllerOutput
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 1)]
        public Vector3 simulation_position;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 1)]
        public Quaternion simulation_rotation;

        public int trajectory_length;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 10)]
        public Vector3[] trajectory_positions;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 10)]
        public Quaternion[] trajectory_rotations;

        public int bone_length;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 23)]
        public Vector3[] bone_positions;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 23)]
        public Quaternion[] bone_rotations;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 23)]
        public Vector3[] global_bone_positions;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 23)]
        public Quaternion[] global_bone_rotations;
    }

    public class MMUtils
    {
        public static Vector3 ToVector3(Vector3 values)
        {
            return new Vector3(-values[0], values[1], values[2]);
        }
        
        public static Vector3[] ToVector3(Vector3[] values)
        {
            for (int i = 0; i < values.Length; i++)
            {
                values[i] = ToVector3(values[i]);
            }

            return values;
        }
        
        public static Quaternion ToQuaternion(Quaternion values)
        {
            return new Quaternion(values[0], -values[1], -values[2], values[3]);
        }
        
        public static Quaternion[] ToQuaternion(Quaternion[] values)
        {
            for (int i = 0; i < values.Length; i++)
            {
                values[i] = ToQuaternion(values[i]);
            }

            return values;
        }
    }
    
    public class MotionMatchingAPI
    {
        [DllImport ("MotionMatchingDll")]
        public static extern IntPtr MM_CreateController();
        
        [DllImport ("MotionMatchingDll")]
        public static extern void MM_DeleteController(IntPtr controller);
        
        [DllImport ("MotionMatchingDll")]
        public static extern bool MM_InitController(IntPtr controller, string path);

        [DllImport ("MotionMatchingDll")]
        public static extern void MM_UpdateController(IntPtr controller, float dt);

        [DllImport ("MotionMatchingDll")]
        public static extern void MM_MoveController(IntPtr controller, float x, float y, float z);

        [DllImport("MotionMatchingDll")]
        public static extern void MM_ControllerSetInput(IntPtr controller, IntPtr input);

        public static void MM_ControllerSetInput(IntPtr controller, ControllerInput input)
        {
            int sizeOut = Marshal.SizeOf(typeof(ControllerInput));
            IntPtr pBuf = Marshal.AllocHGlobal(sizeOut);
            Marshal.StructureToPtr(input, pBuf, true);
            MM_ControllerSetInput(controller, pBuf);
        }

        [DllImport("MotionMatchingDll")]
        public static extern void MM_ControllerGetInput(IntPtr controller, IntPtr input);

        public static ControllerInput MM_ControllerGetInput(IntPtr controller)
        {
            int sizeOut = Marshal.SizeOf(typeof(ControllerInput));
            IntPtr pBuf = Marshal.AllocHGlobal(sizeOut);
            MM_ControllerGetInput(controller, pBuf);
            ControllerInput input = (ControllerInput)Marshal.PtrToStructure(pBuf, typeof(ControllerInput));
            Marshal.FreeHGlobal(pBuf);
            return input;
        }
        
        [DllImport("MotionMatchingDll")]
        public static extern void MM_ControllerGetOutput(IntPtr controller, IntPtr output);

        public static ControllerOutput MM_ControllerGetOutput(IntPtr controller)
        {
            int sizeOut = Marshal.SizeOf(typeof(ControllerOutput));
            IntPtr pBuf = Marshal.AllocHGlobal(sizeOut);
            MM_ControllerGetOutput(controller, pBuf);
            ControllerOutput output = (ControllerOutput)Marshal.PtrToStructure(pBuf, typeof(ControllerOutput));
            
            output.simulation_position = MMUtils.ToVector3(output.simulation_position);
            output.simulation_rotation = MMUtils.ToQuaternion(output.simulation_rotation);
            output.bone_positions = MMUtils.ToVector3(output.bone_positions);
            output.bone_rotations = MMUtils.ToQuaternion(output.bone_rotations);
            output.global_bone_positions = MMUtils.ToVector3(output.global_bone_positions);
            output.global_bone_rotations = MMUtils.ToQuaternion(output.global_bone_rotations);
            output.trajectory_positions = MMUtils.ToVector3(output.trajectory_positions);
            output.trajectory_rotations = MMUtils.ToQuaternion(output.trajectory_rotations);

            Marshal.FreeHGlobal(pBuf);
            return output;
        }
        
        
        [DllImport("MotionMatchingDll")]
        public static extern void MM_ControllerGetPose(IntPtr controller, IntPtr output);

        public static ControllerPose MM_ControllerGetPose(IntPtr controller)
        {
            int sizeOut = Marshal.SizeOf(typeof(ControllerPose));
            IntPtr pBuf = Marshal.AllocHGlobal(sizeOut);
            MM_ControllerGetPose(controller, pBuf);
            ControllerPose output = (ControllerPose)Marshal.PtrToStructure(pBuf, typeof(ControllerPose));

            output.bone_rest_positions = MMUtils.ToVector3(output.bone_rest_positions);
            output.bone_rest_rotations = MMUtils.ToQuaternion(output.bone_rest_rotations);

            Marshal.FreeHGlobal(pBuf);
            return output;
        }
    }
}
using System;
using System.Runtime.InteropServices;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;

namespace FollowMe.Runtime.MotionMatching
{
    
    public class MotionMatchingController : MonoBehaviour
    {
        private IntPtr controller;
        private bool isInit;
        
        public GameObject targetAvatar;
        
        public string resourcePath;
        public ControllerInput controllerInput;
        private ControllerOutput controllerOutput;
        private ControllerPose controllerPose;

        private MotionMatchingRetarget motionMatchingRetarget;
        
        public MotionMatchingController()
        {
            isInit = false;
            controllerInput.Reset();
        }

        void CreateController()
        {
            controller = MotionMatchingAPI.MM_CreateController();
            isInit = MotionMatchingAPI.MM_InitController(controller, resourcePath);
            if (isInit)
            {
                controllerPose = MotionMatchingAPI.MM_ControllerGetPose(controller);
                controllerInput = MotionMatchingAPI.MM_ControllerGetInput(controller);

                motionMatchingRetarget = new MotionMatchingRetarget();
                motionMatchingRetarget.Init(targetAvatar, controllerPose);
            }
        }

        void DeleteController()
        {
            if (controller != IntPtr.Zero)
            {
                MotionMatchingAPI.MM_DeleteController(controller);
            }

            motionMatchingRetarget = null;
        }

        private void Start()
        {
            CreateController();
        }

        private void OnDestroy()
        {
            DeleteController();
        }
        void OnEnable()
        {
            AssemblyReloadEvents.beforeAssemblyReload += OnBeforeAssemblyReload;
            AssemblyReloadEvents.afterAssemblyReload += OnAfterAssemblyReload;
        }

        void OnDisable()
        {
            AssemblyReloadEvents.beforeAssemblyReload -= OnBeforeAssemblyReload;
            AssemblyReloadEvents.afterAssemblyReload -= OnAfterAssemblyReload;
        }

        public void OnBeforeAssemblyReload()
        {
            DeleteController();
        }

        public void OnAfterAssemblyReload()
        {
            CreateController();
        }

        private void Update()
        {
            if (controller == IntPtr.Zero || !isInit)
            {
                return;
            }
            
            MotionMatchingAPI.MM_ControllerSetInput(controller, controllerInput);
            
            Vector3 moveData = GetMoveData();
            MotionMatchingAPI.MM_MoveController(controller, moveData.x, moveData.y, moveData.z);
            MotionMatchingAPI.MM_UpdateController(controller, Time.deltaTime);

            controllerOutput = MotionMatchingAPI.MM_ControllerGetOutput(controller);

            if (motionMatchingRetarget != null)
            {
                motionMatchingRetarget.Update(controllerOutput);
            }
        }
        
        void OnDrawGizmos()
        {
            if (controller == IntPtr.Zero || !isInit)
            {
                return;
            }

            Vector3 simulation_position = controllerOutput.simulation_position;
            Quaternion simulation_rotation = controllerOutput.simulation_rotation;
            
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(simulation_position, 0.05f);
            Gizmos.DrawRay(simulation_position, simulation_rotation * Vector3.forward);

            Gizmos.color = Color.red;
            int trajectory_length = controllerOutput.trajectory_length;
            for (int i = 1; i < trajectory_length; i++)
            {
                Gizmos.DrawWireSphere(controllerOutput.trajectory_positions[i], 0.05f);
                Gizmos.DrawRay(controllerOutput.trajectory_positions[i], controllerOutput.trajectory_rotations[i] * Vector3.forward);
                Gizmos.DrawLine(controllerOutput.trajectory_positions[i-1], controllerOutput.trajectory_positions[i]);
            }
            
            Gizmos.color = Color.green;
            int bone_length = controllerOutput.bone_length;
            for (int i = 0; i < bone_length; i++)
            {
                Gizmos.DrawWireSphere(controllerOutput.global_bone_positions[i], 0.05f);
            }
            
            
            // Gizmos.color = Color.blue;
            // for (int i = 0; i < bone_length; i++)
            // {
            //     Gizmos.DrawWireSphere(controllerPose.bone_rest_positions[i], 0.05f);
            // }
            //
            //
            // Gizmos.color = Color.cyan;
            // for (int i = 0; i < bone_length; i++)
            // {
            //     ;
            //     Gizmos.DrawWireSphere(motionMatchingRetarget.targetAvatarAnimator.boneTransforms[i].initPosition, 0.05f);
            // }
        }

        
        //---------------------------------------------------------------
        
        
        
        //---------------------------------------------------------------
        
        Vector3 GetMoveData(float deadzone = 0.2f)
        {
            float gamepadx = 0;
            float gamepady = 0;

            if (Input.GetKey(KeyCode.W))
            {
                gamepady = -1;
            }else if (Input.GetKey(KeyCode.S))
            {
                gamepady = 1;
            }

            if (Input.GetKey(KeyCode.A))
            {
                gamepadx = -1;
            }else if (Input.GetKey(KeyCode.D))
            {
                gamepadx = 1;
            }
            float scalex = Input.GetKey(KeyCode.LeftShift) ? 1.0f : 0.5f;
            float scaley = Input.GetKey(KeyCode.LeftShift) ? 1.0f : 0.5f;
            
            gamepadx *= scalex;
            gamepady *= scaley;

            float gamepadmag = MathF.Sqrt(gamepadx*gamepadx + gamepady*gamepady);
    
            if (gamepadmag > deadzone)
            {
                float gamepaddirx = gamepadx / gamepadmag;
                float gamepaddiry = gamepady / gamepadmag;
                float gamepadclippedmag = gamepadmag > 1.0f ? 1.0f : gamepadmag*gamepadmag;
                gamepadx = gamepaddirx * gamepadclippedmag;
                gamepady = gamepaddiry * gamepadclippedmag;
            }
            else
            {
                gamepadx = 0.0f;
                gamepady = 0.0f;
            }
    
            return new Vector3(gamepadx, 0.0f, gamepady);
        }
    }
}
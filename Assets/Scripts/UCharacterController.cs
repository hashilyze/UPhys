using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UPhys
{
    public enum EState
    {
        Default, // Based on gravity
        Custom, // User defined movement
    }

    public class UCharacterController : MonoBehaviour
    {
        #region Public
        public void OnCharacterHit ()
        {

        }
        // Called when character is grounded but previous tick is not
        public void OnLand ()
        {
            RefillMoreJump();
        }
        public void OnLeaveGround ()
        {

        }

        //public System.Action<UCharacterController> OnLand;
        // Called when character is falling but previous tick is not
        public System.Action<UCharacterController> OnFall;

        public void SetPlayerInput (InputSet playerInput)
        {
            m_inputDirection = new Vector3(playerInput.moveAxis.x, 0.0f, playerInput.moveAxis.y);
                
        }

        public void SetAIInput(InputSet aiInput) { }

        public void DoJump ()
        {
            m_requestedJump = true;
            m_elapsedRequestedJump = 0.0f;
            m_requestedJumpSpeed = Mathf.Sqrt(2.0f * m_gravity * m_maxJumpDistance);
        }

        public void StopJump ()
        {
            m_requestedStopJump = true;
        }

        /// <summary>
        /// Refill more jump count; default is full
        /// </summary>
        public void RefillMoreJump (int count = int.MaxValue)
        {
            m_leftMoreJumpCount = Mathf.Clamp(count, 0, m_moreJumpCount);
        }

        public void UpdateVelocity (float deltaTime, ref Vector3 velocity, UCharacterMovement movement)
        {
            switch (m_state)
            {
            case EState.Default:
                UpdateVelocityDefault(deltaTime, ref velocity, movement);
                break;
            case EState.Custom:
                break;
            default:
                Debug.LogError("Undefined state");
                break;
            }
        }

        public void UpdateRotation (float deltaTime, ref Quaternion rotation, UCharacterMovement movement)
        {
            switch (m_state)
            {
            case EState.Default:
                // Character could look toward movement direction
                if (movement.Velocity != Vector3.zero)
                {

                    Vector3 horizontalDirection = Vector3.ProjectOnPlane(movement.Velocity, movement.CharacterUp).normalized;
                    if (horizontalDirection != Vector3.zero)
                    {
                        Quaternion targetRotation = Quaternion.LookRotation(horizontalDirection, movement.CharacterUp);
                        rotation = Quaternion.RotateTowards(rotation, targetRotation, m_orientSpeed * deltaTime);
                    }
                }
                break;
            case EState.Custom:
                break;
            default:
                Debug.LogError("Undefined state");
                break;
            }
        }
        #endregion

        [Header("Base")]
        [SerializeField] private EState m_state = EState.Default;

        [Header("Stble Ground Movement")]
        [Tooltip("Maximum speed as move on ground")]
        [SerializeField] private float m_maxSpeed = 5.0f;
        [Tooltip("Reach max speed fastly if higher")]
        [SerializeField] private float m_acceleration = float.PositiveInfinity;
        [Tooltip("Break velocity to zero fastly if higher")]
        [SerializeField] private float m_friction = float.PositiveInfinity;
        [Tooltip("Min distance to move character")]
        [SerializeField] private float m_minDistance = 0.01f;
        [Tooltip("Rotate speed by degree toward move direction")]
        [SerializeField] private float m_orientSpeed = 30.0f;

        [Header("Air Movement")]
        [Tooltip("Maximum speed as move on air")]
        [SerializeField] private float m_maxAirSpeed = 5.0f;
        [Tooltip("Fast to reach MaxSpeed")]
        [SerializeField] private float m_airAcceleration = 10.0f;
        [Tooltip("Fast to reach stop(zero velocity)")]
        [SerializeField] private float m_drag = float.PositiveInfinity;
        [Tooltip("Limit applied garvity")]
        [SerializeField] private float m_maxFallSpeed = 10.0f;

        [Header("Jump")]
        [Tooltip("Maximum distance as jump without obstacle")]
        [SerializeField] private float m_maxJumpDistance = 5.0f;
        //[Tooltip("Minimum distance as jump without obstacle")]
        //[SerializeField] private float m_minJumpDistance = 1.0f;
        [Tooltip("Speed when jump cancealed")]
        [SerializeField] private float m_variableJumpSpeed = 0.0f;
        [Tooltip("Delay cancel jump until time over")]
        [SerializeField] private float m_minJumpTime = 0.02f;

        [Tooltip("How many do jumping on air")]
        [SerializeField] private int m_moreJumpCount = 1;
        [ReadOnly] [SerializeField] private int m_leftMoreJumpCount;

        [Tooltip("Jump a few time later after ungrounded")]
        [SerializeField] private float m_coyoteTime = 0.05f;
        [Tooltip("Kepp jump request if failed")]
        [SerializeField] private float m_jumpBuffer = 0.05f;

        [Tooltip("Extend hang up time if curved jump")]
        [SerializeField] private float m_halfGravityThresdhold = 0.5f;
        private bool m_doJump = false;
        private float m_elapsedJumpTime = 0.0f;

        [Header("Gravity")]
        [Tooltip("Apply gravity as character is on airbone")]
        [SerializeField] private bool m_useGravity = true;
        [Tooltip("Sacle of gravity without any weights; Higher falls faster")]
        [SerializeField] private float m_gravity = 10.0f;
        [Tooltip("Smooth feel of fall")]
        [SerializeField] private float m_fallGravityWeights = 1.2f;

        // Requested values
        // Move
        private Vector3 m_inputDirection;
        // Jump
        private float m_requestedJumpSpeed = 0.0f;
        private bool m_requestedJump = false;
        private float m_elapsedRequestedJump = 0.0f;
        private bool m_requestedStopJump = false;

        private void OnEnable ()
        {
            InputManager.RegisterPlayer(this);
        }
        private void OnDisable ()
        {
            InputManager.UnregisterPlayer(this);
        }

        private void UpdateVelocityDefault (float deltaTime, ref Vector3 velocity, UCharacterMovement movement)
        {
            // Movement on stable ground
            if (movement.GroundReport.HitAnyGround && movement.GroundReport.IsStable)
            {
                // Reorient last velocity 
                Vector3 tangent = Vector3.Cross(movement.GroundReport.Normal, Vector3.Cross(velocity, movement.CharacterUp)).normalized;
                Vector3 reorientVelocity = velocity.magnitude * tangent;

                // Accelerate movement
                if (m_inputDirection != Vector3.zero)
                {
                    // Reorient input direction to surface
                    Vector3 newDirection = Vector3.Cross(movement.GroundReport.Normal, Vector3.Cross(m_inputDirection, movement.CharacterUp)).normalized;
                    velocity = Vector3.Lerp(reorientVelocity, m_maxSpeed * newDirection, 1.0f - Mathf.Exp(-m_acceleration * deltaTime));
                }
                // Break movement
                else
                {
                    if (velocity != Vector3.zero)
                    {
                        velocity = Vector3.Lerp(reorientVelocity, Vector3.zero, 1.0f - Mathf.Exp(-m_friction * deltaTime));
                        // Break completely with discard velocity when lower than min distance
                        if (velocity.sqrMagnitude * deltaTime * deltaTime < m_minDistance * m_minDistance)
                        {
                            velocity = Vector3.zero;
                        }
                    }
                }
            }
            // Movement on air
            else
            {
                Vector3 horizontalVelocity = Vector3.ProjectOnPlane(velocity, movement.CharacterUp);
                // Horizontal velocity
                {
                    if (m_inputDirection != Vector3.zero)
                    {
                        horizontalVelocity = Vector3.Lerp(horizontalVelocity, m_maxAirSpeed * m_inputDirection, 1.0f - Mathf.Exp(-m_airAcceleration * deltaTime));
                    }
                    else
                    {
                        if (horizontalVelocity != Vector3.zero)
                        {
                            horizontalVelocity = Vector3.Lerp(horizontalVelocity, Vector3.zero, 1.0f - Mathf.Exp(-m_drag * deltaTime));
                            if (horizontalVelocity.sqrMagnitude * deltaTime * deltaTime < m_minDistance * m_minDistance)
                            {
                                horizontalVelocity = Vector3.zero;
                            }
                        }
                    }
                }
                // Apply Gravity
                Vector3 verticalVelocity;
                // Vertical velocity
                {
                    float verticalSpeed = Vector3.Dot(velocity, movement.CharacterUp);

                    if (m_useGravity)
                    {
                        verticalSpeed += deltaTime * -m_gravity
                            * (verticalSpeed < 0.0f ? m_fallGravityWeights : 1.0f)
                            * (Mathf.Abs(verticalSpeed) <= m_halfGravityThresdhold ? 0.5f : 1.0f);
                    }

                    verticalSpeed = Mathf.Max(verticalSpeed, -m_maxFallSpeed);
                    verticalVelocity = verticalSpeed * movement.CharacterUp;

                    if (verticalSpeed < 0.0f)
                    {
                        m_doJump = false;

                        OnFall?.Invoke(this);
                    }
                }

                velocity = horizontalVelocity + verticalVelocity;
            }

            HandleJump(deltaTime, ref velocity, movement);
        }

        private void HandleJump (float deltaTime, ref Vector3 velocity, UCharacterMovement movement)
        {
            if (m_doJump)
            {
                m_elapsedJumpTime += deltaTime;
            }

            // Try jump up
            if (m_requestedJump)
            {
                // Check character can jump
                bool canJump = false;
                if (movement.GroundReport.HitAnyGround && movement.GroundReport.IsStable || movement.GroundReport.ElapsedUngroundTime <= m_coyoteTime)
                {
                    // Jump when on stable ground or a few frame later ungrounded
                    canJump = true;
                }
                else if (m_leftMoreJumpCount > 0)
                {
                    // Comsume more jump for jumping
                    canJump = true;
                    --m_leftMoreJumpCount;
                }

                if (canJump)
                {
                    // Consume Jump
                    m_requestedJump = false;
                    m_elapsedRequestedJump = 0.0f;

                    m_doJump = true;
                    m_elapsedJumpTime = 0.0f;

                    // Discard old vertical velocity and set jump velocity
                    //velocity = Vector3.ProjectOnPlane(velocity, movement.CharacterUp) + movement.CharacterUp * Mathf.Sqrt(2.0f * m_gravity * m_maxJumpDistance);
                    velocity = Vector3.ProjectOnPlane(velocity, movement.CharacterUp) + movement.CharacterUp * m_requestedJumpSpeed;

                    // Conserve momentum when jump on movor
                    if (movement.GroundReport.HitAnyGround && movement.GroundReport.IsStable)
                    {
                        Rigidbody rb = movement.GroundReport.Collider.attachedRigidbody;
                        if (rb != null && rb.isKinematic)
                        {
                            if (Vector3.Dot(rb.velocity, velocity) >= 0.0f)
                            {
                                velocity += rb.velocity;
                            }
                        }
                    }

                    // Prevent snapping when jump
                    movement.ForceUnground();
                }

                // Update Jump Buffer
                {
                    m_elapsedRequestedJump += deltaTime;
                    if (m_elapsedRequestedJump > m_jumpBuffer)
                    {
                        m_requestedJump = false;
                        m_elapsedRequestedJump = 0.0f;
                    }
                }
            }

            // Break Jump; Force falling
            if (m_requestedStopJump && m_elapsedJumpTime >= m_minJumpTime)
            {
                m_requestedStopJump = false;

                if (m_doJump)
                {
                    m_doJump = false;

                    // Force falling by increasing gravity
                    //float init = Mathf.Sqrt(2.0f * m_gravity * m_maxJumpDistance);
                    //float currentJumpHeight = init * m_elapsedJumpTime + 0.5f * -m_gravity * m_elapsedJumpTime * m_elapsedJumpTime;
                    //float forceFallingWeights = Mathf.Lerp(m_maxJumpDistance / m_minJumpDistance, 1.0f, currentJumpHeight / m_maxJumpDistance);
                    //m_gravityWeights = forceFallingWeights;

                    // Force falling by setting vertical velocity
                    velocity = Vector3.ProjectOnPlane(velocity, movement.CharacterUp) + Mathf.Min(Vector3.Dot(velocity, movement.CharacterUp), m_variableJumpSpeed) * movement.CharacterUp;
                }
            }
        }
    }
}
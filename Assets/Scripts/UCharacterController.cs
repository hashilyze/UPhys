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
        public void OnCharacterHit () { }
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
            _inputDirection = new Vector3(playerInput.moveAxis.x, 0.0f, playerInput.moveAxis.y);
                
        }

        public void SetAIInput(InputSet aiInput) { }

        public void DoJump ()
        {
            _requestedJump = true;
            _elapsedRequestedJump = 0.0f;
            _requestedJumpSpeed = Mathf.Sqrt(2.0f * _gravity * _maxJumpDistance);
        }
        public void StopJump ()
        {
            _requestedStopJump = true;
        }

        /// <summary>Refill more jump count; default is full</summary>
        public void RefillMoreJump (int count = int.MaxValue)
        {
            _leftMoreJumpCount = Mathf.Clamp(count, 0, _moreJumpCount);
        }

        public void UpdateVelocity (float deltaTime, ref Vector3 velocity, UCharacterMovement movement)
        {
            switch (_state)
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
            switch (_state)
            {
            case EState.Default:
                // Character could look toward movement direction
                if (movement.Velocity != Vector3.zero)
                {

                    Vector3 horizontalDirection = Vector3.ProjectOnPlane(movement.Velocity, movement.CharacterUp).normalized;
                    if (horizontalDirection != Vector3.zero)
                    {
                        Quaternion targetRotation = Quaternion.LookRotation(horizontalDirection, movement.CharacterUp);
                        rotation = Quaternion.RotateTowards(rotation, targetRotation, _orientSpeed * deltaTime);
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
        [SerializeField] private EState _state = EState.Default;

        [Header("Stble Ground Movement")]
        [Tooltip("Maximum speed as move on ground")]
        [SerializeField] private float _maxSpeed = 5.0f;
        [Tooltip("Reach max speed fastly if higher")]
        [SerializeField] private float _acceleration = float.PositiveInfinity;
        [Tooltip("Break velocity to zero fastly if higher")]
        [SerializeField] private float _friction = float.PositiveInfinity;
        [Tooltip("Min distance to move character")]
        [SerializeField] private float _minDistance = 0.01f;
        [Tooltip("Rotate speed by degree toward move direction")]
        [SerializeField] private float _orientSpeed = 30.0f;

        [Header("Air Movement")]
        [Tooltip("Maximum speed as move on air")]
        [SerializeField] private float _maxAirSpeed = 5.0f;
        [Tooltip("Fast to reach MaxSpeed")]
        [SerializeField] private float _airAcceleration = 10.0f;
        [Tooltip("Fast to reach stop(zero velocity)")]
        [SerializeField] private float _drag = float.PositiveInfinity;
        [Tooltip("Limit applied garvity")]
        [SerializeField] private float _maxFallSpeed = 10.0f;

        [Header("Jump")]
        [Tooltip("Maximum distance as jump without obstacle")]
        [SerializeField] private float _maxJumpDistance = 5.0f;
        //[Tooltip("Minimum distance as jump without obstacle")]
        //[SerializeField] private float _minJumpDistance = 1.0f;
        [Tooltip("Speed when jump cancealed")]
        [SerializeField] private float _variableJumpSpeed = 0.0f;
        [Tooltip("Delay cancel jump until time over")]
        [SerializeField] private float _minJumpTime = 0.02f;

        [Tooltip("How many do jumping on air")]
        [SerializeField] private int _moreJumpCount = 1;
        [ReadOnly] [SerializeField] private int _leftMoreJumpCount;

        [Tooltip("Jump a few time later after ungrounded")]
        [SerializeField] private float _coyoteTime = 0.05f;
        [Tooltip("Kepp jump request if failed")]
        [SerializeField] private float _jumpBuffer = 0.05f;

        [Tooltip("Extend hang up time if curved jump")]
        [SerializeField] private float _halfGravityThresdhold = 0.5f;
        private bool _doJump = false;
        private float _elapsedJumpTime = 0.0f;

        [Header("Gravity")]
        [Tooltip("Apply gravity as character is on airbone")]
        [SerializeField] private bool _useGravity = true;
        [Tooltip("Sacle of gravity without any weights; Higher falls faster")]
        [SerializeField] private float _gravity = 10.0f;
        [Tooltip("Smooth feel of fall")]
        [SerializeField] private float _fallGravityWeights = 1.2f;

        // Requested values
        // Move
        private Vector3 _inputDirection;
        // Jump
        private float _requestedJumpSpeed = 0.0f;
        private bool _requestedJump = false;
        private float _elapsedRequestedJump = 0.0f;
        private bool _requestedStopJump = false;

        // Life cycle management
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
                if (_inputDirection != Vector3.zero)
                {
                    // Reorient input direction to surface
                    Vector3 newDirection = Vector3.Cross(movement.GroundReport.Normal, Vector3.Cross(_inputDirection, movement.CharacterUp)).normalized;
                    velocity = Vector3.Lerp(reorientVelocity, _maxSpeed * newDirection, 1.0f - Mathf.Exp(-_acceleration * deltaTime));
                }
                // Break movement
                else
                {
                    if (velocity != Vector3.zero)
                    {
                        velocity = Vector3.Lerp(reorientVelocity, Vector3.zero, 1.0f - Mathf.Exp(-_friction * deltaTime));
                        // Break completely with discard velocity when lower than min distance
                        if (velocity.sqrMagnitude * deltaTime * deltaTime < _minDistance * _minDistance)
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
                    if (_inputDirection != Vector3.zero)
                    {
                        horizontalVelocity = Vector3.Lerp(horizontalVelocity, _maxAirSpeed * _inputDirection, 1.0f - Mathf.Exp(-_airAcceleration * deltaTime));
                    }
                    else
                    {
                        if (horizontalVelocity != Vector3.zero)
                        {
                            horizontalVelocity = Vector3.Lerp(horizontalVelocity, Vector3.zero, 1.0f - Mathf.Exp(-_drag * deltaTime));
                            if (horizontalVelocity.sqrMagnitude * deltaTime * deltaTime < _minDistance * _minDistance)
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

                    if (_useGravity)
                    {
                        verticalSpeed += deltaTime * -_gravity
                            * (verticalSpeed < 0.0f ? _fallGravityWeights : 1.0f)
                            * (Mathf.Abs(verticalSpeed) <= _halfGravityThresdhold ? 0.5f : 1.0f);
                    }

                    verticalSpeed = Mathf.Max(verticalSpeed, -_maxFallSpeed);
                    verticalVelocity = verticalSpeed * movement.CharacterUp;

                    if (verticalSpeed < 0.0f)
                    {
                        _doJump = false;

                        OnFall?.Invoke(this);
                    }
                }

                velocity = horizontalVelocity + verticalVelocity;
            }

            HandleJump(deltaTime, ref velocity, movement);
        }

        private void HandleJump (float deltaTime, ref Vector3 velocity, UCharacterMovement movement)
        {
            if (_doJump)
            {
                _elapsedJumpTime += deltaTime;
            }

            // Try jump up
            if (_requestedJump)
            {
                // Check character can jump
                bool canJump = false;
                if (movement.GroundReport.HitAnyGround && movement.GroundReport.IsStable || movement.GroundReport.ElapsedUngroundTime <= _coyoteTime)
                {
                    // Jump when on stable ground or a few frame later ungrounded
                    canJump = true;
                }
                else if (_leftMoreJumpCount > 0)
                {
                    // Comsume more jump for jumping
                    canJump = true;
                    --_leftMoreJumpCount;
                }

                if (canJump)
                {
                    // Consume Jump
                    _requestedJump = false;
                    _elapsedRequestedJump = 0.0f;

                    _doJump = true;
                    _elapsedJumpTime = 0.0f;

                    // Discard old vertical velocity and set jump velocity
                    //velocity = Vector3.ProjectOnPlane(velocity, movement.CharacterUp) + movement.CharacterUp * Mathf.Sqrt(2.0f * _gravity * _maxJumpDistance);
                    velocity = Vector3.ProjectOnPlane(velocity, movement.CharacterUp) + movement.CharacterUp * _requestedJumpSpeed;

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
                    _elapsedRequestedJump += deltaTime;
                    if (_elapsedRequestedJump > _jumpBuffer)
                    {
                        _requestedJump = false;
                        _elapsedRequestedJump = 0.0f;
                    }
                }
            }

            // Break Jump; Force falling
            if (_requestedStopJump && _elapsedJumpTime >= _minJumpTime)
            {
                _requestedStopJump = false;

                if (_doJump)
                {
                    _doJump = false;

                    // Force falling by increasing gravity
                    //float init = Mathf.Sqrt(2.0f * _gravity * _maxJumpDistance);
                    //float currentJumpHeight = init * _elapsedJumpTime + 0.5f * -_gravity * _elapsedJumpTime * _elapsedJumpTime;
                    //float forceFallingWeights = Mathf.Lerp(_maxJumpDistance / _minJumpDistance, 1.0f, currentJumpHeight / _maxJumpDistance);
                    //_gravityWeights = forceFallingWeights;

                    // Force falling by setting vertical velocity
                    velocity = Vector3.ProjectOnPlane(velocity, movement.CharacterUp) + Mathf.Min(Vector3.Dot(velocity, movement.CharacterUp), _variableJumpSpeed) * movement.CharacterUp;
                }
            }
        }
    }
}
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UPhys
{
    [System.Serializable]
    public struct GroundHitReport
    {
        public bool HitAnyGround { get => _hitAnyGround; set => _hitAnyGround = value; }
        public bool IsStable { get => _isStable; set => _isStable = value; }

        public Collider Collider { get => _collider; set => _collider = value; }
        public Vector3 Point { get => _point; set => _point = value; }
        public Vector3 Normal { get => _normal; set => _normal = value; }
        public float Angle { get => _angle; set => _angle = value; }
        public float Distance { get => _distance; set => _distance = value; }

        public float ElapsedUngroundTime { get => _elapsedUngroundTime; set => _elapsedUngroundTime = value; }


        [Tooltip("Whether detect ground regardless with stablity")]
        [SerializeField] private bool _hitAnyGround;
        [Tooltip("Whether detected ground is stabe, character doesn't slide on")]
        [SerializeField] private bool _isStable;
        [SerializeField] private Collider _collider;
        [SerializeField] private Vector3 _point;
        [SerializeField] private Vector3 _normal;
        [SerializeField] private float _angle;
        [SerializeField] private float _distance;
        [SerializeField] private float _elapsedUngroundTime;
    }

    public class UCharacterMovement : MonoBehaviour
    {
        #region Public
        public Vector3 Velocity => _velocity;
        public float Mass => _mass;
        public Vector3 CharacterUp => _nextCharacterUp;

        public bool UseGroundSnap { get => _useGroundSnap; set => _useGroundSnap = value; }
        public float StableAngle { get => _stableAngle; set => _stableAngle = Mathf.Clamp(value, 0.0f, 90.0f); }
        public float StepOffset { get => _stepOffset; set => _stepOffset = value; }
        public GroundHitReport GroundReport => _groundReport;

        public LayerMask CollidableMask { get => _collidableMask; set => _collidableMask = value; }
        public LayerMask BlockMask { get => _blockMask; set => _blockMask = value; }

        /// <summary>Move character independently with velocity</summary>
        public void Move (Vector3 distance)
        {
            _additionalMoveDistance += distance;
        }

        public void Bounce (Vector3 distance, float time) { }

        /// <summary>Place character to destination and fix position during this frame</summary>
        public void Teleport (Vector3 destination)
        {
            _requestedTeleport = true;
            _teleportDestination = destination;
        }
        /// <summary>Place character to ground with closest to destination</summary>
        public void TeleportUponGround(Vector3 destination, float snapDistance = int.MaxValue) { }

        public void Look (Vector3 direction) { }

        /// <summary>Detach from ground and skip ground handling until time over</summary>
        public void ForceUnground (float time = 0.1f)
        {
            _isForceUnground = true;
            _leftUngroundTime = time;
        }

        public void Simulate (float deltaTime)
        {
            // Initialize transform
            {
                _nextPos = _initPos = _rb.position;
                _nextRot = _initRot = _rb.rotation;
                _nextCharacterUp = _nextRot * Vector3.up;
            }
            // Teleport: Set Position
            if (_requestedTeleport)
            {
                _requestedTeleport = false;
                ConsumeTeleport();
                return;
            }
            // Handle Riding
            if (_useRiding)
            {
                HandleRiding(deltaTime);
            }
            // Overlap Recovery
            if (_handleStuck)
            {
                SolveOverlap();
            }
            // Update Velocity
            {
                Vector3 distance;

                _characterController.UpdateVelocity(deltaTime, ref _velocity, this);
                distance = _velocity * deltaTime;

                distance += _additionalMoveDistance;
                _additionalMoveDistance = Vector3.zero;

                InternalSafeMoveWithSlide(distance);
            }
            // Update Look
            {
                _characterController.UpdateRotation(deltaTime, ref _nextRot, this);
                _nextCharacterUp = _nextRot * Vector3.up;
            }
            // Probe Ground and Snap
            if (_useHandleGround)
            {
                ProbeGround(deltaTime);
            }
            // Move Position and Rotation
            {
                // Update transform in physics scene to inform transform
                _rb.position = _nextPos;
                _rb.rotation = _nextRot;

                transform.position = _nextPos;
                transform.rotation = _nextRot;
            }
        }

        public void SimulateCommit ()
        {
            // Rollback transform as before simulate
            _rb.position = _initPos;
            _rb.rotation = _initRot;
            // Use movement funtion instead directly set raw transform because simulate with dynamic rigidbody
            _rb.MovePosition(_nextPos);
            _rb.MoveRotation(_nextRot);
        }
        #endregion

        #region Private
        [Header("Base")]
        [ReadOnly][SerializeField] private Vector3 _velocity;
        [SerializeField] private float _mass = 100.0f;

        [Header("Ground Setting")]
        [Tooltip("Use ground interaction")]
        [SerializeField] private bool _useHandleGround = true;
        [Tooltip("Snap character to ledge when bounce on")]
        [SerializeField] private bool _useGroundSnap = true;
        [Tooltip("Character could climb on slope")]
        [Range(0.0f, 90.0f)][SerializeField] private float _stableAngle = 50.0f;
        [Tooltip("Character could ignore blocking and step up/down")]
        [SerializeField] private float _stepOffset = 0.2f;
        [ReadOnly][SerializeField] private GroundHitReport _groundReport;
        // Whether skip probe ground
        private bool _isForceUnground = false;
        // Skip probe ground until left time zero
        private float _leftUngroundTime = 0.0f;

        [Header("Riding")]
        [SerializeField] private bool _useRiding = true;
        [ReadOnly][SerializeField] private Rigidbody _riding;
        [ReadOnly][SerializeField] private Vector3 _ridingContactPoint;
        private bool _inHandleRiding = false;

        [Header("Misc")]
        [SerializeField] private bool _handleStuck = true;
        [Tooltip("Character can not pass though collider being layer")]
        [SerializeField] private LayerMask _collidableMask;
        [Tooltip("Character never pass though collider being layer")]
        [SerializeField] private LayerMask _blockMask;

        // Memory buffer
        private readonly Collider[] _colliderBuffer = new Collider[8];
        private readonly RaycastHit[] _hitInfoBuffer = new RaycastHit[8];
        // External Components
        private CapsuleCollider _body;
        private Rigidbody _rb;
        private UCharacterController _characterController;
        private CollisionHandler _collisionHandler;
        // Transform cache
        private Vector3 _initPos;
        private Quaternion _initRot;
        private Vector3 _nextPos;
        private Quaternion _nextRot;
        private Vector3 _nextCharacterUp;
        // Additional movement
        private Vector3 _additionalMoveDistance;
        // Teleport
        private Vector3 _teleportDestination;
        private bool _requestedTeleport = false;

        // Life cycle management
        private void Awake ()
        {
            // Initialize _body
            if(!TryGetComponent(out _body))
            {
                throw new System.NullReferenceException("");
            }
            _body.isTrigger = false;
            // Initialize _rb
            if (!TryGetComponent(out _rb))
            {
                throw new System.NullReferenceException("");
            }
            _rb.useGravity = false;
            _rb.isKinematic = true;
            _rb.constraints = RigidbodyConstraints.None;
            // Initialize _cct
            if (!TryGetComponent(out _characterController))
            {
                throw new System.NullReferenceException("");
            }
            // Initialize _collisionHandler
            if (!TryGetComponent(out _collisionHandler))
            {
                throw new System.NullReferenceException("");
            }
            _collisionHandler.Setup(_body);
        }
        private void OnEnable ()
        {
            UPhysSystem.RegisterCharacterMovement(this);
        }
        private void OnDisable ()
        {
            UPhysSystem.UnregisterCharacterMovement(this);
        }

        // Teleport handling
        private void ConsumeTeleport ()
        {
            _nextPos = _teleportDestination;
        }
        // Riding handling
        private void HandleRiding (float deltaTime)
        {
            // Validate ridings
            _riding = null;
            // Ride moving platform
            if (_groundReport.HitAnyGround && _groundReport.IsStable)
            {
                // Ground Object is removed or destoryed 
                if (_groundReport.Collider == null)
                {
                    return;
                }

                _riding = _groundReport.Collider.attachedRigidbody;
                _ridingContactPoint = _groundReport.Point;
            }

            if (_riding != null)
            {

                Vector3 snapDistance = Vector3.zero;
                if (_riding.TryGetComponent(out UPlatformController platform))
                {
                    // Get velocity from platform component
                    snapDistance = (platform.Velocity + Vector3.Cross(platform.AngularVelocity.eulerAngles, _ridingContactPoint - _riding.position)) * deltaTime;

                }
                if(false)
                {
                    // Get velocity from rigidbody
                    snapDistance = _riding.GetPointVelocity(_ridingContactPoint) * deltaTime;
                    //snapDistance = (_riding.velocity + Vector3.Cross(_riding.angularVelocity, _ridingContactPoint - _riding.position)) * deltaTime;
                }
                _inHandleRiding = true;
                InternalSafeMoveWithCollide(snapDistance, out Vector3 _, out RaycastHit _);
                _inHandleRiding = false;
            }
        }

        // Ground handling
        private void ClearGroundReport ()
        {
            _groundReport.HitAnyGround = false;

            _groundReport.Collider = null;
            _groundReport.Point = Vector3.zero;
            _groundReport.Distance = 0.0f;
            _groundReport.Normal = Vector3.zero;
            _groundReport.Angle = 0.0f;
            _groundReport.IsStable = false;
            _groundReport.ElapsedUngroundTime = 0.0f;
        }

        /// <summary>Update Timer for ForceUnground</summary>
        /// <returns>True when have left time after updated</returns>
        private bool UpdateForceUngroundTimer (float deltaTime)
        {
            return _isForceUnground && (_isForceUnground = (_leftUngroundTime -= deltaTime) > 0.0f);
        }

        private float GetSnapGroundDistance (float horizontalMovementDistance, float verticalMovementDistance, float groundAngle)
        {
            float snapDistance = horizontalMovementDistance * Mathf.Tan(groundAngle * Mathf.Deg2Rad);
            if(verticalMovementDistance > 0.0f)
            {
                snapDistance += verticalMovementDistance;
            }

            return snapDistance;
        }

        /// <summary>Check contacted ground</summary
        private void ProbeGround (float deltaTime)
        {
            if (UpdateForceUngroundTimer(deltaTime))
            {
                // Don't handle while timer actived
                ClearGroundReport();
                return;
            }

            // Before update grounding report, cache previous report; used for checking OnLanded
            bool wasStable = _groundReport.HitAnyGround && _groundReport.IsStable;

            _groundReport.HitAnyGround = false;
            _groundReport.Collider = null;

            float probeDistance = UPhysSettings.Instance.SkinWidth * 2.0f;

            Vector3 movementDistance = _nextPos - _initPos;
            float horizontalDistance = Vector3.ProjectOnPlane(movementDistance, _nextCharacterUp).magnitude;
            float verticalDistance = Vector3.Dot(movementDistance, _nextCharacterUp);

            // Snapable ground probing range
            float maxSnapDistance = 0.0f;
            if (wasStable)
            {
                maxSnapDistance = GetSnapGroundDistance(horizontalDistance, verticalDistance, _stableAngle);
                //maxSnapDistance = 0.2f;
            }

            // Cast to downward from bottom of character
            if (_collisionHandler.Sweep(_nextPos, -_nextCharacterUp, probeDistance + maxSnapDistance, _hitInfoBuffer, out RaycastHit closestHit, _collidableMask & _blockMask, IsValidCollider) > 0)
            {
                float angle = Vector3.Angle(_nextCharacterUp, closestHit.normal);

                // Accurated snapable range because of ground's angleFor
                float minSnapDistance = 0.0f; 
                // Don't snap when on steep ramp
                if (wasStable && angle <= _stableAngle)
                {
                    minSnapDistance = GetSnapGroundDistance(horizontalDistance, verticalDistance, angle);
                    //minSnapDistance = 0.2f;
                }
                // Grounded when on ground or snapable
                if (closestHit.distance <= probeDistance + minSnapDistance)
                {
                    _groundReport.HitAnyGround = true;

                    _groundReport.Collider = closestHit.collider;
                    _groundReport.Point = closestHit.point;
                    _groundReport.Distance = closestHit.distance;
                    _groundReport.Normal = closestHit.normal;
                    _groundReport.Angle = angle;
                    _groundReport.IsStable = angle <= _stableAngle;
                    _groundReport.ElapsedUngroundTime = 0.0f;
                }
            }

            // Events after ground probing
            // On Ground
            if (_groundReport.HitAnyGround && _groundReport.IsStable)
            {
                // Ground snapping
                if (_useGroundSnap)
                {
                    _nextPos -= (_groundReport.Distance - UPhysSettings.Instance.SkinWidth) * _nextCharacterUp;
                    _groundReport.Distance = UPhysSettings.Instance.SkinWidth;
                }

                // On Landed (first tick when grounded)
                if (!wasStable)
                {
                    // Discard vertical velocity
                    _velocity = Vector3.ProjectOnPlane(_velocity, _nextCharacterUp);

                    //_leftMoreJumpCount = _moreJumpCount;
                    //_doJump = false;
                    //OnLand?.Invoke(this);
                    _characterController.OnLand();
                }
            }
            else
            {
                _groundReport.ElapsedUngroundTime += deltaTime;
            }
        }

        private void SolveOverlap ()
        {
            // * How can I determine unrecoverible to stop shaking
            Vector3 backupPosition = _nextPos;

            int currentIteration = 0;
            int maxIteration = UPhysSettings.Instance.DepentrationIteration;
            while (currentIteration < maxIteration)
            {
                // If there are overlaped collider with character, detach character from these
                int overlapCount = _collisionHandler.Overlap(_nextPos, _colliderBuffer, _collidableMask, IsValidCollider);
                // If no more found overlaped collider
                if (overlapCount == 0)
                {
                    break;
                }
                for (int cur = 0; cur < overlapCount; ++cur)
                {
                    Collider overlapCollider = _colliderBuffer[cur];
                    
                    UPhysUtility.GetPosAndRot(overlapCollider, out Vector3 overlapPos, out Quaternion overlapRot);

                    // Depentrate if colliders are overlaped with deeper than zero
                    if (Physics.ComputePenetration(_body, _nextPos, _nextRot,
                        overlapCollider, overlapPos, overlapRot, out Vector3 dir, out float dist))
                    {
                        if(InternalSafeMoveWithCollide(dist * dir, out Vector3 remainingDistance, out RaycastHit hitInfo))
                        {
                            if (remainingDistance.sqrMagnitude > 0.0f)
                            {
                                // * Caution: Very very unstable process, need to more test; recommnad using squish instead of this soultion
                                // Slided push without projection of distance unless squished
                                if (Vector3.Angle(-dir, hitInfo.normal) > 5.0f)
                                {
                                    Vector3 tangent = GetTangent(dir, hitInfo.normal);

                                    float leftDistOnDir = dist - (hitInfo.distance - UPhysSettings.Instance.SkinWidth);
                                    float tangentOnDir = Vector3.Dot(tangent, dir);
                                    if (tangentOnDir > 0.001f)
                                    {
                                        float leftDistOnTangent = leftDistOnDir / tangentOnDir;

                                        InternalSafeMoveWithCollide(leftDistOnTangent * tangent, out Vector3 _, out RaycastHit _);
                                    }
                                }
                            }
                        }
                    }
                }
                ++currentIteration;
            }

            if (currentIteration >= maxIteration)
            {
                // Break overlap recovery because character probably located in unsafed zone
                if (UPhysSettings.Instance.KillPositionWhenExceedDepentrationIteration)
                {
                    _nextPos = backupPosition;
                }
            }
        }

        /// <summary>Move character and stop when collide</summary>
        private bool InternalSafeMoveWithCollide(Vector3 distance, out Vector3 remainingDistance, out RaycastHit hitInfo)
        {
            Vector3 direction = distance.normalized;
            float magnitude = distance.magnitude;
            // Character react to obstacles when crush
            if (_collisionHandler.Sweep(_nextPos, direction, magnitude + UPhysSettings.Instance.SkinWidth, _hitInfoBuffer, out RaycastHit closestHit, _collidableMask & _blockMask, IsValidCollider) > 0)
            {
                hitInfo = closestHit;
                // Move character until blocked
                closestHit.distance -= UPhysSettings.Instance.SkinWidth;
                _nextPos += closestHit.distance * direction;
                // Update remaining distance
                if ((magnitude -= closestHit.distance) < 0.0f)
                {
                    remainingDistance = Vector3.zero;
                    return false;
                }
                else
                {
                    remainingDistance = magnitude * direction;
                    return true;
                }
            }
            else
            {
                // There are no obstacles blocking movement
                _nextPos += distance;
                hitInfo = default;
                remainingDistance = Vector3.zero;
                return false;
            }
        }

        /// <summary>Move character and slide obstacle when collide</summary>
        private void InternalSafeMoveWithSlide (Vector3 distance)
        {
            // Discard unvalidated value
            if (float.IsNaN(distance.x) || float.IsNaN(distance.y) || float.IsNaN(distance.z)) return;
            if (Mathf.Approximately(distance.sqrMagnitude, 0.0f)) return;

            Vector3 backupPosition = _nextPos;
            Vector3 remainingDistance = distance;

            int currentIteration = 0;
            int maxIteration = UPhysSettings.Instance.VelocityIteration;
            while (currentIteration < maxIteration && remainingDistance.sqrMagnitude > 0.0f)
            {
                if (InternalSafeMoveWithCollide(remainingDistance, out Vector3 leftDistance, out RaycastHit hitInfo))
                {
                    // Project remaining distance to surface
                    Vector3 tangent = GetTangent(leftDistance, hitInfo.normal);
                    remainingDistance = Vector3.Dot(tangent, leftDistance.normalized) * leftDistance.magnitude * tangent;

                    // Don't climbing unstable ground when on stable ground
                    // Unstable ground is regarded as a wall
                    if (_groundReport.HitAnyGround && _groundReport.IsStable)
                    {
                        if (Vector3.Angle(_nextCharacterUp, hitInfo.normal) > _stableAngle)
                        {
                            // Discard forward and upward distance
                            Vector3 sideVector = Vector3.Cross(hitInfo.normal, _nextCharacterUp).normalized;
                            remainingDistance = Vector3.Dot(sideVector, remainingDistance) * sideVector;
                        }
                    }
                    ++currentIteration;
                }
                else
                {
                    remainingDistance = Vector3.zero;
                    break;
                }
            }

            // Exceed velocity solve iteration
            if (currentIteration >= maxIteration)
            {
                // Discard calculated movement 
                if (UPhysSettings.Instance.KillPositionWhenExceedVelocityIteration)
                {
                    _nextPos = backupPosition;
                }
                // Appand remained distance to movement
                if (!UPhysSettings.Instance.KillRemainedDistanceWhenExceedVelocityIteration)
                {
                    _nextPos += remainingDistance;
                }
            }
        }

        /// <summary>Surface tanget for direction</summary>
        private Vector3 GetTangent (Vector3 direction, Vector3 surfaceNormal)
        {
            Vector3 rightVector = Vector3.Cross(direction, surfaceNormal);
            return Vector3.Cross(surfaceNormal, rightVector).normalized;
        }

        private bool IsValidCollider (Collider col)
        {
            // Ignore itself
            if (col == _body) return false;

            Rigidbody rb = col.attachedRigidbody;
            if (rb != null)
            {
                // Ignore dynamic rigidbody
                if (!rb.isKinematic) return false;
                // Ignore my object
                if (rb == _body.attachedRigidbody) return false;
                // Ignore movoing block in override update
                if (_inHandleRiding && rb == _riding) return false;
            }

            return true;
        }

        private bool IsValidCollider(RaycastHit hit)
        {
            return IsValidCollider(hit.collider);
        }
#endregion
    }
}
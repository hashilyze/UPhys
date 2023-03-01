using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UPhys
{
    [System.Serializable]
    public struct GroundHitReport
    {
        public bool HitAnyGround { get => m_hitAnyGround; set => m_hitAnyGround = value; }
        public bool IsStable { get => m_isStable; set => m_isStable = value; }

        public Collider Collider { get => m_collider; set => m_collider = value; }
        public Vector3 Point { get => m_point; set => m_point = value; }
        public Vector3 Normal { get => m_normal; set => m_normal = value; }
        public float Angle { get => m_angle; set => m_angle = value; }
        public float Distance { get => m_distance; set => m_distance = value; }

        public float ElapsedUngroundTime { get => m_elapsedUngroundTime; set => m_elapsedUngroundTime = value; }


        [Tooltip("Whether detect ground regardless with stablity")]
        [SerializeField] private bool m_hitAnyGround;
        [Tooltip("Whether detected ground is stabe, character doesn't slide on")]
        [SerializeField] private bool m_isStable;
        [SerializeField] private Collider m_collider;
        [SerializeField] private Vector3 m_point;
        [SerializeField] private Vector3 m_normal;
        [SerializeField] private float m_angle;
        [SerializeField] private float m_distance;
        [SerializeField] private float m_elapsedUngroundTime;
    }

    public class UCharacterMovement : MonoBehaviour
    {
        #region Public
        public Vector3 Velocity => m_velocity;
        public float Mass => m_mass;
        public Vector3 CharacterUp => m_nextCharacterUp;

        public bool UseGroundSnap { get => m_useGroundSnap; set => m_useGroundSnap = value; }
        public float StableAngle { get => m_stableAngle; set => m_stableAngle = Mathf.Clamp(value, 0.0f, 90.0f); }
        public float StepOffset { get => m_stepOffset; set => m_stepOffset = value; }
        public GroundHitReport GroundReport => m_groundReport;

        public LayerMask CollidableMask { get => m_collidableMask; set => m_collidableMask = value; }
        public LayerMask BlockMask { get => m_blockMask; set => m_blockMask = value; }

        /// <summaryMove character independently with velocity</summary>
        public void Move (Vector3 distance)
        {
            m_additionalMoveDistance += distance;
        }

        public void Bounce (Vector3 distance, float time) { }

        /// <summary>Place character to destination and fix position during this frame</summary>
        public void Teleport (Vector3 destination)
        {
            m_requestedTeleport = true;
            m_teleportDestination = destination;
        }
        /// <summary>Place character to ground with closest to destination</summary>
        public void TeleportUponGround(Vector3 destination, float snapDistance = int.MaxValue)
        {

        }

        public void Look (Vector3 direction) { }

        /// <summary>Detach from ground and skip ground handling until time over</summary>
        public void ForceUnground (float time = 0.1f)
        {
            m_isForceUnground = true;
            m_leftUngroundTime = time;
        }

        public void Simulate (float deltaTime)
        {
            // Initialize transform
            {
                m_nextPos = m_initPos = m_rb.position;
                m_nextRot = m_initRot = m_rb.rotation;
                m_nextCharacterUp = m_nextRot * Vector3.up;
            }
            // Teleport: Set Position
            if (m_requestedTeleport)
            {
                m_requestedTeleport = false;
                ConsumeTeleport();
                return;
            }
            // Handle Riding
            if (m_useRiding)
            {
                HandleRiding(deltaTime);
            }
            // Overlap Recovery
            if (m_handleStuck)
            {
                SolveOverlap();
            }
            // Update Velocity
            {
                Vector3 distance;

                m_characterController.UpdateVelocity(deltaTime, ref m_velocity, this);
                distance = m_velocity * deltaTime;

                distance += m_additionalMoveDistance;
                m_additionalMoveDistance = Vector3.zero;

                InternalSafeMoveWithSlide(distance);
            }
            // Update Look
            {
                m_characterController.UpdateRotation(deltaTime, ref m_nextRot, this);
                m_nextCharacterUp = m_nextRot * Vector3.up;
            }
            // Probe Ground and Snap
            if (m_useHandleGround)
            {
                ProbeGround(deltaTime);
            }
            // Move Position and Rotation
            {
                // Update transform in physics scene to inform transform
                m_rb.position = m_nextPos;
                m_rb.rotation = m_nextRot;

                transform.position = m_nextPos;
                transform.rotation = m_nextRot;
            }
        }

        public void SimulateCommit ()
        {
            // Rollback transform as before simulate
            m_rb.position = m_initPos;
            m_rb.rotation = m_initRot;
            // Use movement funtion instead directly set raw transform because simulate with dynamic rigidbody
            m_rb.MovePosition(m_nextPos);
            m_rb.MoveRotation(m_nextRot);
        }
        #endregion

        #region Private
        [Header("Base")]
        [ReadOnly][SerializeField] private Vector3 m_velocity;
        [SerializeField] private float m_mass = 100.0f;

        [Header("Ground Setting")]
        [Tooltip("Use ground interaction")]
        [SerializeField] private bool m_useHandleGround = true;
        [Tooltip("Snap character to ledge when bounce on")]
        [SerializeField] private bool m_useGroundSnap = true;
        [Tooltip("Character could climb on slope")]
        [Range(0.0f, 90.0f)][SerializeField] private float m_stableAngle = 50.0f;
        [Tooltip("Character could ignore blocking and step up/down")]
        [SerializeField] private float m_stepOffset = 0.2f;
        [ReadOnly][SerializeField] private GroundHitReport m_groundReport;
        // Whether skip probe ground
        private bool m_isForceUnground = false;
        // Skip probe ground until left time zero
        private float m_leftUngroundTime = 0.0f;

        [Header("Riding")]
        [SerializeField] private bool m_useRiding = true;
        [ReadOnly][SerializeField] private Rigidbody m_riding;
        [ReadOnly][SerializeField] private Vector3 m_ridingContactPoint;
        private bool m_inHandleRiding = false;

        [Header("Misc")]
        [SerializeField] private bool m_handleStuck = true;
        [Tooltip("Character can not pass though collider being layer")]
        [SerializeField] private LayerMask m_collidableMask;
        [Tooltip("Character never pass though collider being layer")]
        [SerializeField] private LayerMask m_blockMask;

        // Memory buffer
        private readonly Collider[] m_colliderBuffer = new Collider[8];
        private readonly RaycastHit[] m_hitInfoBuffer = new RaycastHit[8];
        // External Components
        private CapsuleCollider m_body;
        private Rigidbody m_rb;
        private UCharacterController m_characterController;
        private CollisionHandler m_collisionHandler;
        // Transform cache
        private Vector3 m_initPos;
        private Quaternion m_initRot;
        private Vector3 m_nextPos;
        private Quaternion m_nextRot;
        private Vector3 m_nextCharacterUp;
        // Additional movement
        private Vector3 m_additionalMoveDistance;
        // Teleport
        private Vector3 m_teleportDestination;
        private bool m_requestedTeleport = false;


        private void Awake ()
        {
            // Initialize _body
            m_body = GetComponent<CapsuleCollider>();
            if (m_body == null)
            {
                throw new System.NullReferenceException("");
            }
            m_body.isTrigger = false;
            // Initialize _rb
            m_rb = GetComponent<Rigidbody>();
            if (m_rb == null)
            {
                throw new System.NullReferenceException("");
            }
            m_rb.useGravity = false;
            m_rb.isKinematic = true;
            m_rb.constraints = RigidbodyConstraints.None;
            // Initialize _cct
            m_characterController = GetComponent<UCharacterController>();
            if (m_characterController == null)
            {
                throw new System.NullReferenceException("");
            }
            // Initialize _collisionHandler
            m_collisionHandler = GetComponent<CollisionHandler>();
            if (m_collisionHandler == null)
            {
                throw new System.NullReferenceException("");
            }
            m_collisionHandler.Setup(m_body);
        }
        private void OnEnable ()
        {
            UPhysSystem.RegisterCharacterMovement(this);
        }
        private void OnDisable ()
        {
            UPhysSystem.UnregisterCharacterMovement(this);
        }

        private void ConsumeTeleport ()
        {
            m_nextPos = m_teleportDestination;
        }

        private void HandleRiding (float deltaTime)
        {
            // Validate ridings
            m_riding = null;
            // Ride moving platform
            if (m_groundReport.HitAnyGround && m_groundReport.IsStable)
            {
                // Ground Object is removed or destoryed 
                if (m_groundReport.Collider == null)
                {
                    return;
                }

                m_riding = m_groundReport.Collider.attachedRigidbody;
                m_ridingContactPoint = m_groundReport.Point;
            }

            if (m_riding != null)
            {
                Vector3 snapDistance = m_riding.GetPointVelocity(m_ridingContactPoint) * deltaTime;
                //Vector3 snapDistance = (m_riding.velocity + Vector3.Cross(m_riding.angularVelocity, m_ridingContactPoint - m_riding.position)) * deltaTime;

                m_inHandleRiding = true;
                InternalSafeMoveWithCollide(snapDistance, out Vector3 _, out RaycastHit _);
                m_inHandleRiding = false;
            }
        }

        /// <summary>Depentrate character from obstacles; Pushed character from colliders</summary>
        

        private void ClearGroundReport ()
        {
            m_groundReport.HitAnyGround = false;

            m_groundReport.Collider = null;
            m_groundReport.Point = Vector3.zero;
            m_groundReport.Distance = 0.0f;
            m_groundReport.Normal = Vector3.zero;
            m_groundReport.Angle = 0.0f;
            m_groundReport.IsStable = false;
            m_groundReport.ElapsedUngroundTime = 0.0f;
        }

        /// <summary>Update Timer for ForceUnground</summary>
        /// <returns>True when have left time after updated</returns>
        private bool UpdateForceUngroundTimer (float deltaTime)
        {
            return m_isForceUnground && (m_isForceUnground = (m_leftUngroundTime -= deltaTime) > 0.0f);
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
            bool wasStable = m_groundReport.HitAnyGround && m_groundReport.IsStable;

            m_groundReport.HitAnyGround = false;
            m_groundReport.Collider = null;

            float probeDistance = UPhysSettings.Instance.SkinWidth * 2.0f;

            Vector3 movementDistance = m_nextPos - m_initPos;
            float horizontalDistance = Vector3.ProjectOnPlane(movementDistance, m_nextCharacterUp).magnitude;
            float verticalDistance = Vector3.Dot(movementDistance, m_nextCharacterUp);

            // Snapable ground probing range
            float maxSnapDistance = 0.0f;
            if (wasStable)
            {
                maxSnapDistance = GetSnapGroundDistance(horizontalDistance, verticalDistance, m_stableAngle);
            }

            // Cast to downward from bottom of character
            if (m_collisionHandler.Sweep(m_nextPos, -m_nextCharacterUp, probeDistance + maxSnapDistance, m_hitInfoBuffer, out RaycastHit closestHit, m_collidableMask & m_blockMask, IsValidCollider) > 0)
            {
                float angle = Vector3.Angle(m_nextCharacterUp, closestHit.normal);

                // Accurated snapable range because of ground's angleFor
                float minSnapDistance = 0.0f; 
                // Don't snap when on steep ramp
                if (wasStable && angle <= m_stableAngle)
                {
                    minSnapDistance = GetSnapGroundDistance(horizontalDistance, verticalDistance, angle);
                }
                // Grounded when on ground or snapable
                if (closestHit.distance <= probeDistance + minSnapDistance)
                {
                    m_groundReport.HitAnyGround = true;

                    m_groundReport.Collider = closestHit.collider;
                    m_groundReport.Point = closestHit.point;
                    m_groundReport.Distance = closestHit.distance;
                    m_groundReport.Normal = closestHit.normal;
                    m_groundReport.Angle = angle;
                    m_groundReport.IsStable = angle <= m_stableAngle;
                    m_groundReport.ElapsedUngroundTime = 0.0f;
                }
            }

            // Events after ground probing
            // On Ground
            if (m_groundReport.HitAnyGround && m_groundReport.IsStable)
            {
                // Ground snapping
                if (m_useGroundSnap)
                {
                    m_nextPos -= (m_groundReport.Distance - UPhysSettings.Instance.SkinWidth) * m_nextCharacterUp;
                    m_groundReport.Distance = UPhysSettings.Instance.SkinWidth;
                }

                // On Landed (first tick when grounded)
                if (!wasStable)
                {
                    // Discard vertical velocity
                    m_velocity = Vector3.ProjectOnPlane(m_velocity, m_nextCharacterUp);

                    //m_leftMoreJumpCount = m_moreJumpCount;
                    //m_doJump = false;
                    //OnLand?.Invoke(this);
                    m_characterController.OnLand();
                }
            }
            else
            {
                m_groundReport.ElapsedUngroundTime += deltaTime;
            }
        }

        private void SolveOverlap ()
        {
            // * How can I determine unrecoverible to stop shaking
            Vector3 backupPosition = m_nextPos;

            int currentIteration = 0;
            int maxIteration = UPhysSettings.Instance.DepentrationIteration;
            while (currentIteration < maxIteration)
            {
                // If there are overlaped collider with character, detach character from these
                int overlapCount = m_collisionHandler.Overlap(m_nextPos, m_colliderBuffer, m_collidableMask, IsValidCollider);
                // If no more found overlaped collider
                if (overlapCount == 0)
                {
                    break;
                }
                for (int cur = 0; cur < overlapCount; ++cur)
                {
                    Collider overlapCollider = m_colliderBuffer[cur];
                    
                    UPhysUtility.GetPosAndRot(overlapCollider, out Vector3 overlapPos, out Quaternion overlapRot);

                    // Depentrate if colliders are overlaped with deeper than zero
                    if (Physics.ComputePenetration(m_body, m_nextPos, m_nextRot,
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
                    m_nextPos = backupPosition;
                }
            }
        }

        /// <summary>Move character and stop when collide</summary>
        private bool InternalSafeMoveWithCollide(Vector3 distance, out Vector3 remainingDistance, out RaycastHit hitInfo)
        {
            Vector3 direction = distance.normalized;
            float magnitude = distance.magnitude;
            // Character react to obstacles when crush
            if (m_collisionHandler.Sweep(m_nextPos, direction, magnitude + UPhysSettings.Instance.SkinWidth, m_hitInfoBuffer, out RaycastHit closestHit, m_collidableMask & m_blockMask, IsValidCollider) > 0)
            {
                hitInfo = closestHit;
                // Move character until blocked
                closestHit.distance -= UPhysSettings.Instance.SkinWidth;
                m_nextPos += closestHit.distance * direction;
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
                m_nextPos += distance;
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

            Vector3 backupPosition = m_nextPos;
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
                    if (m_groundReport.HitAnyGround && m_groundReport.IsStable)
                    {
                        if (Vector3.Angle(m_nextCharacterUp, hitInfo.normal) > m_stableAngle)
                        {
                            // Discard forward and upward distance
                            Vector3 sideVector = Vector3.Cross(hitInfo.normal, m_nextCharacterUp).normalized;
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
                    m_nextPos = backupPosition;
                }
                // Appand remained distance to movement
                if (!UPhysSettings.Instance.KillRemainedDistanceWhenExceedVelocityIteration)
                {
                    m_nextPos += remainingDistance;
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
            if (col == m_body) return false;

            Rigidbody rb = col.attachedRigidbody;
            if (rb != null)
            {
                // Ignore dynamic rigidbody
                if (!rb.isKinematic) return false;
                // Ignore my object
                if (rb == m_body.attachedRigidbody) return false;
                // Ignore movoing block in override update
                if (m_inHandleRiding && rb == m_riding) return false;
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
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UPhys
{
    [System.Serializable]
    public struct GroundHitReport
    {
        public bool HitAnyGround
        {
            get => m_hitAnyGround;
            set => m_hitAnyGround = value;
        }
        public bool IsStable
        {
            get => m_isStable;
            set => m_isStable = value;
        }
        public Collider Collider
        {
            get => m_collider;
            set => m_collider = value;
        }
        public Vector3 Point
        {
            get => m_point;
            set => m_point = value;
        }
        public Vector3 Normal
        {
            get => m_normal;
            set => m_normal = value;
        }
        public float Angle
        {
            get => m_angle;
            set => m_angle = value;
        }
        public float Distance
        {
            get => m_distance;
            set => m_distance = value;
        }
        public float ElapsedUngroundTime
        {
            get => m_elapsedUngroundTime;
            set => m_elapsedUngroundTime = value;
        }


        [Tooltip("Whether detect ground regardless with stablity")]
        private bool m_hitAnyGround;
        [Tooltip("Whether detected ground is stabe, character doesn't slide on")]
        private bool m_isStable;
        private Collider m_collider;
        private Vector3 m_point;
        private Vector3 m_normal;
        private float m_angle;
        private float m_distance;
        private float m_elapsedUngroundTime;
    }

    public class UCharacterMovement : MonoBehaviour
    {
        #region Public
        public Vector3 Velocity => m_velocity;
        public float Mass => m_mass;
        public Vector3 CharacterUp => m_nextCharacterUp;

        public bool UseGroundSnap
        {
            get => m_useGroundSnap;
            set => m_useGroundSnap = value;
        }
        public float StableAngle
        {
            get => m_stableAngle;
            set => m_stableAngle = Mathf.Clamp(value, 0.0f, 90.0f);
        }
        public float StepOffset
        {
            get => m_stepOffset;
            set => m_stepOffset = value;
        }
        public GroundHitReport GroundReport => m_groundReport;

        public LayerMask CollidableMask
        {
            get => m_collidableMask;
            set => m_collidableMask = value;
        }
        public LayerMask BlockMask
        {
            get => m_blockMask;
            set => m_blockMask = value;
        }

        /// <summary>
        /// Move character independently with velocity
        /// </summary>
        public void Move(Vector3 distance) 
        {
            m_additionalMoveDistance += distance;
        }

        /// <summary>
        /// 
        /// </summary>
        public void Bounce(Vector3 distance, float time) 
        { 
        
        }

        /// <summary>
        /// Place character to destination and fix position during this frame
        /// </summary>
        public void Teleport(Vector3 destination)
        {
            m_requestedTeleport = true;
            m_teleportDestination = destination;
        }

        public void Look(Vector3 direction) {  }
        public void Stand(Vector3 normal) { }

        /// <summary>
        /// Character couldn't touch ground until time over
        /// </summary>
        public void ForceUnground (float time = 0.1f)
        {
            m_isForceUnground = true;
            m_leftUngroundTime = time;
        }

        public void Simulate (float deltaTime)
        {
            if (deltaTime <= 0.0f) return;

            // Initialize transform
            {
                m_nextPos = m_initPos = m_rigidbody.position;
                m_nextRot = m_initRot = m_rigidbody.rotation;
                m_nextCharacterUp = m_nextRot * Vector3.up;
            }

            // Teleport: Set Position
            {
                if (m_requestedTeleport)
                {
                    m_requestedTeleport = false;
                    ConsumeTeleport();
                    return;
                }
            }

            // Handle Riding
            {
                if (m_useRiding)
                {
                    HandleRiding(deltaTime);
                }
            }

            // Overlap Recovery
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

                InternalSafeMove(distance);
            }

            // Update Look
            {
                m_characterController.UpdateRotation(deltaTime, ref m_nextRot, this);
                m_nextCharacterUp = m_nextRot * Vector3.up;
            }

            // Stand on Planet and Snapping
            {
            }

            // Probe Ground and Snap
            {
                ProbeGround(deltaTime);
            }

            // Move Position and Rotation
            {
                // Update transform in physics scene to inform transform
                m_rigidbody.position = m_nextPos;
                m_rigidbody.rotation = m_nextRot;
            }
        }

        public void SimulateCommit ()
        {
            // Rollback transform as before simulate
            m_rigidbody.position = m_initPos;
            m_rigidbody.rotation = m_initRot;
            // Use movement funtion instead directly set raw transform because simulate with dynamic rigidbody
            m_rigidbody.MovePosition(m_nextPos);
            m_rigidbody.MoveRotation(m_nextRot);
        }
        #endregion

        #region Private
        [Header("Base")]
        [ReadOnly][SerializeField] private Vector3 m_velocity;
        [SerializeField] private float m_mass = 100.0f;

        [Header("Ground Setting")]
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
        
        [Header("Gravity")]
        [Tooltip("Apply gravity as character is on airbone")]
        [SerializeField] private bool m_useGravity = true;
        [Tooltip("Sacle of gravity without any weights; Higher falls faster")]
        [SerializeField] private float m_gravity = 10.0f;
        [Tooltip("Smooth feel of fall")]
        [SerializeField] private float m_fallGravityWeights = 1.2f;

        [Header("Riding")]
        [SerializeField] private bool m_useRiding = true;
        [SerializeField] private bool m_isRideDynamic = false;
        [ReadOnly] [SerializeField] private Rigidbody m_riding;
        [ReadOnly] [SerializeField] private Vector3 m_ridingContactPoint;

        [Header("Misc")]
        [Tooltip("Character can not pass though collider being layer")]
        [SerializeField] private LayerMask m_collidableMask;
        [Tooltip("Character never pass though collider being layer")]
        [SerializeField] private LayerMask m_blockMask;
        private bool m_inHandleRiding = false;

        // Memory buffer
        private readonly Collider[] m_colliderBuffer = new Collider[8];
        private readonly RaycastHit[] m_hitInfoBuffer = new RaycastHit[8];
        // External Components
        private CapsuleCollider m_bodyCollider;
        private Rigidbody m_rigidbody;
        private UCharacterController m_characterController;
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
            m_bodyCollider = GetComponent<CapsuleCollider>();
            if (m_bodyCollider != null)
            {
                m_bodyCollider.isTrigger = false;
            }

            m_rigidbody = GetComponent<Rigidbody>();
            if (m_rigidbody != null)
            {
                m_rigidbody.useGravity = false;
                m_rigidbody.isKinematic = true;
                m_rigidbody.constraints = RigidbodyConstraints.None;
            }
            m_characterController = GetComponent<UCharacterController>();
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

            if(m_riding != null)
            {
                // Snap riding
                if (m_isRideDynamic || m_riding.isKinematic)
                {
                    // Lacay
                    //Vector3 snapDistance;
                    //// Get delta distance from linear velocity
                    ////groundDistance = platform.Velocity * deltaTime;
                    //snapDistance = m_riding.velocity * deltaTime;

                    //// Get delta distance from angular velocity
                    ////if (platform.RotateAxis != Vector3.zero && !Mathf.Approximately(platform.RotateSpeed, 0.0f))
                    //if (m_riding.angularVelocity != Vector3.zero)
                    //{
                    //    // Torque calculated at last movor location
                    //    Vector3 lastStart = m_ridingContactPoint;
                    //    Vector3 lastCenter = m_riding.position - snapDistance;
                    //    //Quaternion deltaRotation = Quaternion.AngleAxis(platform.RotateSpeed * deltaTime, platform.RotateAxis);
                    //    Quaternion deltaRotation = Quaternion.Euler((m_riding.angularVelocity * deltaTime));
                    //    Vector3 lastDestination = deltaRotation * (lastStart - lastCenter) + lastCenter;
                    //    snapDistance += lastDestination - lastStart;
                    //}

                    Vector3 snapDistance = m_riding.GetPointVelocity(m_ridingContactPoint) * deltaTime;
                    //Vector3 snapDistance = (m_riding.velocity + Vector3.Cross(m_riding.angularVelocity, m_ridingContactPoint - m_riding.position)) * deltaTime;

                    //m_nextPos += snapDistance;
                    m_inHandleRiding = true;
                    if (SweepCharacter(m_nextPos, m_nextRot, snapDistance.normalized, snapDistance.magnitude + UPhysSystem.Settings.ContactOffset, m_hitInfoBuffer, out RaycastHit hit, true) > 0)
                    {
                        m_nextPos += (hit.distance - UPhysSystem.Settings.ContactOffset) * snapDistance.normalized;
                    }
                    else
                    {
                        m_nextPos += snapDistance;
                    }
                    m_inHandleRiding = false;
                }
            }
        }

        /// <summary>
        /// Depentrate character from obstacles; Pushed character from colliders
        /// </summary>
        private void SolveOverlap ()
        {
            UPhysSettings settings = UPhysSystem.Settings;
            // * How can I determine unrecoverible to stop shaking
            Vector3 tgtPos = m_nextPos;

            int currentIteration = 0;
            int maxIteration = settings.DepentrationIteration;
            while (currentIteration < maxIteration)
            {
                // If there are overlaped collider with character, detach character from these
                int overlapCount = OverlapCharacter(tgtPos, m_nextRot, m_colliderBuffer);

                // If no more found overlaped collider
                if (overlapCount == 0)
                {
                    break;
                }
                for (int cur = 0; cur < overlapCount; ++cur)
                {
                    Collider overlapCollider = m_colliderBuffer[cur];

                    GetPosAndRotInSimulation(overlapCollider, out Vector3 overlapPos, out Quaternion overlapRot);

                    // Depentrate if colliders are overlaped with deeper than zero
                    if (Physics.ComputePenetration(m_bodyCollider, tgtPos, m_nextRot,
                        overlapCollider, overlapPos, overlapRot, out Vector3 dir, out float dist))
                    {
                        //tgtPos += dir * (dist + m_contactOffset * 0.5f);
                        if (SweepCharacter(tgtPos, m_nextRot, dir, dist + settings.ContactOffset, m_hitInfoBuffer, out RaycastHit hitInfo, true) > 0)
                        {
                            // Move until collide
                            tgtPos += dir * (hitInfo.distance - settings.ContactOffset);

                            // * What do something? I'm forgot
                            // * Caution: Very very unstable process, need to more test
                            // * Do I need to unpush when on unstable ground?
                            // Reorient pushed distance without shrink unless squished
                            if (Vector3.Angle(-dir, hitInfo.normal) > 5.0f)
                            {
                                Vector3 tangent = GetTangent(dir, hitInfo.normal);

                                float leftDistOnDir = dist - (hitInfo.distance - settings.ContactOffset) + settings.ContactOffset * 0.5f;
                                float tangentOnDir = Vector3.Dot(tangent, dir);
                                if (tangentOnDir > 0.001f)
                                {
                                    float leftDistOnTangent = leftDistOnDir / tangentOnDir;

                                    //tgtPos += tangent * leftDistOnTangent;
                                    if (SweepCharacter(tgtPos, m_nextRot, tangent, leftDistOnTangent, m_hitInfoBuffer, out hitInfo, true) > 0)
                                    {
                                        tgtPos += tangent * (hitInfo.distance - settings.ContactOffset);
                                    }
                                    else
                                    {
                                        tgtPos += tangent * leftDistOnTangent;
                                    }
                                }
                            }
                        }
                        else
                        {
                            tgtPos += dir * (dist + settings.ContactOffset * 0.5f);
                        }
                    }
                }
                ++currentIteration;
            }

            if (currentIteration >= maxIteration)
            {
                // Break overlap recovery because character probably located in unsafed zone
                if (settings.KillPositionWhenExceedDepentrationIteration)
                {
                    tgtPos = m_nextPos;
                }
            }

            m_nextPos = tgtPos;
        }

        /// <summary>Move character continously</summary>
        private void InternalSafeMove (Vector3 distance)
        {
            // Discard unvalidated value
            if (float.IsNaN(distance.x) || float.IsNaN(distance.y) || float.IsNaN(distance.z)) return;
            if (Mathf.Approximately(distance.sqrMagnitude, 0.0f)) return;

            UPhysSettings settings = UPhysSystem.Settings;
            Vector3 tgtPos = m_nextPos;
            Vector3 remainingDistance = distance;

            int currentIteration = 0;
            int maxIteration = settings.VelocityIteration;
            while (currentIteration < maxIteration && remainingDistance.sqrMagnitude > 0.0f)
            {
                Vector3 remainingDirection = remainingDistance.normalized;
                float remainingMagnitude = remainingDistance.magnitude;

                // Character react to obstacles when crush
                if (SweepCharacter(tgtPos, m_nextRot, remainingDirection, remainingMagnitude + settings.ContactOffset, m_hitInfoBuffer, out RaycastHit closestHit, true) > 0)
                {
                    // Move character until blocked
                    closestHit.distance -= settings.ContactOffset;
                    tgtPos += closestHit.distance * remainingDirection;

                    // Update remaining distance
                    //remainingMagnitude = Mathf.Max(remainingMagnitude - closestHit.distance, 0.0f);
                    remainingMagnitude -= closestHit.distance;
                    if (remainingMagnitude < 0.0f)
                    {
                        // Stop iteration unless be remained distance
                        remainingDistance = Vector3.zero;
                        break;
                    }

                    // Slide the surface of blocked obstacles
                    // Project remaining distance to surface
                    Vector3 tangent = GetTangent(remainingDirection, closestHit.normal);
                    remainingDistance = Vector3.Dot(tangent, remainingDirection) * remainingMagnitude * tangent;

                    // * How can I be independent from EState?
                    // Don't climbing unstable ground when on stable ground
                    // Unstable ground is regarded as a wall
                    if (m_groundReport.HitAnyGround && m_groundReport.IsStable)
                    {
                        if (Vector3.Angle(m_nextCharacterUp, closestHit.normal) > m_stableAngle)
                        {
                            // Discard forward and upward distance
                            // Legacy: this process couldn't discard forward distance
                            //float climbDistance = Vector3.Dot(remainingDistance, m_nextCharacterUp);
                            //if (climbDistance > 0.0f)
                            //{
                            //    remainingDistance -= climbDistance * m_nextCharacterUp;
                            //}
                            Vector3 sideVector = Vector3.Cross(closestHit.normal, m_nextCharacterUp).normalized;
                            remainingDistance = Vector3.Dot(sideVector, remainingDistance) * sideVector;
                        }
                    }
                    ++currentIteration;
                }
                else
                {
                    // There are no obstacles blocking movement
                    tgtPos += remainingDistance;
                    remainingDistance = Vector3.zero;
                    break;
                }
            }

            // When Exceed velocity iteration
            if (currentIteration >= maxIteration)
            {
                // Discard calculated next position
                if (settings.KillPositionWhenExceedVelocityIteration)
                {
                    tgtPos = m_nextPos;
                }

                // Appand remained distance to destination
                if (!settings.KillRemainedDistanceWhenExceedVelocityIteration)
                {
                    tgtPos += remainingDistance;
                }
            }
            m_nextPos = tgtPos;
        }

        private void ProbeGround (float deltaTime)
        {
            // Update Timer
            if (m_isForceUnground && (m_isForceUnground = (m_leftUngroundTime -= deltaTime) > 0.0f))
            {
                // Clear ground report
                m_groundReport.HitAnyGround = false;
                m_groundReport.Collider = null;
                return;
            }

            // Before update grounding report, cache previous report; used for checking OnLanded
            bool wasStable = m_groundReport.HitAnyGround && m_groundReport.IsStable;

            m_groundReport.HitAnyGround = false;
            m_groundReport.Collider = null;

            float probeDistance = UPhysSystem.Settings.ContactOffset * 2.0f;

            //float horizontalDistance = Vector3.ProjectOnPlane(m_velocity, m_nextCharacterUp).magnitude * deltaTime;
            //float verticalDistance = Vector3.Dot(m_velocity, m_nextCharacterUp) * deltaTime;
            float horizontalDistance = Vector3.ProjectOnPlane(m_nextPos - m_initPos, m_nextCharacterUp).magnitude;
            float verticalDistance = Vector3.Dot(m_nextPos - m_initPos, m_nextCharacterUp);

            float maxSnapDistance = 0.2f;   // Snapable ground probing range
            if (wasStable)
            {
                maxSnapDistance = horizontalDistance * Mathf.Tan(m_stableAngle * Mathf.Deg2Rad);

                if (verticalDistance > 0.0f)
                {
                    maxSnapDistance += verticalDistance;
                }
            }
            // Cast to downward from bottom of character
            if (SweepCharacter(m_nextPos, m_nextRot, -m_nextCharacterUp, probeDistance + maxSnapDistance, m_hitInfoBuffer, out RaycastHit closestHit, true) > 0)
            {
                float angle = Vector3.Angle(m_nextCharacterUp, closestHit.normal);

                float minSnapDistance = 0.0f; // Accurated snapable range because of ground's angle
                // Don't snap when on steep ramp
                if (wasStable && angle <= m_stableAngle)
                {
                    minSnapDistance += horizontalDistance * Mathf.Tan(angle * Mathf.Deg2Rad);

                    if (verticalDistance > 0.0f)
                    {
                        minSnapDistance += verticalDistance;
                    }
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
                    m_nextPos -= (m_groundReport.Distance - UPhysSystem.Settings.ContactOffset) * m_nextCharacterUp;
                    m_groundReport.Distance = UPhysSystem.Settings.ContactOffset;
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

        private Vector3 GetTangent (Vector3 direction, Vector3 surfaceNormal)
        {
            Vector3 rightVector = Vector3.Cross(direction, surfaceNormal);
            return Vector3.Cross(surfaceNormal, rightVector).normalized;
        }

        /// <summary>
        // Get overlaped collider's transform in current physics scene
        /// </summary>
        private void GetPosAndRotInSimulation (Collider collider, out Vector3 pos, out Quaternion rot)
        {   
            // * How can I get transform of collider's more simply?

            // Get changed transform if UPhys Object
            Rigidbody overlapRigidbody = collider.attachedRigidbody;
            if (overlapRigidbody != null)
            {
                if(overlapRigidbody.isKinematic)
                {
                    pos = overlapRigidbody.position;
                    if (overlapRigidbody.gameObject != collider.gameObject)
                    {
                        pos += overlapRigidbody.rotation * collider.transform.localPosition;
                    }
                    rot = overlapRigidbody.rotation;
                }
                else
                {
                    pos = overlapRigidbody.position;
                    rot = overlapRigidbody.rotation;
                }
            }
            else
            {
                Transform overlapTransform = collider.transform;
                pos = overlapTransform.position;
                rot = overlapTransform.rotation;
            }
        }

        private bool IsValidCollider (Collider col)
        {
            // Ignore itself
            if (col == m_bodyCollider) return false;

            Rigidbody rb = col.attachedRigidbody;
            if (rb != null)
            {
                // Ignore dynamic rigidbody
                if (!rb.isKinematic) return false;
                // Ignore my object
                if (rb == m_bodyCollider.attachedRigidbody) return false;
                // Ignore movoing block in override update
                if (m_inHandleRiding && rb == m_riding) return false;
            }

            return true;
        }

        /// <summary>
        /// Find obstacle overlaped with character when located at
        /// </summary>
        private int OverlapCharacter (Vector3 position, Quaternion rotation, Collider[] results, bool onlyBlock = false)
        {
            int layerMask = m_collidableMask;
            if (onlyBlock)
            {
                m_collidableMask &= m_blockMask;
            }

            Vector3 center = position + rotation * m_bodyCollider.center;
            Vector3 anchor = rotation * ((0.5f * m_bodyCollider.height - m_bodyCollider.radius) * Vector3.up);
            int count = Physics.OverlapCapsuleNonAlloc(center + anchor, center - anchor,
                m_bodyCollider.radius, results, layerMask, QueryTriggerInteraction.Ignore);

            // Filter collider
            int ret = 0;
            for (int cur = 0; cur < count; ++cur)
            {
                if (IsValidCollider(results[cur]))
                {
                    results[ret++] = results[cur];
                }
            }
            return ret;
        }

        /// <summary>
        /// Cast character body and get the closest hit when located at
        /// </summary>
        private int SweepCharacter (Vector3 position, Quaternion rotation, Vector3 direction, float distance, RaycastHit[] results, out RaycastHit closestHit, bool onlyBlock = false)
        {
            int layerMask = m_collidableMask;
            if (onlyBlock)
            {
                layerMask &= m_blockMask;
            }

            int ret = 0;
            closestHit = results[0];
            float closestDist = float.PositiveInfinity;

            Vector3 center = position + rotation * m_bodyCollider.center;
            Vector3 anchor = rotation * ((0.5f * m_bodyCollider.height - m_bodyCollider.radius) * Vector3.up);
            int count = Physics.CapsuleCastNonAlloc(center + anchor, center - anchor,
                m_bodyCollider.radius, direction, results, distance, layerMask, QueryTriggerInteraction.Ignore);

            for (int cur = 0; cur < count; ++cur)
            {
                ref RaycastHit hit = ref m_hitInfoBuffer[cur];

                if (!IsValidCollider(hit.collider))
                {
                    continue;
                }
                // Recalculate overlaped collider's hitInfo because it has reversed normal
                if (hit.distance <= 0.0f)
                {
                    GetPosAndRotInSimulation(hit.collider, out Vector3 hitPos, out Quaternion hitRot);

                    if (Physics.ComputePenetration(
                        m_bodyCollider, position, rotation,
                        hit.collider, hitPos, hitRot,
                        out Vector3 dir, out float _))
                    {
                        hit.normal = dir;
                    }
                }
                // Ignore reversed normal 
                if (Vector3.Dot(direction, hit.normal) >= 0.0f)
                {
                    continue;
                }

                // Update to closer hitInfo
                if (hit.distance < closestDist)
                {
                    closestHit = hit;
                    closestDist = hit.distance;
                }
                results[ret++] = hit;
            }
            return ret;
        }
        #endregion
    }
}
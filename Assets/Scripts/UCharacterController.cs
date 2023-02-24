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

    [System.Serializable]
    public struct GroundHitReport
    {
        [Tooltip("Whether detect ground regardless with stablity")]
        public bool hitAnyGround;
        [Tooltip("Whether detected ground is stabe, character doesn't slide on")]
        public bool isStable;
        public Collider collider;
        public Vector3 point;
        public Vector3 normal;
        public float angle;
        public float distance;
        public float elapsedUngroundTime;
    }

    public class UCharacterController : MonoBehaviour
    {
        // Called when character is grounded but previous tick is not
        public System.Action<UCharacterController> OnLand;
        // Called when character is falling but previous tick is not
        public System.Action<UCharacterController> OnFall;

        public void Move ()
        {

        }
        public void Jump ()
        {

        }
        public void Teleport (Vector3 destination)
        {
            m_requestedTeleport = true;
            m_teleportDestination = destination;
        }
        public void Stand ()
        {

        }
        public void Look ()
        {

        }
        public void Bounce (Vector3 force)
        {
            //m_isAutoJump = true;
            Vector3 characterUp = transform.up;
            {
                float verticalBounce = Vector3.Dot(force, characterUp);
                m_requestedJump = true;
                m_requestedJumpSpeed = verticalBounce;
            }

            {
                Vector3 horizontalBounce = Vector3.ProjectOnPlane(force, characterUp);
                m_velocity += horizontalBounce;
            }
        }

        public void SetPlayerInput (InputSet inputSet)
        {
            m_inputDirection = new Vector3(inputSet.moveAxis.x, 0.0f, inputSet.moveAxis.y);
                
        }
        public void DoJump ()
        {
            m_requestedJump = true;
            m_requestedJumpSpeed = Mathf.Sqrt(2.0f * m_gravity * m_maxJumpDistance);
        }
        public void StopJump ()
        {
            m_requestedStopJump = true;
        }

        /// <summary>
        /// Character couldn't touch ground until time over
        /// </summary>
        public void ForceUnground (float time = 0.1f)
        {
            m_isForceUnground = true;
            m_leftUngroundTime = time;
        }


        [Header("Base")]
        [ReadOnly] [SerializeField] private Vector3 m_velocity;
        //[SerializeField] private float m_mass = 100.0f;
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
        [Tooltip("Dealy canceal jump until time over")]
        [SerializeField] private float m_minJumpTime = 0.02f;
        [Tooltip("How many do jumping on air")]
        [SerializeField] private int m_moreJumpCount = 1;
        [ReadOnly] [SerializeField] private int m_leftMoreJumpCount;
        [Tooltip("Jump a few time later after ungrounded")]
        [SerializeField] private float m_coyoteTime = 0.05f;
        [Tooltip("Kepp jump if failed")]
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


        [Header("Ground Setting")]
        [Tooltip("Snap character to ledge when bounce on")]
        [SerializeField] private bool m_useGroundSnap = true;
        [Tooltip("Character could climb on slope")]
        [Range(0.0f, 90.0f)] [SerializeField] private float m_stableAngle = 50.0f;
        [Tooltip("Character could ignore blocking and step up/down")]
        [SerializeField] private float m_stepOffset = 0.2f;
        [ReadOnly] [SerializeField] private GroundHitReport m_groundReport;
        // Force Unground parameters
        private bool m_isForceUnground = false;
        private float m_leftUngroundTime = 0.0f;

        [Header("Physics Setting")]
        [Tooltip("Character can not pass though collider being layer")]
        [SerializeField] private LayerMask m_collidableMask;
        [Tooltip("Character never pass though collider being layer")]
        [SerializeField] private LayerMask m_blockMask;
        // Solve Overlap Setting
        [Tooltip("Gap between character and others")]
        [SerializeField] private float m_contactOffset = 0.02f;
        [Tooltip("Accuracy of depentration solver; Higher costs more")]
        [SerializeField] private int m_depentrationIteration = 1;
        [Tooltip("Canceal depentration interation")]
        [SerializeField] private bool m_killPositionWhenExceedDepentrationIteration = false;
        [Tooltip("Accuracy of velocity solver; Higher costs more")]
        // Move with velocity Setting
        [SerializeField] private int m_velocityIteration = 3;
        [Tooltip("Canceal velocity interation")]
        [SerializeField] private bool m_killPositionWhenExceedVelocityIteration = true;
        [Tooltip("Discard remained deistance but not appand")]
        [SerializeField] private bool m_killRemainedDistanceWhenExceedVelocityIteration = true;
        private bool m_inMoverHandle = false;

        // Memory buffer
        private readonly Collider[] m_colliderBuffer = new Collider[8];
        private readonly RaycastHit[] m_hitInfoBuffer = new RaycastHit[8];

        // External Components
        private CapsuleCollider m_bodyCollider;
        private Rigidbody m_rigidbody;

        // Transform cache to simulate
        private Vector3 m_initPos;
        private Quaternion m_initRot;
        private Vector3 m_nextPos;
        private Quaternion m_nextRot;

        // Requested values
        // Move
        private Vector3 m_inputDirection;
        // Jump
        private float m_requestedJumpSpeed = 0.0f;
        private bool m_requestedJump = false;
        private float m_elapsedRequestedJump = 0.0f;
        private bool m_isAutoJump = false;
        private bool m_requestedStopJump = false;
        // Teleport
        private Vector3 m_teleportDestination;
        private bool m_requestedTeleport = false;
        

        private UWindZone m_windZone;
        
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
        }
        private void OnEnable ()
        {
            UPhysSystem.RegisterCharacterController(this);
            UInputManager.RegisterPlayer(this);
        }
        private void OnDisable ()
        {
            UPhysSystem.UnregisterCharacterController(this);
            UInputManager.UnregisterPlayer(this);
        }


        private void OnTriggerEnter (Collider other)
        {
            if (other.TryGetComponent(out UWindZone windZone))
            {
                m_windZone = windZone;
            }
            else if(other.TryGetComponent(out UWarpZone warpZone))
            {
                warpZone.Warp(this);
            }
            else if (other.TryGetComponent(out UBoostZone boostZone))
            {
                boostZone.Boost(this);
                Vector3 characterUp = transform.up;
                m_velocity
                    = Vector3.Dot(m_velocity, characterUp) * characterUp
                    + Vector3.ProjectOnPlane(m_velocity, characterUp).normalized * boostZone.BoostSpeed;
            }
            else if (other.TryGetComponent(out UBounceZone bounceZone))
            {
                Bounce(bounceZone.BounceForce);
            }
        }

        private void OnTriggerExit (Collider other)
        {
            if (other.TryGetComponent(out UWindZone windZone) && m_windZone == windZone)
            {
                m_windZone = null;
            }
            
        }


        #region Simulate
        /// <summary>
        /// Simulate after platform move
        /// </summary>
        public void Simulate (float deltaTime)
        {
            // Initialize transform
            {
                m_nextPos = m_initPos = m_rigidbody.position;
                m_nextRot = m_initRot = m_rigidbody.rotation;
            }

            // Teleport: Set Position
            {
                if (m_requestedTeleport)
                {
                    m_requestedTeleport = false;
                    m_nextPos = m_teleportDestination;
                }
            }

            // Handle Movor
            {
                HandleMovor(deltaTime);
            }

            // Overlap Recovery
            {
                SolveOverlap();
            }

            // Update Velocity
            {
                Vector3 distance;

                UpdateVelocity(deltaTime);
                distance = m_velocity * deltaTime;

                if (m_windZone)
                {
                    //distance += m_windZone.WindVelocity * deltaTime;
                    distance += m_windZone.GetWindVelocity(m_nextPos, m_velocity) * deltaTime;
                }

                CharacterMove(distance);
            }

            // Update Look
            {
                UpdateRotation(deltaTime);
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

        private void HandleMovor (float deltaTime)
        {
            // Ride moving platform
            if (m_groundReport.hitAnyGround && m_groundReport.isStable)
            {
                // Ground Object is removed or destoryed 
                if (m_groundReport.collider == null)
                {
                    return;
                }

                Rigidbody groundRB = m_groundReport.collider.attachedRigidbody;
                // * How do I not use 'GetComponent' to syncronize with Movor? : I need 'velocity' or 'distance'
                // * Need to refactoring of 'UPlatformController'
                if (groundRB != null && groundRB.isKinematic /*&& groundRB.TryGetComponent(out UPlatformController platform)*/)
                {
                    Vector3 groundDistance;

                    // Get delta distance from linear velocity
                    //groundDistance = platform.Velocity * deltaTime;
                    groundDistance = groundRB.velocity * deltaTime;

                    // Get delta distance from angular velocity
                    //if (platform.RotateAxis != Vector3.zero && !Mathf.Approximately(platform.RotateSpeed, 0.0f))
                    if (groundRB.angularVelocity != Vector3.zero)
                    {
                        // Torque calculated at last movor location
                        Vector3 lastStart = m_groundReport.point;
                        Vector3 lastCenter = groundRB.position - groundDistance;
                        //Quaternion deltaRotation = Quaternion.AngleAxis(platform.RotateSpeed * deltaTime, platform.RotateAxis);
                        Quaternion deltaRotation = Quaternion.Euler((groundRB.angularVelocity * deltaTime));
                        Vector3 lastDestination = deltaRotation * (lastStart - lastCenter) + lastCenter;
                        groundDistance += lastDestination - lastStart;
                    }

                    //m_nextPos += groundDistance;
                    m_inMoverHandle = true;
                    if (GetClosestHit(m_nextPos, m_nextRot, groundDistance.normalized, groundDistance.magnitude + m_contactOffset, out RaycastHit hit, true))
                    {
                        m_nextPos += (hit.distance - m_contactOffset) * groundDistance.normalized;
                    }
                    else
                    {
                        m_nextPos += groundDistance;
                    }
                    m_inMoverHandle = false;
                }
            }
        }

        /// <summary>
        /// Depentrate character from obstacles; Pushed character from colliders
        /// </summary>
        private void SolveOverlap ()
        {
            // * How can I determine unrecoverible to stop shaking
            Vector3 tgtPos = m_nextPos;

            int currentIteration = 0;
            while (currentIteration < m_depentrationIteration)
            {
                // If there are overlaped collider with character, detach character from these
                int overlapCnt = CharacterOverlap(tgtPos, m_nextRot, m_colliderBuffer);

                // If no more found overlaped collider
                if (overlapCnt == 0)
                {
                    break;
                }

                for (int cur = 0; cur < overlapCnt; ++cur)
                {
                    Collider overlapCollider = m_colliderBuffer[cur];

                    GetPosAndRotInPhysicsScene(overlapCollider, out Vector3 overlapPos, out Quaternion overlapRot);

                    // Depentrate if colliders are overlaped with deeper than zero
                    if (Physics.ComputePenetration(
                        m_bodyCollider, tgtPos, m_nextRot,
                        overlapCollider, overlapPos, overlapRot,
                        out Vector3 dir, out float dist))
                    {
                        //tgtPos += dir * (dist + m_contactOffset * 0.5f);
                        if (GetClosestHit(tgtPos, m_nextRot, dir, dist + m_contactOffset, out RaycastHit hitInfo, true))
                        {
                            // Move until collide
                            tgtPos += dir * (hitInfo.distance - m_contactOffset);

                            // * Caution: Very very unstable process, need to more test
                            // * Do I need to unpush when on unstable ground?
                            // Reorient pushed distance without shrink unless squished
                            if (Vector3.Angle(-dir, hitInfo.normal) > 5.0f)
                            {
                                //Vector3 characterUp = m_nextRot * Vector3.up;
                                //Vector3 tangent = Vector3.Cross(hitInfo.normal, Vector3.Cross(dir, characterUp)).normalized;
                                Vector3 tangent = Vector3.Cross(hitInfo.normal, Vector3.Cross(dir, hitInfo.normal)).normalized;
                                tangent *= Mathf.Sign(Vector3.Dot(tangent, dir));

                                float leftDistOnDir = dist - (hitInfo.distance - m_contactOffset) + m_contactOffset * 0.5f;
                                float tangentOnDir = Vector3.Dot(tangent, dir);
                                //if (tangentOnDir > 0.001f)
                                {
                                    float leftDistOnTangent = leftDistOnDir / tangentOnDir;

                                    //tgtPos += tangent * leftDistOnTangent;
                                    if (GetClosestHit(tgtPos, m_nextRot, tangent, leftDistOnTangent, out hitInfo, true))
                                    {
                                        tgtPos += tangent * (hitInfo.distance - m_contactOffset);
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
                            tgtPos += dir * (dist + m_contactOffset * 0.5f);
                        }
                    }
                }
                ++currentIteration;
            }

            if (currentIteration >= m_depentrationIteration)
            {
                // Break overlap recovery because character probably located in unsafed zone
                if (m_killPositionWhenExceedDepentrationIteration)
                {
                    tgtPos = m_nextPos;
                }
            }

            m_nextPos = tgtPos;
        }

        /// <summary>Move character continously</summary>
        /// <param name="distance">Distance character will move based on world unit</param>
        private void CharacterMove (Vector3 distance)
        {
            // Discard unvalidated value
            if (float.IsNaN(distance.x) || float.IsNaN(distance.y) || float.IsNaN(distance.z)) return;
            if (Mathf.Approximately(distance.sqrMagnitude, 0.0f)) return;

            Vector3 tgtPos = m_nextPos;
            Vector3 remainingDistance = distance;

            int currentIteration = 0;
            while (currentIteration < m_velocityIteration && remainingDistance.sqrMagnitude > 0.0f)
            {
                Vector3 remainingDirection = remainingDistance.normalized;
                float remainingMagnitude = remainingDistance.magnitude;

                // Character react to obstacles when crush
                if (GetClosestHit(tgtPos, m_nextRot, remainingDirection, remainingMagnitude + m_contactOffset, out RaycastHit closestHit, true))
                {
                    // move character
                    closestHit.distance -= m_contactOffset;
                    tgtPos += closestHit.distance * remainingDirection;

                    //remainingMagnitude = Mathf.Max(remainingMagnitude - closestHit.distance, 0.0f);
                    remainingMagnitude -= closestHit.distance;
                    if (remainingMagnitude < 0.0f)
                    {
                        // Stop iteration no remained distance
                        remainingDistance = Vector3.zero;
                        break;
                    }

                    // Slide the surface of blocked obstacles
                    Vector3 tmp = Vector3.Cross(remainingDirection, closestHit.normal);
                    Vector3 tangent = Vector3.Cross(closestHit.normal, tmp).normalized; // slide direction

                    // Project remaining distance to surface
                    remainingDistance = Vector3.Dot(tangent, remainingDirection) * remainingMagnitude * tangent;

                    // * How can I be independent from EState?
                    // Don't climbing unstable ground when on stable ground
                    if(m_groundReport.hitAnyGround && m_groundReport.isStable)
                    {
                        Vector3 characterUp = m_nextRot * Vector3.up;

                        if (Vector3.Angle(characterUp, closestHit.normal) > m_stableAngle)
                        {
                            float climbDistance = Vector3.Dot(remainingDistance, characterUp);
                            if (climbDistance > 0.0f)
                            {
                                remainingDistance -= climbDistance * characterUp;
                            }
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
            if (currentIteration >= m_velocityIteration)
            {
                // Discard calculated next position
                if (m_killPositionWhenExceedVelocityIteration)
                {
                    tgtPos = m_nextPos;
                }

                // Appand remained distance to destination
                if (!m_killRemainedDistanceWhenExceedVelocityIteration)
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
                m_groundReport.hitAnyGround = false;
                m_groundReport.collider = null;
                return;
            }

            Vector3 characterUp = m_nextRot * Vector3.up;
            // Before update grounding report, cache previous report; used for checking OnLanded
            bool wasStable = m_groundReport.hitAnyGround && m_groundReport.isStable;

            m_groundReport.hitAnyGround = false;
            m_groundReport.collider = null;

            float probeDistance = m_contactOffset * 1.5f;

            //float horizontalDistance = Vector3.ProjectOnPlane(m_velocity, characterUp).magnitude * deltaTime;
            //float verticalDistance = Vector3.Dot(m_velocity, characterUp) * deltaTime;
            float horizontalDistance = Vector3.ProjectOnPlane(m_nextPos - m_initPos, characterUp).magnitude;
            float verticalDistance = Vector3.Dot(m_nextPos - m_initPos, characterUp);// * deltaTime;

            float maxSnapDistance = 0.0f;   // Snapable ground probing range
            if (wasStable)
            {
                maxSnapDistance = horizontalDistance * Mathf.Tan(m_stableAngle * Mathf.Deg2Rad);

                if (verticalDistance > 0.0f)
                {
                    maxSnapDistance += verticalDistance;
                }
            }

            // Cast toward downward from bottom of character
            if (GetClosestHit(m_nextPos, m_nextRot, -characterUp, probeDistance + maxSnapDistance, out RaycastHit closestHit, true))
            {
                float angle = Vector3.Angle(characterUp, closestHit.normal);

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
                    m_groundReport.hitAnyGround = true;

                    m_groundReport.collider = closestHit.collider;
                    m_groundReport.point = closestHit.point;
                    m_groundReport.distance = closestHit.distance;
                    m_groundReport.normal = closestHit.normal;
                    m_groundReport.angle = angle;
                    m_groundReport.isStable = angle <= m_stableAngle;
                    m_groundReport.elapsedUngroundTime = 0.0f;
                }
            }

            // Events for ground probing
            // On Ground
            if (m_groundReport.hitAnyGround && m_groundReport.isStable)
            {
                // Ground snapping
                if (m_useGroundSnap)
                {
                    m_nextPos -= (m_groundReport.distance - m_contactOffset) * characterUp;
                    m_groundReport.distance = m_contactOffset;
                }

                // On Landed (first tick when grounded)
                if (!wasStable)
                {
                    // Clear vertical velocity
                    m_velocity = Vector3.ProjectOnPlane(m_velocity, characterUp);

                    m_leftMoreJumpCount = m_moreJumpCount;
                    m_doJump = false;

                    OnLand?.Invoke(this);
                }
            }
            else
            {
                m_groundReport.elapsedUngroundTime += deltaTime;
            }
        }
        #endregion

        #region Update Velocity And Rotation
        private void UpdateVelocity (float deltaTime)
        {
            switch (m_state)
            {
            case EState.Default:
                Vector3 characterUp = m_nextRot * Vector3.up;

                // Movement on stable ground 
                if (m_groundReport.hitAnyGround && m_groundReport.isStable)
                {
                    // Reorient last velocity 
                    Vector3 tangent = Vector3.Cross(m_groundReport.normal, Vector3.Cross(m_velocity, characterUp)).normalized;
                    Vector3 reorientVelocity = m_velocity.magnitude * tangent;

                    // Accelerate movement
                    if (m_inputDirection != Vector3.zero)
                    {
                        // Reorient input direction to surface
                        //Quaternion upToSurface = Quaternion.FromToRotation(characterUp, m_groundReport.normal);
                        //Vector3 inputDirOnSurface = upToSurface * m_inputDirection;

                        Vector3 inputDirOnSurface = Vector3.Cross(m_groundReport.normal, Vector3.Cross(m_inputDirection, characterUp)).normalized;

                        m_velocity = Vector3.Lerp(reorientVelocity, m_maxSpeed * inputDirOnSurface, 1.0f - Mathf.Exp(-m_acceleration * deltaTime));
                    }
                    // Break movement
                    else
                    {
                        if (m_velocity != Vector3.zero)
                        {
                            m_velocity = Vector3.Lerp(reorientVelocity, Vector3.zero, 1.0f - Mathf.Exp(-m_friction * deltaTime));
                            // Break completely with discard velocity when lower than min distance
                            if (m_velocity.sqrMagnitude * deltaTime * deltaTime < m_minDistance * m_minDistance)
                            {
                                m_velocity = Vector3.zero;
                            }
                        }
                    }
                }
                // Movement on air
                else
                {
                    Vector3 horizontalVelocity = Vector3.ProjectOnPlane(m_velocity, characterUp);
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

                    Vector3 verticalVelocity;
                    // Vertical velocity
                    {
                        float verticalSpeed = Vector3.Dot(m_velocity, characterUp);

                        if (m_useGravity)
                        {
                            verticalSpeed += deltaTime * -m_gravity 
                                * (verticalSpeed < 0.0f ? m_fallGravityWeights : 1.0f)
                                * (Mathf.Abs(verticalSpeed) <= m_halfGravityThresdhold ? 0.5f : 1.0f);
                        }

                        verticalSpeed = Mathf.Max(verticalSpeed, -m_maxFallSpeed);
                        verticalVelocity = verticalSpeed * characterUp;

                        if (verticalSpeed < 0.0f)
                        {
                            m_doJump = false;

                            OnFall?.Invoke(this);
                        }
                    }

                    m_velocity = horizontalVelocity + verticalVelocity;
                }

                // Handle Jump
                {
                    if (m_doJump)
                    {
                        m_elapsedJumpTime += deltaTime;
                    }

                    if (m_requestedJump && m_elapsedRequestedJump <= m_jumpBuffer)
                    {
                        bool canJump = false;

                        if (m_groundReport.hitAnyGround && m_groundReport.isStable || m_groundReport.elapsedUngroundTime <= m_coyoteTime)
                        {
                            // Jump when on stable ground or a few frame later ungrounded
                            canJump = true;
                        }
                        else if (m_leftMoreJumpCount > 0)
                        {
                            // Should have more jump to jump on airbone
                            canJump = true;
                            --m_leftMoreJumpCount;
                        }
                        else
                        {
                            // Retry jump at next fream when failed to jump
                            m_elapsedRequestedJump += deltaTime;
                        }

                        if (canJump)
                        {
                            m_requestedJump = false;
                            m_elapsedRequestedJump = 0.0f;

                            m_doJump = true;
                            m_elapsedJumpTime = 0.0f;

                            // Discard old vertical velocity and set jump velocity
                            m_velocity = Vector3.ProjectOnPlane(m_velocity, characterUp) + characterUp * Mathf.Sqrt(2.0f * m_gravity * m_maxJumpDistance);
                            m_velocity = Vector3.ProjectOnPlane(m_velocity, characterUp) + characterUp * m_requestedJumpSpeed;

                            // * 
                            // Conserve momentum when jump on movor
                            if(m_groundReport.hitAnyGround && m_groundReport.isStable)
                            {
                                Rigidbody rb = m_groundReport.collider.attachedRigidbody;
                                if(rb != null && rb.isKinematic)
                                {
                                    if(Vector3.Dot(rb.velocity, m_velocity) >= 0.0f)
                                    {
                                        m_velocity += rb.velocity;
                                    }
                                }
                            }

                            // Prevent snapping when jump
                            ForceUnground();
                        }
                    }
                    else
                    {
                        // Jump waiting time over
                        m_requestedJump = false;
                        m_elapsedRequestedJump = 0.0f;
                    }

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
                            m_velocity = Vector3.ProjectOnPlane(m_velocity, characterUp) + Mathf.Min(Vector3.Dot(m_velocity, characterUp), m_variableJumpSpeed) * characterUp;
                        }
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

        private void UpdateRotation (float deltaTime)
        {
            switch (m_state)
            {
            case EState.Default:
                // Character could look toward movement direction
                Vector3 characterUp = m_nextRot * Vector3.up;
                if (m_velocity != Vector3.zero)
                {

                    Vector3 horizontalDirection = Vector3.ProjectOnPlane(m_velocity, characterUp).normalized;
                    if (horizontalDirection != Vector3.zero)
                    {
                        Quaternion targetRotation = Quaternion.LookRotation(horizontalDirection, characterUp);
                        m_nextRot = Quaternion.RotateTowards(m_nextRot, targetRotation, m_orientSpeed * deltaTime);
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

        #region Physics Query
        /// <summary>
        // Get overlaped collider's transform in current physics scene
        /// </summary>
        private void GetPosAndRotInPhysicsScene (Collider collider, out Vector3 pos, out Quaternion rot)
        {
            // * How can I get transform of collider's more simply?

            // Colliders managed by UPhys has probability its position will be changed in Simulate
            // To get correct position UPhys Objects use rigidbody no tranform maybe Static Collider
            Rigidbody overlapRigidbody = collider.attachedRigidbody;
            if (overlapRigidbody != null)
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
                if (m_inMoverHandle && rb == m_groundReport.collider.attachedRigidbody) return false;
            }

            return true;
        }

        /// <summary>
        /// Find obstacle overlaped with character when located at
        /// </summary>
        /// <param name="colliders">Obstacles will be stored</param>
        /// <returns>The number of stored obstacles in 'colliders'</returns>
        private int CharacterOverlap (Vector3 position, Quaternion rotation, Collider[] colliders, bool onlyBlock = false)
        {
            int layerMask = m_collidableMask;
            if (onlyBlock)
            {
                m_collidableMask &= m_blockMask;
            }

            Vector3 center = position + rotation * m_bodyCollider.center;
            Vector3 relativePoint = rotation * ((0.5f * m_bodyCollider.height - m_bodyCollider.radius) * Vector3.up);

            int cnt = Physics.OverlapCapsuleNonAlloc(center + relativePoint, center - relativePoint,
                m_bodyCollider.radius, colliders, layerMask, QueryTriggerInteraction.Ignore);
            // Filter collider
            int ret = 0;
            for (int cur = 0; cur < cnt; ++cur)
            {
                if (IsValidCollider(colliders[cur]))
                {
                    colliders[ret++] = colliders[cur];
                }
            }
            return ret;
        }

        /// <summary>
        /// Cast character body located at
        /// </summary>
        /// <param name="hitInfos">Cast results</param>
        /// <returns>The number of cast result stored in 'hitInfos'</returns>
        private int CharacterSweep (Vector3 position, Quaternion rotation, Vector3 direction, float distance, RaycastHit[] hitInfos, int layerMask)
        {
            Vector3 center = position + rotation * m_bodyCollider.center;
            Vector3 relativePoint = rotation * ((0.5f * m_bodyCollider.height - m_bodyCollider.radius) * Vector3.up);

            return Physics.CapsuleCastNonAlloc(center + relativePoint, center - relativePoint,
                m_bodyCollider.radius, direction, hitInfos, distance, layerMask, QueryTriggerInteraction.Ignore);
        }

        /// <summary>
        /// Cast character body and get the closest hit
        /// </summary>
        private bool GetClosestHit (Vector3 position, Quaternion rotation, Vector3 direction, float distance, out RaycastHit closestHit, bool onlyBlock = false)
        {
            int layerMask = m_collidableMask;
            if (onlyBlock)
            {
                layerMask &= m_blockMask;
            }

            bool hasValidHit = false;   // Return value for whether find validated hinInfo
            closestHit = default;
            float closestDist = float.PositiveInfinity;

            int hitCnt = CharacterSweep(position, rotation, direction, distance, m_hitInfoBuffer, layerMask);
            for (int cur = 0; cur < hitCnt; ++cur)
            {
                ref RaycastHit hit = ref m_hitInfoBuffer[cur];

                if (!IsValidCollider(hit.collider))
                {
                    continue;
                }
                // Recalculate overlaped collider's hitInfo because it has reversed normal
                if (hit.distance <= 0.0f)
                {
                    GetPosAndRotInPhysicsScene(hit.collider, out Vector3 hitPos, out Quaternion hitRot);

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
                    hasValidHit = true;
                }
            }
            return hasValidHit;
        }
        #endregion
    }
}
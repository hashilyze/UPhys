using UnityEngine;

namespace UPhys
{
    [CreateAssetMenu(fileName = "UPhys Settings", menuName = "UPhys/UPhysSettings")]
    public class UPhysSettings : ScriptableObject
    {
        #region Public
        // Solve Overlap
        public int DepentrationIteration => _depentrationIteration;
        public bool KillPositionWhenExceedDepentrationIteration => _killPositionWhenExceedDepentrationIteration;
        // Solve Velocity
        public int VelocityIteration => _velocityIteration;
        public bool KillPositionWhenExceedVelocityIteration => _killPositionWhenExceedVelocityIteration;
        public bool KillRemainedDistanceWhenExceedVelocityIteration => _killRemainedDistanceWhenExceedVelocityIteration;
        // Misc
        public float ContactOffset => _contactOffset;
        #endregion

        #region Private
        [Header("Solve Overlap")]
        [Tooltip("Accuracy of depentration solver; Higher costs more")]
        [SerializeField] private int _depentrationIteration = 1;
        [Tooltip("Canceal depentration interation")]
        [SerializeField] private bool _killPositionWhenExceedDepentrationIteration = false;
        [Tooltip("Accuracy of velocity solver; Higher costs more")]
        [Header("Solve Velocity")]
        [SerializeField] private int _velocityIteration = 3;
        [Tooltip("Canceal velocity interation")]
        [SerializeField] private bool _killPositionWhenExceedVelocityIteration = true;
        [Tooltip("Discard remained deistance but not appand")]
        [SerializeField] private bool _killRemainedDistanceWhenExceedVelocityIteration = true;
        [Header("Misc")]
        [Tooltip("Gap between character and others")]
        [SerializeField] private float _contactOffset = 0.02f;
        #endregion
    }
}
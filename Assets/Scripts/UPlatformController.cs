using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UPhys
{
    public class UPlatformController : MonoBehaviour
    {
        public Vector3 Velocity { get => _velocity; set => _velocity = value; }
        public Quaternion AngularVelocity { get => Quaternion.AngleAxis(_rotateSpeed * Mathf.Deg2Rad, transform.rotation * _rotateAxis); }
        public float RotateSpeed { get => _rotateSpeed; set => _rotateSpeed = value; }  
        public Vector3 RotateAxis { get => _rotateAxis; set => _rotateAxis = value; }

        public virtual void UpdateVelocity (float deltaTime) { }

        public void Simulate (float deltaTime)
        {
            UpdateVelocity(deltaTime);

            // Cache physics values before simulate
            _nextPos = _initPos = _rigidbody.position;
            _nextRot = _initRot = _rigidbody.rotation;
            
            _nextPos += _velocity * deltaTime;
            _nextRot *= Quaternion.AngleAxis(_rotateSpeed * deltaTime, _rotateAxis);
            
            _rigidbody.position = _nextPos;
            _rigidbody.rotation = _nextRot;

            transform.position = _nextPos;
            transform.rotation = _nextRot;
        }

        public void SimulateCommit ()
        {
            _rigidbody.position = _initPos;
            _rigidbody.rotation = _initRot;

            _rigidbody.MovePosition(_nextPos);
            _rigidbody.MoveRotation(_nextRot);
        }


        [SerializeField] private Vector3 _velocity;
        [SerializeField] private Vector3 _rotateAxis;
        [SerializeField] private float _rotateSpeed;

        private Rigidbody _rigidbody;

        private Vector3 _initPos;
        private Quaternion _initRot;

        private Vector3 _nextPos;
        private Quaternion _nextRot;


        private void Awake ()
        {
            if(!TryGetComponent(out _rigidbody))
            {
                throw new System.NullReferenceException($"{_rigidbody.GetType()} {gameObject.name} does not have rigidbody; Attach rigidbody.");
            }

            UPhysSystem.RegisterPlatformController(this);
        }
        private void OnDestroy ()
        {
            UPhysSystem.UnregisterPlatformController(this);
        }
    }
}
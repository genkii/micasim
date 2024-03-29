/*
 * Copyright (c) Contributors, http://opensimulator.org/
 * See CONTRIBUTORS.TXT for a full list of copyright holders.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the OpenSim Project nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE DEVELOPERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

using System;
using System.Collections.Generic;
using Axiom.Math;
using Nini.Config;
using OpenSim.Framework;
using OpenSim.Region.Physics.Manager;

namespace OpenSim.Region.Physics.NewtonPlugin
{
    public class NewtonPrim : PhysicsActor
    {

		private float _mass;
		private PhysicsVector _position;
        private PhysicsVector _velocity;
        private PhysicsVector _oldPosition;
        private PhysicsVector _oldVelocity;
        private PhysicsVector _acceleration;
        private PhysicsVector _oldAcceleration;
        private PhysicsVector _jerk;
        private PhysicsVector _oldJerk;
        private PhysicsVector _size;
		
		private PhysicsVector _a0;
		private PhysicsVector _j0;
		private PhysicsVector _a1;
		private PhysicsVector _a2;
		private PhysicsVector _j2;
		private bool _firstStep;
		private float _hOld;
		
        private PhysicsVector m_rotationalVelocity = PhysicsVector.Zero;
        private Quaternion _orientation;
        private bool iscolliding;
        private bool isPhysical;

        public NewtonPrim()
        {
			_mass = 1.0f;
            _velocity = new PhysicsVector();
            _position = new PhysicsVector();
            _acceleration = new PhysicsVector();
            _jerk = new PhysicsVector();
            _oldPosition = new PhysicsVector();
            _oldVelocity = new PhysicsVector();
            _oldAcceleration = new PhysicsVector();
            _oldJerk = new PhysicsVector();
			
			_a0 = new PhysicsVector();
			_j0 = new PhysicsVector();
			_a1 = new PhysicsVector();
			_a2 = new PhysicsVector();
			_j2 = new PhysicsVector();
			_firstStep = true;
        }

        public override int PhysicsActorType
        {
            get { return (int) ActorTypes.Prim; }
            set { return; }
        }

        public override PhysicsVector RotationalVelocity
        {
            get { return m_rotationalVelocity; }
            set { m_rotationalVelocity = value; }
        }

        public override bool IsPhysical
        {
            get { return isPhysical; }
            set { isPhysical = value; }
        }

        public override bool ThrottleUpdates
        {
            get { return false; }
            set { return; }
        }

        public override bool IsColliding
        {
            get { return iscolliding; }
            set { iscolliding = value; }
        }

        public override bool CollidingGround
        {
            get { return false; }
            set { return; }
        }

        public override bool CollidingObj
        {
            get { return false; }
            set { return; }
        }

        public override bool Stopped
        {
            get { return false; }
        }

        public PhysicsVector OldPosition
        {
          get { return _oldPosition; }
          set { _oldPosition = value; }
        }

        public override PhysicsVector Position
        {
            get { return _position; }
            set { _position = value; }
        }

        public override PhysicsVector Size
        {
            get { return _size; }
            set { _size = value; }
        }

        public override float Mass
        {
            get { return _mass; }

        }


		public void SetMass(float mass) {
			_mass = mass;
		}
		public override PhysicsVector Force
        {
            get { return PhysicsVector.Zero; }
            set { return; }
        }

        public override PhysicsVector CenterOfMass
        {
            get { return PhysicsVector.Zero; }
        }

        public override PhysicsVector GeometricCenter
        {
            get { return PhysicsVector.Zero; }
        }

        public override PrimitiveBaseShape Shape
        {
            set { return; }
        }

        public override float Buoyancy
        {
            get { return 0f; }
            set { return; }
        }

        public override bool FloatOnWater
        {
            set { return; }
        }

        public PhysicsVector OldVelocity
        {
          get { return _oldVelocity; }
          set { _oldVelocity = value; }
        }

        public override PhysicsVector Velocity
        {
            get { return _velocity; }
            set { _velocity = value; }
        }

        public override float CollisionScore
        {
            get { return 0f; }
            set { }
        }

        public override Quaternion Orientation
        {
            get { return _orientation; }
            set { _orientation = value; }
        }

        public PhysicsVector OldAcceleration
        {
          get { return _oldAcceleration; }
          set { _oldAcceleration = value; }
        }

        public override PhysicsVector Acceleration
        {
            get { return _acceleration; }
 //           set { _acceleration = value; }
        }

        public PhysicsVector Jerk 
        {
          get { return _jerk; }
          set { _jerk = value; }
        }

        public PhysicsVector OldJerk
        {
          get { return _oldJerk; }
          set { _oldJerk = value; }
        }

        public override bool Kinematic
        {
            get { return true; }
            set { }
        }

		public PhysicsVector A0 {
			get { return _a0; }
			set { _a0 = value; }
		}
		
		public PhysicsVector A1 {
			get { return _a1; }
			set { _a1 = value; }
		}
		
		public PhysicsVector A2 {
			get { return _a2; }
			set { _a2 = value; }
		}
		
		public PhysicsVector J0 {
			get { return _j0; }
			set { _j0 = value; }
		}
		
		public PhysicsVector J2 {
			get { return _j2; }
			set { _j2 = value; }
		}
		
		public bool FirstStep {
			get { return _firstStep; }
			set { _firstStep = value; }
		}
		
		public float HOld {
			get { return _hOld; }
			set { _hOld = value; }
		}
		

        // TODO: Replace with a setter once there is one in PhysicsActor to override
        public void SetAcceleration(PhysicsVector accel)
        {
            _acceleration = accel;
        }

        public void SetOldAcceleration(PhysicsVector accel) 
        {
          _oldAcceleration = accel;
        }

        public void SaveState() 
        {
			_oldPosition = _position;
			_oldVelocity = _velocity;
			_oldAcceleration = _acceleration;
			_oldJerk = _jerk;

        }
		
		public override void AddForce(PhysicsVector force, bool pushforce)
        {
        }

        public override void SetMomentum(PhysicsVector momentum)
        {
        }

        public override bool Flying
        {
            get { return false; }
            set { }
        }

        public override bool SetAlwaysRun
        {
            get { return false; }
            set { return; }
        }

        public override uint LocalID
        {
            set { return; }
        }

        public override bool Grabbed
        {
            set { return; }
        }

        public override void link(PhysicsActor obj)
        {
        }

        public override void delink()
        {
        }

        public override void LockAngularMotion(PhysicsVector axis)
        {
        }

        public override bool Selected
        {
            set { return; }
        }

        public override void CrossingFailure()
        {
        }

        public override PhysicsVector PIDTarget
        {
            set { return; }
        }

        public override bool PIDActive
        {
            set { return; }
        }

        public override float PIDTau
        {
            set { return; }
        }

        public override void SubscribeEvents(int ms)
        {
        }

        public override void UnSubscribeEvents()
        {
        }

        public override bool SubscribedEvents()
        {
            return false;
        }

        public float KineticEnergy() 
        {
          float v = Velocity.length();
          return 0.5f*Mass*v*v;
        }
    }
}

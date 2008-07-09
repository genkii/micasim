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
using System.Reflection;
using Axiom.Math;
using Nini.Config;
using OpenSim.Framework;
using OpenSim.Region.Physics.Manager;
using log4net;

namespace OpenSim.Region.Physics.NewtonPlugin
{
    public class NewtonScene : PhysicsScene
    {
		private static readonly ILog m_log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);
        private List<NewtonCharacter> _characters = new List<NewtonCharacter>();
        private List<NewtonPrim> _prims = new List<NewtonPrim>();
        private float[] _heightMap;
        private const float gravity = -9.8f;
        private const float eps = 1.0f; // Softening distance. 

		private const int energyInterval = 10;
		private int energyCounter = 0;
		

		public NewtonScene()
        {
        }

        public override void Initialise(IMesher meshmerizer, IConfigSource config)
        {
			
        }

        public override void Dispose()
        {
        }

        public override PhysicsActor AddAvatar(string avName, PhysicsVector position, PhysicsVector size)
        {
            NewtonCharacter act = new NewtonCharacter();
            act.Position = position;
            _characters.Add(act);
            return act;
        }

        public override void SetWaterLevel(float baseheight)
        {
        }

        public override void RemovePrim(PhysicsActor prim)
        {
            NewtonPrim p = (NewtonPrim) prim;
            if (_prims.Contains(p))
            {
                _prims.Remove(p);
            }
        }

        public override void RemoveAvatar(PhysicsActor character)
        {
            NewtonCharacter act = (NewtonCharacter) character;
            if (_characters.Contains(act))
            {
                _characters.Remove(act);
            }
        }

/*
        public override PhysicsActor AddPrim(PhysicsVector position, PhysicsVector size, Quaternion rotation)
        {
            return null;
        }
*/

        public override PhysicsActor AddPrimShape(string primName, PrimitiveBaseShape pbs, PhysicsVector position,
                                                  PhysicsVector size, Quaternion rotation)
        {
            return AddPrimShape(primName, pbs, position, size, rotation, false);
        }

        public override PhysicsActor AddPrimShape(string primName, PrimitiveBaseShape pbs, PhysicsVector position,
                                                  PhysicsVector size, Quaternion rotation, bool isPhysical)
        {
            NewtonPrim prim = new NewtonPrim();
            prim.Position = position;
            prim.Orientation = rotation;
            prim.Size = size;
            prim.IsPhysical = isPhysical;

            lock (_prims)
            {
                _prims.Add(prim);
            }
            return prim;
        }

        public override void AddPhysicsActorTaint(PhysicsActor prim)
        {
        }

        /// <summary>
        /// Returns the square of the distance between two points
        /// </summary>
        private float distance2(PhysicsVector a, PhysicsVector b)
        {
          float dx = a.X - b.X;
          float dy = a.Y - b.Y;
          float dz = a.Z - b.Z;

          // Math.Pow probably is a lot slower than dx*dx.
          return dx*dx + dy*dy + dz*dz;
          // return (float) (Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2) + Math.Pow(a.Z - b.Z, 2));
        }

        private float dot(PhysicsVector a, PhysicsVector b) 
        {
          return a.X*b.X + a.Y*b.Y + a.Z*b.Z;
        }

        private void calculateAcceleration(int i)
        {
            PhysicsVector accel = new PhysicsVector();

            for (int j = 0; j < _prims.Count; ++j)
            {
                if (i == j || !_prims[j].IsPhysical)
                    continue;

                PhysicsVector direction = _prims[j].Position - _prims[i].Position;
                float dist2 = distance2(_prims[j].Position, _prims[i].Position);
                float r = (float) Math.Sqrt(dist2 + eps*eps); // r is softened radius.

                accel += Math.Abs(gravity) * _prims[j].Mass / (r*r*r) * direction;
            }

            _prims[i].SetAcceleration(accel);
        }

        private void calculateAccAndJerk(int i) {
          PhysicsVector acc = new PhysicsVector();
          PhysicsVector jerk = new PhysicsVector();

          for (int j = 0; j < _prims.Count; j++) 
            {
              if (i == j || !_prims[j].IsPhysical)
                continue;

              PhysicsVector direction = _prims[j].Position - _prims[i].Position;
              PhysicsVector velocity = _prims[j].Velocity - _prims[i].Velocity;
              float dist2 = distance2(_prims[j].Position, _prims[i].Position);
              float r = (float) Math.Sqrt(dist2 + eps*eps);
              float r2 = r*r;
              float r3 = r2*r;
              float r5 = r2*r3;

              acc += Math.Abs(gravity) * _prims[j].Mass * direction / r3;
              jerk += Math.Abs(gravity) * _prims[j].Mass * (velocity/r3 - 3.0f*dot(direction, velocity)/r5 * direction);
            }
			_prims[i].SetAcceleration(acc);

			_prims[i].Jerk = jerk;
        }

        private void savePrimStates() 
        {
          for (int j = 0; j < _prims.Count; j++) 
            {
              _prims[j].SaveState();
            }
        }

        private void predictPrims(float dt) 
        {
          float dt2 = dt*dt;
          float dt3 = dt*dt2;

          for (int j = 0; j < _prims.Count; j++)
            {
              _prims[j].Position = _prims[j].Position + dt*_prims[j].Velocity + 0.5f*dt2*_prims[j].Acceleration + (1.0f/6.0f)*dt3*_prims[j].Jerk;
              _prims[j].Velocity = _prims[j].Velocity + dt*_prims[j].Acceleration + 0.5f*dt2*_prims[j].Jerk;
            }
        }

        private void finishPrims(float dt) 
        {
          float dt2 = dt*dt;

          for (int j = 0; j < _prims.Count; j++) 
            {
				// Important to do velocity first, so that more accurate velocity value can be used in position calculation.
				_prims[j].Velocity = _prims[j].OldVelocity + 0.5f*(_prims[j].OldAcceleration + _prims[j].Acceleration)*dt + 
					(1.0f/12.0f)*(_prims[j].OldJerk - _prims[j].Jerk)*dt2;
				_prims[j].Position = _prims[j].OldPosition + 0.5f*(_prims[j].OldVelocity + _prims[j].Velocity)*dt + 
					(1.0f/12.0f)*(_prims[j].OldAcceleration - _prims[j].Acceleration)*dt2;
				periodicBCs(_prims[j].Position);
            }
        }

        private void updatePrimVelocities(float timestep)
        {
            lock (_prims)
            {
                for (int i = 0; i < _prims.Count; ++i)
                {
                    if (!_prims[i].IsPhysical)
                        continue;

                    calculateAcceleration(i);

                    _prims[i].Velocity += timestep * _prims[i].Acceleration;
                }
            }
        }


		private void periodicBCs(PhysicsVector pos)
		{
			float smallestPos = 0.01f;
			float largestPos = Constants.RegionSize - 0.01f;
			
			if (pos.X < smallestPos) 
			{
				pos.X = largestPos;
			} else if (pos.X > largestPos) {
				pos.X = smallestPos;
			}
			
			if (pos.Y < smallestPos)
			{
				pos.Y = largestPos;
			} else if (pos.Y > largestPos) {
				pos.Y = smallestPos;
			}
				
		}
		private void updatePrimPositions(float timestep)
        {
            for (int i = 0; i < _prims.Count; ++i)
            {
                if (!_prims[i].IsPhysical)
                    continue;

				_prims[i].Position += timestep * _prims[i].Velocity;

				periodicBCs(_prims[i].Position);
			}
        }

        private void updateCharacter(NewtonCharacter character, float timestep)
        {
            float oldposX = character.Position.X;
            float oldposY = character.Position.Y;
            float oldposZ = character.Position.Z;

            if (!character.Flying)
            {
                character._target_velocity.Z += gravity * timestep;
            }

            character.Position.X += character._target_velocity.X * timestep;
            character.Position.Y += character._target_velocity.Y * timestep;

            character.Position.X = Util.Clamp(character.Position.X, 0.01f, Constants.RegionSize - 0.01f);
            character.Position.Y = Util.Clamp(character.Position.Y, 0.01f, Constants.RegionSize - 0.01f);

            bool forcedZ = false;

            float terrainheight = _heightMap[(int)character.Position.Y * Constants.RegionSize + (int)character.Position.X];
            if (character.Position.Z + (character._target_velocity.Z * timestep) < terrainheight + 2)
            {
                character.Position.Z = terrainheight + 1.0f;
                forcedZ = true;
            }
            else
            {
                character.Position.Z += character._target_velocity.Z * timestep;
            }

            character._velocity.X = (character.Position.X - oldposX) / timestep;
            character._velocity.Y = (character.Position.Y - oldposY) / timestep;

            if (forcedZ)
            {
                character._velocity.Z = 0;
                character._target_velocity.Z = 0;
                ((PhysicsActor)character).IsColliding = true;
                character.RequestPhysicsterseUpdate();
            }
            else
            {
                ((PhysicsActor)character).IsColliding = false;
                character._velocity.Z = (character.Position.Z - oldposZ) / timestep;
            }
        }

        private float simulateCharacters(float timestep) 
        {
          float fps = 0;
          for (int i = 0; i < _characters.Count; ++i) 
          {
            fps++;
            updateCharacter(_characters[i], timestep);
          }

          return fps;
        }

		
		public override float Simulate(float timestep)
		{

			float result = simulateCharacters(timestep);
			for (int i = 0; i < 10; i++) {
				SimulateHermite(timestep/10.0f);
			}

			
			if (((++energyCounter) % energyInterval) == 0)
			{
				m_log.Info(Energy());
				energyCounter = 0;
			}	
			
			return result;
		}
		

        private void SimulateForwardEuler(float timestep)
        {          
			updatePrimVelocities(timestep);
			updatePrimPositions(timestep);


		}

        private void SimulateDKD(float timestep) 
        {
          updatePrimPositions(0.5f*timestep);
          updatePrimVelocities(timestep);
          updatePrimPositions(0.5f*timestep);

		}

        // This re-computes the force on each subsequent timestep:
        // (KDK)(KDK)(KDK)....  Usually, you would store the results
        // of the force computation, and re-use them in a
        // "Kick-stored" advance so there is only one force
        // computation per timestep on average: (KDK)(KsDK)(KsDK)....
        private void SimulateKDK(float timestep)
        {
          updatePrimVelocities(0.5f*timestep);
          updatePrimPositions(timestep);
          updatePrimVelocities(0.5f*timestep);

		}

        private void SimulateHermite(float timestep) 
        {
			for (int i = 0; i < _prims.Count; i++) {
				calculateAccAndJerk(i);

			}
			savePrimStates();
			predictPrims(timestep);
			for (int i = 0; i < _prims.Count; i++) {
				calculateAccAndJerk(i);
			}
			finishPrims(timestep);

		}

        public override void GetResults()
        {
        }

        public override bool IsThreaded
        {
            // for now we won't be multithreaded
            get { return (false); }
        }

        public override void SetTerrain(float[] heightMap)
        {
            _heightMap = heightMap;
        }

        public override void DeleteTerrain()
        {
        }

        public override Dictionary<uint, float> GetTopColliders()
        {
            Dictionary<uint, float> returncolliders = new Dictionary<uint, float>();
            return returncolliders;
        }

        private float PairPotential(NewtonPrim p1, NewtonPrim p2) 
        {
          float d2 = distance2(p1.Position, p2.Position);
          float r = (float) Math.Sqrt(d2 + eps*eps);

          return -Math.Abs(gravity)*p1.Mass*p2.Mass/r;
        }

        public float KineticEnergy() 
        {
          float ke = 0.0f;

          for (int i = 0; i < _prims.Count; i++) 
            {
              ke += _prims[i].KineticEnergy();
            }

          return ke;
        }

        public float PotentialEnergy() 
        {
          float pe = 0.0f;

          for (int i = 0; i < _prims.Count; i++) 
            {
              for (int j = i+1; j < _prims.Count; j++) 
                {
                  pe += PairPotential(_prims[i], _prims[j]);
                }
            }

          return pe;
        }

        public float Energy() 
        {
          return KineticEnergy() + PotentialEnergy();
        }
		
		private void gglPredict(float h) {
			float h2 = h*h;
			float h3 = h2*h;
			float h4 = h3*h;
			float h5 = h4*h;
			float h6 = h5*h;
			
			for (int i = 0; i < _prims.Count; i++) {
				float hold = _prims[i].HOld;
				float hold2 = hold*hold;
				float hold3 = hold2*hold;
				float hold4 = hold3*hold;
				PhysicsVector a0 = _prims[i].A0;
				PhysicsVector a1 = _prims[i].A1;
				PhysicsVector a2 = _prims[i].A2;
				PhysicsVector j0 = _prims[i].J0;
				PhysicsVector j2 = _prims[i].J2;


				
				_prims[i].OldPosition = _prims[i].Position;
				_prims[i].OldVelocity = _prims[i].Velocity;
				_prims[i].HOld = h;
				
				_prims[i].Position = _prims[i].Position + 0.5f*h*_prims[i].Velocity + h2*a2/8.0f + j2*h3/48.0f - (5.0f*a0 - 16.0f*a1 + 11.0f*a2 + hold*j0 - 4.0f*hold*j2)*h4/(96.0f*hold2) - (14.0f*a0 - 32.0f*a1 + 18.0f*a2 + 3.0f*hold*j0 - 5.0f*hold*j2)*h5/(192.0f*hold3) - (4.0f*a0 - 8.0f*a1 + 4.0f*a2 + hold*j0 - hold*j2)*h6/(192.0f*hold4);
				
				_prims[i].A0 = _prims[i].A2;
				_prims[i].J0 = _prims[i].J0;
				_prims[i].FirstStep = false;
			}
		}
		
		private void gglSaveA0J0() {
			for (int i = 0; i < _prims.Count; i++) {
				_prims[i].A0 = _prims[i].Acceleration;
				_prims[i].J0 = _prims[i].Jerk;
			}
		}
		
		private void gglSaveA1() {
			for (int i = 0; i < _prims.Count; i++) {
				_prims[i].A1 = _prims[i].Acceleration;
			}
		}
		
		private void gglSaveA2() {
			for (int i = 0; i < _prims.Count; i++) {
				_prims[i].A2 = _prims[i].Acceleration;
			}
		}
		
		private void gglSaveA2J2() {
			for (int i = 0; i < _prims.Count; i++) {
				_prims[i].A2 = _prims[i].Acceleration;
				_prims[i].J2 = _prims[i].Jerk;
			}
		}
		
		private void gglFinalPosition(float h) {
			float h2 = h*h;
			
			for (int i = 0; i < _prims.Count; i++) {
				_prims[i].Position = _prims[i].OldPosition + h*_prims[i].Velocity + 0.5f*h2*(_prims[i].A0/3.0f + 2.0f*_prims[i].A1/3.0f);
			}
		}
		
		private void gglFinalVelocity(float h) {
			for (int i = 0; i < _prims.Count; i++) {
				_prims[i].Velocity = _prims[i].Velocity + h*(_prims[i].A0/6.0f + 2.0f*_prims[i].A1/3.0f + _prims[i].A2/6.0f);
			}
		}
		
		private void SimulateGGL(float timestep) {
			// Need something for first timestep.
			
			gglPredict(timestep);
			for (int i = 0; i < _prims.Count; i++) {
				calculateAcceleration(i);
			}
			gglSaveA1();
			gglFinalPosition(timestep);
			for (int i = 0; i < _prims.Count; i++) {
				calculateAcceleration(i);
			}
			gglSaveA2();
			gglFinalVelocity(timestep);
			for (int i = 0; i < _prims.Count; i++) {
				calculateAccAndJerk(i);
			}
			gglSaveA2J2();
			
		}
    }
}
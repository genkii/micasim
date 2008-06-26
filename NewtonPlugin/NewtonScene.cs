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
    public class NewtonScene : PhysicsScene
    {
        private List<NewtonCharacter> _characters = new List<NewtonCharacter>();
        private List<NewtonPrim> _prims = new List<NewtonPrim>();
        private float[] _heightMap;
        private const float gravity = -9.8f;

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
            return (float) (Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2) + Math.Pow(a.Z - b.Z, 2));
        }

        private void calculateAcceleration(int i)
        {
            // if prims are closer than this distance, gravitational forces between them are ignored
            const float softeningDistance = 1.0f;

            PhysicsVector accel = new PhysicsVector();

            for (int j = 0; j < _prims.Count; ++j)
            {
                if (i == j || !_prims[j].IsPhysical)
                    continue;

                float dist2 = distance2(_prims[i].Position, _prims[j].Position);

                if (dist2 < softeningDistance)
                    continue;

                PhysicsVector direction = _prims[j].Position - _prims[i].Position;
                direction /= direction.length();

                accel += Math.Abs(gravity) * _prims[j].Mass / dist2 * direction;
            }

            _prims[i].SetAcceleration(accel);
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

        private void updatePrimPositions(float timestep)
        {
            for (int i = 0; i < _prims.Count; ++i)
            {
                if (!_prims[i].IsPhysical)
                    continue;

                _prims[i].Position += timestep * _prims[i].Velocity;

                _prims[i].Position.X = Util.Clamp(_prims[i].Position.X, 0.01f, Constants.RegionSize - 0.01f);
                _prims[i].Position.Y = Util.Clamp(_prims[i].Position.Y, 0.01f, Constants.RegionSize - 0.01f);

                float terrainheight = _heightMap[(int)_prims[i].Position.Y * Constants.RegionSize + (int)_prims[i].Position.X];

                if (_prims[i].Position.Z < terrainheight)
                    _prims[i].Position.Z = terrainheight;
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

        public override float Simulate(float timestep)
        {
            float fps = 0;
            for (int i = 0; i < _characters.Count; ++i)
            {
                fps++;
                
                updateCharacter(_characters[i], timestep);
            }

            updatePrimVelocities(timestep);
            updatePrimPositions(timestep);

            return fps;
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
    }
}

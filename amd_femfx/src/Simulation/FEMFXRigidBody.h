/*
MIT License

Copyright (c) 2019 Advanced Micro Devices, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

//---------------------------------------------------------------------------------------
// Rigid body definition
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommonInternal.h"
#include "FEMFXAtomicOps.h"
#include "FEMFXBvh.h"
#include "FEMFXRigidBodyState.h"
#include "FEMFXGraphColoring.h"
#include "FEMFXVelStats.h"

namespace AMD
{
    struct FmTetMeshBuffer;

    // Representation of rigid body in FEM system
    struct FmRigidBody
    {
        uint                  objectId = FM_INVALID_ID;
        uint                  rbIslandId = FM_INVALID_ID;                                   // Island id assigned by rigid body engine; The FEM library will group FEM objects in the same island if connected to the same rigid body island.
        uint                  femIslandId = FM_INVALID_ID;                                  // Id of island that contains this body in FEM scene.
        FmRigidBodyState      state;                                                        // Dynamic state of rigid body
        float                 mass = 1.0f;                                                  // Mass of rigid body
        float                 frictionCoeff = FM_DEFAULT_FRICTION_COEFF;                    // Used for contacts with this body
        FmMatrix3             bodyInertiaTensor = FmMatrix3::scale(FmVector3(0.1666666f));  // Inertial tensor in rigid body's local space
        FmMatrix3             worldInertiaTensor = FmMatrix3::scale(FmVector3(0.1666666f)); // Current inertial tensor in world space
        FmVector3             gravityVector = FmVector3(0.0f);                              // Gravity acceleration, 0 by default; added with FmSceneControlParams::gravityVector

        float                 dims[3] = { 0.5f, 0.5f, 0.5f };    // Shape dimensions (box half-widths)
        float                 maxRadius = 0.866025403f;          // Largest distance from center to point on rigid body, used to bound motion for CCD
        FmTetMeshBuffer*      collisionObj = nullptr;            // Used for intersection with FEM tet meshes
        FmAabb                aabb;                              // AABB for rigid body
        uint8_t               collisionGroup = 0;                // Collision group index < 32

        // These are updated after each pass of solving or stabilization.
        // In a solver pass, deltaVel is change in velocity and deltaAngVel is change in angular velocity.
        // In a stabilization pass, deltaPos is change in position, deltaAngPos is change in "angular position", which can be seen treated as integrated angular velocity.
        // When a user solves external constraints, these should be updated prior to running the next pass.
        FmVector3             deltaVel;
        FmVector3             deltaAngVel;
        FmVector3             deltaPos;
        FmVector3             deltaAngPos;

        bool                  foundInConstraint = false;       // Used to decide whether to add rigid body to constraint island

        float                 sleepMaxSpeedThreshold = 2.0f;   // Max speed must be under this threshold for sleeping 
        float                 sleepAvgSpeedThreshold = 0.15f;  // Average speed must be under this threshold for sleeping
        uint                  sleepStableCount = 15;           // Number of steps that speeds must fall under thresholds to initiate sleeping

        FmVelStats            velStats;                        // Statistics for sleeping test

        void*                 userData = nullptr;              // User data pointer
        uint16_t              flags = 0;                       // Bitwise or of FM_OBJECT_FLAG* values

        inline void Init()
        {
            *this = FmRigidBody();
        }

        inline void SetDimensions(float hx, float hy, float hz)
        {
            dims[0] = hx;
            dims[1] = hy;
            dims[2] = hz;
            maxRadius = sqrtf(hx * hx + hy * hy + hz * hz);
        }

        inline void SetMassMatrixForBox(float hx, float hy, float hz, float inMass)
        {
            mass = inMass;
            bodyInertiaTensor = FmComputeBodyInertiaTensorForBox(hx, hy, hz, inMass);
            worldInertiaTensor = bodyInertiaTensor;
            SetDimensions(hx, hy, hz);
        }
    };

    static inline FmMatrix3 FmTransformBodyInertiaToWorld(const FmMatrix3& inertiaTensorBody, const FmMatrix3& bodyToWorld)
    {
        return (bodyToWorld * inertiaTensorBody) * transpose(bodyToWorld);
    }

    static inline void FmStepState(FmRigidBodyState& state, float dt)
    {
        state.pos = state.pos + state.vel * dt;
        state.quat = FmIntegrateQuat(state.quat, state.angVel, dt);
    }

    static inline FmVector3 FmComputeTorque(const FmVector3& force, const FmVector3& comToForcePos)
    {
        return cross(comToForcePos, force);
    }

    // Angular term of 6D force = torque - angVel X (inertiaTensor * angVel)
    static inline FmVector3 FmComputeAngularForce(const FmVector3& torque, const FmMatrix3& inertiaTensor, const FmVector3& angVel)
    {
        return torque - cross(angVel, inertiaTensor * angVel);
    }

    static FM_FORCE_INLINE void FmDynamicAndMovingFlags(uint* dynamicFlags, uint* movingFlags, const FmRigidBody& rigidBody)
    {
        int dynMask = -(int)FM_NOT_SET(rigidBody.flags, FM_OBJECT_FLAG_KINEMATIC);
        int movMask = -(int)!FmIsZero(rigidBody.state.vel + rigidBody.state.angVel);

        *dynamicFlags = dynMask & 0x3;  // Using two bits for a rigid body corresponding to two 3D states, vel and angvel 
        *movingFlags = movMask & 0x3;
    }

    void FmSetRigidBodyIsland(FmRigidBody* rigidBody, uint rigidBodyIslandId);
    void FmSetAabb(FmRigidBody* rigidBody, const FmAabb& inAabb);

    // Create and destroy box collision object for a rigid body
    void FmCreateRigidBodyBoxCollObj(FmRigidBody* rigidBody);
    void FmDestroyRigidBodyCollObj(FmRigidBody* rigidBody);
}
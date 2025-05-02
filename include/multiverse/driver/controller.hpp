// VehicleController.h
// Unified controller supporting Ackermann (front-wheel steering) and Differential (2-wheel) drive

#pragma once
#include "muli/world.h"
#include <cassert>

namespace mvs {

    using namespace muli;

    class VehicleController {
      public:
        enum class DriveMode { Ackermann, Differential };

        struct Wheel {
            RigidBody *body = nullptr;
            Vec2 forward, lateral;
            float driveForce = 0.f;
            float lateralFriction = 0.f;
            float maxLateralImpulse = 0.f;
            float dragCoefficient = 0.f;

            void init(muli::World *world, float radius, const Vec2 &localPos, const CollisionFilter &filter,
                      float linearDamp, float angularDamp, float driveF, float latFric, float maxImp, float dragC) {
                body = world->CreateCapsule(radius, radius);
                body->SetCollisionFilter(filter);
                Transform tf;
                tf.position = localPos;
                body->SetTransform(tf);
                body->SetLinearDamping(linearDamp);
                body->SetAngularDamping(angularDamp);

                driveForce = driveF;
                lateralFriction = latFric;
                maxLateralImpulse = maxImp;
                dragCoefficient = dragC;
            }

            void ApplyDrive(float throttle) {
                Vec2 f = forward * (driveForce * throttle);
                body->ApplyForce(body->GetPosition(), f, true);
            }

            void Step() {
                forward = Mul(body->GetRotation(), Vec2(0, 1));
                lateral = Mul(body->GetRotation(), Vec2(1, 0));
                Vec2 v = body->GetLinearVelocity();

                // lateral friction
                float latVel = Dot(v, lateral);
                if (fabs(latVel) > 1e-3f) {
                    Vec2 j = -body->GetMass() * lateralFriction * latVel * lateral;
                    if (Length(j) > maxLateralImpulse)
                        j = Normalize(j) * maxLateralImpulse;
                    body->ApplyLinearImpulse(body->GetPosition(), j, true);
                }
                // rolling drag
                float fwdVel = Dot(v, forward);
                if (fabs(fwdVel) > 1e-3f) {
                    Vec2 df = -dragCoefficient * fwdVel * forward;
                    body->ApplyForce(body->GetPosition(), df, true);
                }
            }
        };

        // Public API: unified init
        void init(muli::World *world, DriveMode mode, const Vec2 &chassisSize, const Vec2 &chassisPose, float wheelR,
                  const Vec2 offsets[], size_t count, const CollisionFilter &filter) {
            if (mode == DriveMode::Ackermann && count == 4) {
                InitAckermann(world, chassisSize, chassisPose, wheelR, reinterpret_cast<const Vec2(&)[4]>(offsets),
                              filter);
            } else if (mode == DriveMode::Differential && count == 2) {
                InitDifferential(world, chassisSize, chassisPose, wheelR, reinterpret_cast<const Vec2(&)[2]>(offsets),
                                 filter);
            } else {
                assert(false && "Invalid drive mode or wheel count");
            }
        }

        // Set desired linear (m/s) and angular (rad/s) velocity
        void update(float linearVel, float angularVel) {
            if (mode == DriveMode::Ackermann) {
                float throttle = linearVel;
                float delta = (fabs(linearVel) > 1e-3f) ? atan2(angularVel * wheelBase, linearVel)
                                                        : (angularVel > 0 ? steerAngleMax : -steerAngleMax);
                delta = fmaxf(fminf(delta, steerAngleMax), -steerAngleMax);
                float steer[4] = {delta, delta, 0.f, 0.f};
                ApplyControl(steer, throttle);
            } else {
                float vl = linearVel - angularVel * (trackWidth * 0.5f);
                float vr = linearVel + angularVel * (trackWidth * 0.5f);
                float m = fmaxf(fabs(vl), fabs(vr));
                if (m > 1.f) {
                    vl /= m;
                    vr /= m;
                }
                float steer[4] = {vl, vr, 0.f, 0.f};
                ApplyControl(steer, 0.f);
            }
        }

        // Step simulation for each wheel
        void tick() {
            size_t cnt = (mode == DriveMode::Ackermann) ? 4 : 2;
            for (size_t i = 0; i < cnt; ++i)
                wheels[i].Step();
        }

        void teleport(const Vec2 &pos) { chassis->SetPosition(pos); }

      private:
        // Ackermann-specific init
        void InitAckermann(muli::World *world, const Vec2 &chassisSize, const Vec2 &chassisPose, float wheelR,
                           const Vec2 (&offs)[4], const CollisionFilter &filter) {
            mode = DriveMode::Ackermann;
            wheelRadius = wheelR;
            wheelBase = fabs(offs[0].y - offs[2].y);
            trackWidth = fabs(offs[0].x - offs[1].x);

            chassis = world->CreateBox(chassisSize.x, chassisSize.y);
            chassis->SetPosition(chassisPose);
            chassis->SetCollisionFilter(filter);
            chassis->SetLinearDamping(linearDamping);
            chassis->SetAngularDamping(angularDamping);

            for (int i = 0; i < 4; ++i) {
                wheels[i].init(world, wheelRadius, offs[i], filter, linearDamping, angularDamping, driveForce,
                               lateralFriction, maxLateralImpulse, dragCoefficient);
                if (i < 2) {
                    steerJoints[i] = world->CreateMotorJoint(chassis, wheels[i].body, wheels[i].body->GetPosition(),
                                                             0.0f, steerTorque, 0.0f, 0.5f, chassis->GetMass());
                } else {
                    wheelJoints[i] = world->CreateRevoluteJoint(chassis, wheels[i].body, wheels[i].body->GetPosition());
                }
            }
        }

        // Differential-specific init
        void InitDifferential(muli::World *world, const Vec2 &chassisSize, const Vec2 &chassisPose, float wheelR,
                              const Vec2 (&offs)[2], const CollisionFilter &filter) {
            mode = DriveMode::Differential;
            wheelRadius = wheelR;
            trackWidth = fabs(offs[0].x - offs[1].x);

            chassis = world->CreateBox(chassisSize.x, chassisSize.y);
            chassis->SetPosition(chassisPose);
            chassis->SetCollisionFilter(filter);
            chassis->SetLinearDamping(linearDamping);
            chassis->SetAngularDamping(angularDamping);

            for (int i = 0; i < 2; ++i) {
                wheels[i].init(world, wheelRadius, offs[i], filter, linearDamping, angularDamping, driveForce,
                               lateralFriction, maxLateralImpulse, dragCoefficient);
                wheelJoints[i] = world->CreateRevoluteJoint(chassis, wheels[i].body, wheels[i].body->GetPosition());
            }
        }

        // Apply steering/throttle inputs
        void ApplyControl(const float steerIn[4], float throttle) {
            if (mode == DriveMode::Ackermann) {
                for (int i = 0; i < 2; ++i)
                    steerJoints[i]->SetAngularOffset(steerIn[i]);
                for (int i = 2; i < 4; ++i)
                    wheels[i].ApplyDrive(throttle);
            } else {
                wheels[0].ApplyDrive(steerIn[0]);
                wheels[1].ApplyDrive(steerIn[1]);
            }
        }

        // Members
        DriveMode mode = DriveMode::Ackermann;
        RigidBody *chassis = nullptr;
        Wheel wheels[4];
        MotorJoint *steerJoints[2] = {nullptr, nullptr};
        Joint *wheelJoints[4] = {nullptr, nullptr, nullptr, nullptr};

        // Parameters
        float linearDamping = 0.2f;
        float angularDamping = 2.0f;
        float wheelRadius = 0.2f;
        float driveForce = 30.f;
        float lateralFriction = 0.4f;
        float maxLateralImpulse = 0.5f;
        float dragCoefficient = 0.5f;
        float steerAngleMax = 35.0f * (3.1415926f / 180.0f);
        float steerTorque = 10.f;
        float wheelBase = 1.0f;
        float trackWidth = 1.0f;
    };

} // namespace mvs

// VehicleController.h
// Unified controller supporting Ackermann (front-wheel steering) and Differential (2-wheel) drive

#pragma once
#include "muli/world.h"

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

            void Init(World *world, float radius, const Vec2 &localPos, const CollisionFilter &filter, float linearDamp,
                      float angularDamp, float driveF, float latFric, float maxImp, float dragC) {
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

                // lateral friction (side-slip suppression)
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

        DriveMode mode = DriveMode::Ackermann;
        RigidBody *chassis = nullptr;
        Wheel wheels[4]; // 0=front-left,1=front-right,2=rear-left,3=rear-right
        MotorJoint *steerJoints[2] = {nullptr, nullptr};
        Joint *wheelJoints[4] = {nullptr, nullptr, nullptr, nullptr};

        // Physical and geometry parameters
        float linearDamping = 0.2f;
        float angularDamping = 2.0f;
        float wheelRadius = 0.2f;
        float driveForce = 30.f;
        float lateralFriction = 0.4f;
        float maxLateralImpulse = 0.5f;
        float dragCoefficient = 0.5f;
        float steerAngleMax = 35.0f * (3.1415926f / 180.0f); // max steering angle
        float steerTorque = 10.f;

        // Derived kinematics parameters for steering computation
        float wheelBase = 1.0f;  // distance between front and rear axle
        float trackWidth = 1.0f; // distance between left and right wheels

        // Initialize as 4-wheel Ackermann drive
        // wheelOffsets: FL, FR, RL, RR (relative to chassis center)
        void InitAckermann(World *world, const Vec2 &chassisSize, float wheelR, const Vec2 wheelOffsets[4],
                           const CollisionFilter &filter) {
            mode = DriveMode::Ackermann;
            wheelRadius = wheelR;
            // compute wheelBase and trackWidth from offsets
            wheelBase = fabs(wheelOffsets[0].y - wheelOffsets[2].y);
            trackWidth = fabs(wheelOffsets[0].x - wheelOffsets[1].x);

            chassis = world->CreateBox(chassisSize.x, chassisSize.y);
            chassis->SetCollisionFilter(filter);
            chassis->SetLinearDamping(linearDamping);
            chassis->SetAngularDamping(angularDamping);

            for (int i = 0; i < 4; ++i) {
                wheels[i].Init(world, wheelRadius, wheelOffsets[i], filter, linearDamping, angularDamping, driveForce,
                               lateralFriction, maxLateralImpulse, dragCoefficient);
                if (i < 2) {
                    // front wheels: steering joints
                    steerJoints[i] = world->CreateMotorJoint(chassis, wheels[i].body, wheels[i].body->GetPosition(),
                                                             0.0f, steerTorque, 0.0f, 0.5f, chassis->GetMass());
                } else {
                    // rear wheels: free-spinning via revolute joints
                    wheelJoints[i] = world->CreateRevoluteJoint(chassis, wheels[i].body, wheels[i].body->GetPosition());
                }
            }
        }

        // Initialize as 2-wheel Differential drive
        // wheelOffsets: Left, Right
        void InitDifferential(World *world, const Vec2 &chassisSize, const Vec2 wheelOffsets[2],
                              const CollisionFilter &filter) {
            mode = DriveMode::Differential;
            wheelBase = 0.0f;
            trackWidth = fabs(wheelOffsets[0].x - wheelOffsets[1].x);

            chassis = world->CreateBox(chassisSize.x, chassisSize.y);
            chassis->SetCollisionFilter(filter);
            chassis->SetLinearDamping(linearDamping);
            chassis->SetAngularDamping(angularDamping);

            for (int i = 0; i < 2; ++i) {
                wheels[i].Init(world, wheelRadius, wheelOffsets[i], filter, linearDamping, angularDamping, driveForce,
                               lateralFriction, maxLateralImpulse, dragCoefficient);
                wheelJoints[i] = world->CreateRevoluteJoint(chassis, wheels[i].body, wheels[i].body->GetPosition());
            }
        }

        // Update desired linear (m/s) and angular (rad/s) velocity
        void UpdateKinematics(float linearVel, float angularVel) {
            if (mode == DriveMode::Ackermann) {
                // throttle is proportional to forward velocity
                float throttle = linearVel;
                // compute steering angle delta = atan(omega * L / v)
                float delta;
                if (fabs(linearVel) > 1e-3f)
                    delta = atan2(angularVel * wheelBase, linearVel);
                else
                    delta = (angularVel > 0 ? steerAngleMax : -steerAngleMax);
                // clamp to max steer angle
                delta = fmaxf(fminf(delta, steerAngleMax), -steerAngleMax);
                // apply same steering to both front wheels
                float steeringInputs[4] = {delta, delta, 0.f, 0.f};
                UpdateControl(steeringInputs, throttle);
            } else {
                // differential: v_l = v - omega * (trackWidth/2)
                float vl = linearVel - angularVel * (trackWidth * 0.5f);
                float vr = linearVel + angularVel * (trackWidth * 0.5f);
                // normalize if needed to [-1,1]
                float mx = fmaxf(fabs(vl), fabs(vr));
                if (mx > 1.f) {
                    vl /= mx;
                    vr /= mx;
                }
                float steeringInputs[4] = {vl, vr, 0.f, 0.f};
                UpdateControl(steeringInputs, 0.f);
            }
        }

        // internal legacy update (steering+throttle)
        void UpdateControl(const float steeringInputs[4], float throttle) {
            if (mode == DriveMode::Ackermann) {
                for (int i = 0; i < 2; ++i)
                    steerJoints[i]->SetAngularOffset(steeringInputs[i]);
                for (int i = 2; i < 4; ++i)
                    wheels[i].ApplyDrive(throttle);
            } else {
                wheels[0].ApplyDrive(steeringInputs[0]);
                wheels[1].ApplyDrive(steeringInputs[1]);
            }
        }

        void Step() {
            int count = (mode == DriveMode::Ackermann) ? 4 : 2;
            for (int i = 0; i < count; ++i)
                wheels[i].Step();
        }
    };

} // namespace mvs

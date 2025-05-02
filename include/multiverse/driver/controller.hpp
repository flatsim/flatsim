#include "concord/types_basic.hpp"
#include "muli/world.h"

#include <cmath> // for DegToRad conversion
#include <memory>

namespace mvs {
    namespace vehicle {
        using namespace muli;

        // --- Utility functions ---
        inline float DegToRad(float deg) { return deg * static_cast<float>(M_PI) / 180.0f; }

        // --- Configurable parameters ---
        static float EPSILON = 0.01f;
        static float LINEAR_DAMPING = 0.2f;
        static float ANGULAR_DAMPING_COEFF = 2.0f;
        static float DRIVE_FORCE = 30.0f;
        static float SIDE_FRICTION = 0.4f;
        static float MAX_SIDE_IMPULSE = 0.5f;
        static float BRAKE_FORCE = 10.0f;
        static float DRAG_COEFF = 0.5f;
        static float STEERING_TORQUE = 10.0f;
        static float MAX_STEER_ANGLE = DegToRad(35.0f);

        // --- Setter functions ---
        inline void SetEpsilon(float e) { EPSILON = e; }
        inline void SetLinearDamping(float d) { LINEAR_DAMPING = d; }
        inline void SetAngularDampingCoeff(float d) { ANGULAR_DAMPING_COEFF = d; }
        inline void SetDriveForce(float f) { DRIVE_FORCE = f; }
        inline void SetSideFriction(float f) { SIDE_FRICTION = f; }
        inline void SetMaxSideImpulse(float i) { MAX_SIDE_IMPULSE = i; }
        inline void SetBrakeForce(float f) { BRAKE_FORCE = f; }
        inline void SetDragCoeff(float d) { DRAG_COEFF = d; }
        inline void SetSteeringTorque(float t) { STEERING_TORQUE = t; }
        inline void SetMaxSteerAngle(float radians) { MAX_STEER_ANGLE = radians; }

        struct Wheel {
            RigidBody *body;
            Vec2 forward, side;
            float friction, maxImpulse;
            float drag;

            void init(std::shared_ptr<muli::World> world, float scale, const Transform &tf,
                      const CollisionFilter &filter) {
                body = world->CreateCapsule(scale, scale, false, tf);
                body->SetCollisionFilter(filter);
                body->SetLinearDamping(LINEAR_DAMPING);
                body->SetAngularDamping(ANGULAR_DAMPING_COEFF);

                friction = SIDE_FRICTION;
                maxImpulse = MAX_SIDE_IMPULSE;
                drag = DRAG_COEFF;
            }

            void step(float dt) {
                forward = Mul(body->GetRotation(), Vec2(0, 1));
                side = Mul(body->GetRotation(), Vec2(1, 0));

                Vec2 vel = body->GetLinearVelocity();
                float vF = Dot(vel, forward);
                float vS = Dot(vel, side);

                // 1) lateral friction impulse
                if (Abs(vS) > EPSILON) {
                    Vec2 j = -body->GetMass() * friction * vS * dt * side;
                    float jLen = Length(j);
                    float maxJ = maxImpulse * dt;
                    if (jLen > maxJ)
                        j = Normalize(j) * maxJ;
                    body->ApplyLinearImpulse(body->GetPosition(), j, true);
                }

                // 2) angular damping impulse
                float av = body->GetAngularVelocity();
                if (Abs(av) > EPSILON) {
                    float angImp = -av * ANGULAR_DAMPING_COEFF * body->GetInertia() * dt;
                    body->ApplyAngularImpulse(angImp, true);
                }

                // 3) rolling drag force
                if (Abs(vF) > EPSILON) {
                    float dragForce = -drag * vF * dt;
                    body->ApplyForce(body->GetPosition(), forward * dragForce, true);
                }
            }
        };

        class Vehicle {
          public:
            Vehicle(std::shared_ptr<muli::World> w, const concord::Pose &pose) : world(w) {
                // chassis
                CollisionFilter filter;
                filter.bit = 1 << 1;
                filter.mask = ~(1 << 1);

                chassis = world->CreateBox(0.8f, 1.4f);
                chassis->SetCollisionFilter(filter);
                chassis->SetLinearDamping(LINEAR_DAMPING);
                chassis->SetAngularDamping(ANGULAR_DAMPING_COEFF);

                Vec2 chassis_pose;
                chassis_pose.x = pose.point.enu.x;
                chassis_pose.y = pose.point.enu.y;
                chassis->SetPosition(chassis_pose);

                // wheel positions relative to chassis
                float scale = 0.2f;
                Transform pos[4] = {Transform(Vec2(0.4f, 0.7f)), Transform(Vec2(-0.4f, 0.7f)),
                                    Transform(Vec2(0.4f, -0.7f)), Transform(Vec2(-0.4f, -0.7f))};

                for (int i = 0; i < 4; ++i)
                    wheels[i].init(world, scale, pos[i], filter);

                // steering/drive joints
                float m = chassis->GetMass();
                for (int i = 0; i < 4; ++i) {
                    joints[i] = world->CreateMotorJoint(chassis, wheels[i].body, wheels[i].body->GetPosition(), -1.0f,
                                                        STEERING_TORQUE, -1.0f, 0.1f, m);
                }
            }

            void tick(float dt) {
                for (int i = 0; i < 4; ++i)
                    wheels[i].step(dt);
            }

            void update(float steering, float throttle) {
                float angle = steering * MAX_STEER_ANGLE;
                // front wheels steer
                joints[0]->SetAngularOffset(angle);
                joints[1]->SetAngularOffset(angle);
                // rear fixed
                joints[2]->SetAngularOffset(0.0f);
                joints[3]->SetAngularOffset(0.0f);

                // drive on rear wheels
                for (int i = 2; i < 4; ++i) {
                    Vec2 f = wheels[i].forward * (throttle * DRIVE_FORCE);
                    wheels[i].body->ApplyForce(wheels[i].body->GetPosition(), f, true);
                }
            }

          private:
            std::shared_ptr<muli::World> world;
            RigidBody *chassis;
            Wheel wheels[4];
            MotorJoint *joints[4];
        };
    } // namespace vehicle

} // namespace mvs

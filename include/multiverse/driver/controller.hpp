#include "concord/types_basic.hpp"
#include "muli/world.h"

#include <cmath> // for DegToRad conversion
#include <math.h>
#include <memory>

namespace mvs {
    namespace vehicle {
        using namespace muli;

        // --- Utility functions ---
        // inline float DegToRad(float deg) { return deg * static_cast<float>(M_PI) / 180.0f; }

        static bool followCam = true;
        static bool rotateCam = false;
        static bool drawAxis = false;
        static float linearDamping = 0.2f;
        static float angularDamping = 2.0f;
        static float force = 30;
        static float torque = 10;
        static float friction = 0.4;
        static float maxImpulse = 0.5;
        static float brake = 10;
        static float drag = 0.5f;

        struct Wheel {
            RigidBody *wheel;
            Vec2 forward, normal;

            float force, torque;
            float brake, drag;

            float friction, maxImpulse;

            void init(World *world, float scale, Transform tf, CollisionFilter filter, float linearDamping,
                      float angularDamping, float _force, float _friction, float _maxImpulse, float _brake,
                      float _drag) {
                wheel = world->CreateCapsule(scale, scale, false, tf);
                wheel->SetCollisionFilter(filter);

                wheel->SetLinearDamping(linearDamping);
                wheel->SetAngularDamping(angularDamping);

                force = _force;
                friction = _friction;
                maxImpulse = _maxImpulse;

                brake = _brake;
                drag = _drag;
            }

            void step(float dt) {
                const Vec2 up(0, 1);
                const Vec2 right(1, 0);

                forward = Mul(wheel->GetRotation(), up);
                normal = Mul(wheel->GetRotation(), right);

                Vec2 v = wheel->GetLinearVelocity();
                float vf = Dot(v, forward);
                float vn = Dot(v, normal);

                if (Abs(vn) > epsilon) {
                    Vec2 j = -wheel->GetMass() * friction * vn * normal;
                    if (Length(j) > maxImpulse) {
                        j = Normalize(j) * maxImpulse;
                    }
                    wheel->ApplyLinearImpulse(wheel->GetPosition(), j, true);
                }

                float av = wheel->GetAngularVelocity();
                if (Abs(av) > epsilon) {
                    wheel->ApplyAngularImpulse(0.1f * wheel->GetInertia() * -av, true);
                }

                if (Abs(vf) > epsilon) {
                    float dragForceMagnitude = -drag * vf;
                    wheel->ApplyForce(wheel->GetPosition(), dragForceMagnitude * forward, true);
                }
            }
        };

        class Vehicle {
          public:
            Vehicle(muli::World *world, const concord::Pose &pose) : world(world) {
                CollisionFilter filter;
                filter.bit = 1 << 1;
                filter.mask = ~(1 << 1);

                float w = 0.8f;
                float h = 1.4f;

                body = world->CreateBox(w, h);
                body->SetCollisionFilter(filter);

                Vec2 p;
                p.x = pose.point.enu.x;
                p.y = pose.point.enu.y;
                body->SetPosition(p);

                body->SetLinearDamping(linearDamping);
                body->SetAngularDamping(angularDamping);
                // chassis
                float s = 0.2f;

                // Front wheels
                Transform tf_fr(Vec2(p.x + w / 2, p.y + h / 2));
                wheels[0].init(world, s, tf_fr, filter, linearDamping, angularDamping, force, friction, maxImpulse,
                               brake, drag);
                Transform tf_fl(Vec2(p.x - w / 2, p.y + h / 2));
                wheels[1].init(world, s, tf_fl, filter, linearDamping, angularDamping, force, friction, maxImpulse,
                               brake, drag);

                // Rear wheels
                Transform tf_rr(Vec2(p.x + w / 2, p.y - h / 2));
                wheels[2].init(world, s, tf_rr, filter, linearDamping, angularDamping, force, friction, maxImpulse,
                               brake, drag);
                Transform tf_rl(Vec2(p.x - w / 2, p.y - h / 2));
                wheels[3].init(world, s, tf_rl, filter, linearDamping, angularDamping, force, friction, maxImpulse,
                               brake, drag);

                for (int i = 0; i < 4; ++i) {
                    std::cout << "wheel raw pointer: " << wheels[i].wheel << std::endl;
                }

                float mf = -1;
                float fr = -1;
                float dr = 0.1f;
                float jm = body->GetMass();

                for (int32 i = 0; i < 4; ++i) {
                    joints[i] = world->CreateMotorJoint(body, wheels[i].wheel, wheels[i].wheel->GetPosition(), mf,
                                                        torque, fr, dr, jm);
                }
            }

            void tick(float dt) {
                for (int i = 0; i < 4; ++i) {
                    wheels[i].step(dt);
                }
            }

            // void update(float steering, float throttle) {
            //     float angle = steering * MAX_STEER_ANGLE;
            //     // front wheels steer
            //     joints[0]->SetAngularOffset(angle);
            //     joints[1]->SetAngularOffset(angle);
            //     // rear fixed
            //     joints[2]->SetAngularOffset(0.0f);
            //     joints[3]->SetAngularOffset(0.0f);
            //
            //     // drive on rear wheels
            //     for (int i = 2; i < 4; ++i) {
            //         Vec2 f = wheels[i].forward * (throttle * DRIVE_FORCE);
            //         // wheels[i].body->ApplyForce(wheels[i].body->GetPosition(), f, true);
            //     }
            // }

          private:
            muli::World *world;
            RigidBody *body;
            Wheel wheels[4];
            MotorJoint *joints[4];
        };
    } // namespace vehicle

} // namespace mvs

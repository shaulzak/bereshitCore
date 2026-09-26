//
// Created by yaly on 23/07/2026.
//

#ifndef BERESHITCORE_HINGEJOINT_H
#define BERESHITCORE_HINGEJOINT_H

#include "Joint.h"
#include "Vector2.h"
class GameObject;

class HingeJoint : public Joint {
    public:
        Vector3 GetWorldAxis();

        HingeJoint(GameObject* bodyB, Vector3 axis, Vector3* anchor = nullptr, double beta = 0.2);
        HingeJoint *Copy() const override;
        void PhysicsUpdate(double dt) override;
        void ResetToDefault() override;

        // Motor: drives the relative angular velocity (B - A) along the world axis toward motorSpeed
        // (rad/s), with at most maxMotorTorque (N*m). Solved together with the joint constraints.
        bool motorEnabled = false;
        double motorSpeed = 0.0;
        double maxMotorTorque = 0.0;
        // Motor impulse (N*m*s) applied in the last solved tick; divide by the tick for the torque.
        [[nodiscard]] double GetMotorImpulse() const { return motorImpulse; }
    private:
        double motorImpulse = 0.0;  // accumulated this tick, clamped to maxMotorTorque * dt
        Vector3 axisLocal;
        static Vector3 perp(Vector3&);
        void SolveLinear(double dt) override;
        void SolveAngular(double dt) override;


};


#endif //BERESHITCORE_HINGEJOINT_H

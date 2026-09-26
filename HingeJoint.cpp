//
// Created by yaly on 23/07/2026.
//

#include "HingeJoint.h"

#include <algorithm>
#include <cmath>

#include "Rigidbody.h"
#include "World.h"

Vector3 HingeJoint::GetWorldAxis() {
    // In A's frame, like the constraint and the motor: with B's, a body turned relative to the other at the
    // start reported an axis the joint doesn't turn about.
    return transformA->quaternion.RotateConjugated(axisLocal).normalized();
}

HingeJoint::HingeJoint(GameObject* bodyB, Vector3 axis, Vector3* anchor, double beta) : Joint(bodyB, anchor, beta) {
    axisLocal = axis.normalized();
}

HingeJoint * HingeJoint::Copy() const {
    HingeJoint* hinge_joint = new HingeJoint(bodyB,axisLocal, nullptr, beta);
    hinge_joint->CastAnchor(worldAnchor);
    hinge_joint->motorEnabled = motorEnabled;
    hinge_joint->motorSpeed = motorSpeed;
    hinge_joint->maxMotorTorque = maxMotorTorque;
    return hinge_joint;
}

Vector3 HingeJoint::perp(Vector3 & axis) {
        Vector3 helper;
       if (std::abs(axis.x) < 0.9) {
           helper = Vector3(1.0, 0.0, 0.0);
       }else {
           helper = Vector3(0.0, 1.0, 0.0);

       }
        return axis.cross(helper).normalized();
}




void HingeJoint::SolveLinear(double dt) {
    auto IinvA = rbA->GetInvertWorld();
    auto IinvB = rbB->GetInvertWorld();
    double invertMassA = rbA->GetInvMass();
    double invertMassB = rbB->GetInvMass();
    Vector3 rA = transformA->quaternion.RotateConjugated(localAnchorA);
    Vector3 rB = transformB->quaternion.RotateConjugated(localAnchorB);

    Vector3 vA = rbA->velocity + rbA->angularVelocity.cross(-rA);
    Vector3 vB = rbB->velocity + rbB->angularVelocity.cross(-rB);

    Vector3 dv = vB - vA;

    Vector3 worldAnchorA = transformA->position + rA;
    Vector3 worldAnchorB = transformB->position + rB;

    Vector3 positionError = worldAnchorB - worldAnchorA;

    Vector3 bias = positionError * (beta / dt);

    // Effective mass matrix  K = (1/mA + 1/mB)*I + [rA]x * IinvA * [rA]x^T + [rB]x * IinvB * [rB]x^T

    double invertMass = invertMassA + invertMassB;

    BuildEffectiveMassMatrix(invertMass, rA, rB, *IinvA, *IinvB); // finds K

    Vector3 impulse = -Solve3x3(dv + bias);

    Rigidbody::ApplyImpulsePair(*rbA, *rbB, impulse, rA, rB);
}

void HingeJoint::SolveAngular(double dt) {
    auto IinvA = rbA->GetInvertWorld();
    auto IinvB = rbB->GetInvertWorld();

    Vector3 axis_world = transformA->quaternion.RotateConjugated(axisLocal).normalized();

    Vector3 t1 = perp(axis_world);
    Vector3 t2 = axis_world.cross(t1).normalized();

    Vector3 rel_w = rbB->angularVelocity - rbA->angularVelocity;

    // Construct the 2-row Jacobian: J = [t1^T; t2^T]
    // Effective mass:  K_ang = J * (IinvA + IinvB) * J^T   (2x2)

    AddMatrix(*IinvA, *IinvB); // result is in K

    Vector3 Kt1 = t1.MatrixMultiplication(K);
    Vector3 Kt2 = t2.MatrixMultiplication(K);

    // J @ K_full @ J.T
    Vector2 K_ang[2];
    K_ang[0] = Vector2(t1.dot(Kt1), t1.dot(Kt2));
    K_ang[1] = Vector2(t2.dot(Kt1), t2.dot(Kt2));


    Vector2 vel_error(t1.dot(rel_w), t2.dot(rel_w));

    Quaternion q_rel = transformA->quaternion.Inverse() * transformB->quaternion;

    // self.clamp_rotation(q_rel, IinvA, IinvB, a, b)

    Quaternion q_error = q_rel * initialRelativeRotation.Inverse();

    Vector3 err_vec(q_error.x, q_error.y, q_error.z);
    if (q_error.w < 0) {
        err_vec = err_vec * -1;
    }
    Vector3 ang_error = err_vec * 2.0;

    Vector2 bias(t1.dot(ang_error) * (beta / dt), t2.dot(ang_error) * (beta / dt));

    Vector2 ang_impulse2d = -Solve2x2(vel_error + bias, K_ang);

    Vector3 ang_impulse = t1 * ang_impulse2d.x + t2 * ang_impulse2d.y;

    if (!rbA->IsKinematic()) {
        rbA->angularVelocity -= ang_impulse.MatrixMultiplication(*IinvA);

    }
    if (!rbB->IsKinematic()) {
        rbB->angularVelocity += ang_impulse.MatrixMultiplication(*IinvB);

    }

    if (motorEnabled) {
        double k = axis_world.dot(axis_world.MatrixMultiplication(K));  // K = IinvA + IinvB
        if (k > 0) {
            Vector3 motor_rel_w = rbB->angularVelocity - rbA->angularVelocity;
            double lambda = -(axis_world.dot(motor_rel_w) - motorSpeed) / k;
            if (!std::isfinite(lambda)) {
                lambda = 0.0;
            }
            double maxImpulse = std::abs(maxMotorTorque) * dt;   // a negative limit clamped to a constant push
            double previous = motorImpulse;
            motorImpulse = std::max(std::min(previous + lambda, maxImpulse), -maxImpulse);
            Vector3 motor_impulse = axis_world * (motorImpulse - previous);
            if (!rbA->IsKinematic()) {
                rbA->angularVelocity -= motor_impulse.MatrixMultiplication(*IinvA);
            }
            if (!rbB->IsKinematic()) {
                rbB->angularVelocity += motor_impulse.MatrixMultiplication(*IinvB);
            }
        }
    }
}

void HingeJoint::ResetToDefault() {
    motorImpulse = 0.0;
}

void HingeJoint::PhysicsUpdate(double dt) {
    // Called once per tick before the solver iterations. Warm start: begin from last tick's motor
    // impulse, so a steady holding torque (e.g. against gravity) isn't rebuilt from zero every tick.
    // Light bodies between two joints (ankle / hip blocks) make that rebuild converge very slowly.
    // A NaN impulse (after the simulation blew up) would otherwise survive every clamp and reset.
    if (!motorEnabled || !std::isfinite(motorImpulse)) {
        motorImpulse = 0.0;
        if (!motorEnabled) {
            return;
        }
    }
    double maxImpulse = std::abs(maxMotorTorque) * dt;
    motorImpulse = std::max(std::min(motorImpulse, maxImpulse), -maxImpulse);
    if (motorImpulse != 0.0) {
        Vector3 axis_world = transformA->quaternion.RotateConjugated(axisLocal).normalized();
        Vector3 impulse = axis_world * motorImpulse;
        if (!rbA->IsKinematic()) {
            rbA->angularVelocity -= impulse.MatrixMultiplication(*rbA->GetInvertWorld());
        }
        if (!rbB->IsKinematic()) {
            rbB->angularVelocity += impulse.MatrixMultiplication(*rbB->GetInvertWorld());
        }
    }
}

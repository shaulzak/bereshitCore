//
// Created by yaly on 29/06/2026.
//

#include "Rigidbody.h"
#include <cmath>
#include "GameObject.h"
#include "World.h"
#include "Vector3.h"


double Rigidbody::GetFrictionCoefficient(const Rigidbody& rb1, const Rigidbody& rb2) {
    return std::min(rb1.GetFrictionCoefficient(), rb2.GetFrictionCoefficient());
}

void Rigidbody::UpdateInertiaWorld() {
    if (isKinematic) {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                InvertWorld[i][j] = 0.0;
        return;
    }

    auto R = transform->quaternion.ToMatrix3(&GetParent()->cache);

    // temp = R * inverse_inertia
    std::array<std::array<double, 3>, 3> temp{};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            temp[i][j] = 0.0;
            for (int k = 0; k < 3; k++) {
                temp[i][j] += R[i][k] * invertInertiaMetrix[k][j];
            }
        }
    }

    // Iinv_world = temp * R^T
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            InvertWorld[i][j] = 0.0;
            for (int k = 0; k < 3; k++) {
                InvertWorld[i][j] += temp[i][k] * R[j][k]; // R^T[k][j] = R[j][k]
            }
        }
    }
}

void Rigidbody::PositionalCorrection(const Rigidbody &rb1, const Rigidbody &rb2, double penetration, const Vector3 &normal, double inv_eff_mass) {
    float percent = 0.02;
    float slop = 0.005;

    double correction_mag = std::max(penetration - slop, 0.0) / inv_eff_mass * percent;
    Vector3 correction = normal * correction_mag;

    if (!rb1.isKinematic) {
        rb1.transform->position -= correction * rb1.invMass;
    }

    if (!rb2.isKinematic) {
        rb2.transform->position += correction * rb2.invMass;
    }
}


#include <algorithm>
void Rigidbody::ApplyFrictionImpulse(Rigidbody& rb1, Rigidbody& rb2, const Vector3& relativeVelocity, const Vector3& normal,
    double J,const Vector3& r1, const Vector3& r2) {

    Vector3 tangent = relativeVelocity - normal * relativeVelocity.dot(normal);
    double tangentLength = tangent.magnitude();

    if (tangentLength < 1e-6) {
        return;
    }

    tangent = tangent.normalized();
    double mu = GetFrictionCoefficient(rb1, rb2);

    double Jt_magnitude = -relativeVelocity.dot(tangent);

    double denom = 0.0;

    if (!rb1.isKinematic) {
        denom += rb1.invMass;

        Vector3 r1xt = r1.cross(tangent);
        Vector3 ang1 = r1xt.MatrixMultiplication(*rb1.GetInvertWorld());
        denom += (ang1.cross(r1)).dot(tangent);
    }
    if (!rb2.isKinematic) {
        denom += rb2.invMass;

        Vector3 r2xt = r2.cross(tangent);
        Vector3 ang2 = r2xt.MatrixMultiplication(*rb2.GetInvertWorld());
        denom += (ang2.cross(r2)).dot(tangent);
    }


    if (denom == 0.0) {
        return;
    }

    Jt_magnitude /= denom;
    double max_friction = mu * J;
    Jt_magnitude = std::max(-max_friction, std::min(Jt_magnitude, max_friction));

    ApplyImpulsePair(rb1, rb2, tangent * Jt_magnitude, r1, r2);
}

std::optional<std::tuple<double, Vector3, Vector3, Vector3, double>> Rigidbody::FindImpulse(Rigidbody &rb1, Rigidbody &rb2, const Vector3 &contact_point, const Vector3 &normal, double dt) {
    rb1.ForceIntegrate(dt);
    rb2.ForceIntegrate(dt);

    Vector3 r1 = contact_point - rb1.transform->position;
    Vector3 r2 = contact_point - rb2.transform->position;

    Vector3 v1_at_p = rb1.velocity - rb1.angularVelocity.cross(r1);
    Vector3 v2_at_p = rb2.velocity - rb2.angularVelocity.cross(r2);
    Vector3 relative_vel = v2_at_p - v1_at_p;
    double v_norm = relative_vel.dot(normal);

    if (v_norm >= 0) {
        return std::nullopt;
    }

    Vector3 rn1 = r1.cross(normal);
    Vector3 rn2 = r2.cross(normal);
    Vector3 term1(0, 0, 0);
    Vector3 term2(0, 0, 0);
    if (!rb1.isKinematic) {
        term1 = (rn1.MatrixMultiplication(*rb1.GetInvertWorld())).cross(r1);
    }
    if  (!rb2.isKinematic) {
        term2 = (rn2.MatrixMultiplication(*rb2.GetInvertWorld())).cross(r2);
    }

    double restitution = FindRestitution(rb1, rb2, v_norm);
    double kLinear =  (rb1.isKinematic ? 0.0 : rb1.invMass) + (rb2.isKinematic ? 0.0 : rb2.invMass);
    double kAngular = normal.dot(term1 + term2);
    double inverseMass = kLinear + kAngular;

    double J = -(1 + restitution) * v_norm / inverseMass;

    return std::make_tuple(J, r1, r2, relative_vel, inverseMass);

}


Rigidbody* Rigidbody::Copy() const {
    Rigidbody* rigidbody = new Rigidbody(mass, isKinematic, velocity, angularVelocity,
        useGravity, frictionCoefficient, restitution, freezeRotation);

    return rigidbody;
}

void Rigidbody::ApplyGravity(const Vector3 &gravity) {
    if (!isKinematic && useGravity) {
        force += gravity * mass;
    }
}

void Rigidbody::SetIsKinematic(bool state) {
    isKinematic = state;
    invMass = isKinematic ? 0.0 : 1 / mass;
    UpdateInertiaWorld();
}

void Rigidbody::SetUseGravity(bool state) {
    useGravity = state;
}


Rigidbody::Rigidbody(float mass, bool isKinematic, Vector3 initialVelocity, Vector3 initialAngularVelocity, bool useGravity,
                     float frictionCoefficient, float restitution, Vector3 freezeRotation) : mass(mass), isKinematic(isKinematic),
                                                                                             velocity(initialVelocity), angularVelocity(initialAngularVelocity), useGravity(useGravity),
                                                                                             frictionCoefficient(frictionCoefficient), restitution(restitution), freezeRotation(freezeRotation){

    SetName("Rigidbody");
    invMass = isKinematic ? 0.0 : 1 / mass;

}

void Rigidbody::PhysicsUpdate(double dt) {

}

void Rigidbody::PhysicsUpdateFirstIteration(double dt) {
    if (!isKinematic) {
        // apply_gravity(this->GetParent()->GetWorld()->gravity);
        integrate(dt);
    }
}


void Rigidbody::integrate(double dt) {
    acceleration = force * invMass;

    Vector3 pos = velocity * dt + acceleration * 0.5 * dt * dt;
    if (pos.magnitude() > 0) {
        cache->aabbDirty = true;
    }

    transform->position += pos;

    if (!freezeRotation.x) {
        angularAcceleration.x = torque.x / inertia.x;
    }if (!freezeRotation.y) {
        angularAcceleration.y = torque.y / inertia.y;
    }if (!freezeRotation.z) {
        angularAcceleration.z = torque.z / inertia.z;
    }

    Vector3 angDisp = angularVelocity * dt + angularAcceleration * dt * dt * 0.5;

    angularVelocity += angularAcceleration * dt;

    transform->quaternion *= Quaternion::EulerRadians(angDisp);
    if (angDisp.magnitude() > 0) {
        UpdateInertiaWorld();
        transform->rotation = transform->quaternion.ToEuler();
        cache->rotationDirty = true;
        cache->rotationDirtyAbs = true;
        cache->aabbDirty = true;
        up = transform->quaternion.Rotate(Vector3(0, 1, 0));
        forward = transform->quaternion.Rotate(Vector3(0, 0, 1));

    }


    velocity += acceleration * dt;
    force.Zero();
    torque.Zero();

}

void Rigidbody::ForceIntegrate(double dt) {
    if (!isKinematic) {
        velocity += (force / mass) * dt;
        force.Zero();
    }
}

void Rigidbody::SolveImpulse(Rigidbody &rb1, Rigidbody &rb2, const Vector3& contact_point, const Vector3& normal, double penetration, double dt) {
        auto result = FindImpulse(rb1, rb2, contact_point, normal, dt);
        if (!result) {
            return;
        }

        auto& [J, r1, r2, relative_vel, inverseMass] = *result;

        PositionalCorrection(rb1, rb2, penetration, normal, inverseMass);

        ApplyImpulsePair(rb1, rb2, normal * J, r1, r2);
        // Updated velocities at the contact point.
        const Vector3 v1 =
            rb1.velocity - rb1.angularVelocity.cross(r1);

        const Vector3 v2 =
            rb2.velocity - rb2.angularVelocity.cross(r2);

        // Use the same relative-velocity convention as FindImpulse().
        const Vector3 updated_relative_vel = v2 - v1;
        ApplyFrictionImpulse(rb1, rb2, updated_relative_vel, normal, J, r1, r2);
}

double Rigidbody::FindRestitution(const Rigidbody &rb1, const Rigidbody &rb2, double normalVelocity) {
    // if (normalVelocity > -0.2){
    //     return 0.0;
    // }
    return std::min(rb1.GetRestitution(), rb2.GetRestitution());
}

void Rigidbody::ApplyImpulsePair(Rigidbody& rb1, Rigidbody &rb2, const Vector3 &impulseVec,
    const Vector3 &r1, const Vector3 &r2) {
    Vector3 negative_impulse = -impulseVec;

    if (!rb1.isKinematic) {
        rb1.velocity += negative_impulse * rb1.invMass;
        rb1.ApplyTorqueImpulse(impulseVec, r1);
    }


    if (!rb2.isKinematic) {
        rb2.velocity += impulseVec * rb2.invMass;
        rb2.ApplyTorqueImpulse(negative_impulse, r2);
    }


}

void Rigidbody::ApplyTorqueImpulse(Vector3 impulse, Vector3 r) {
    Vector3 torqueImpulse = r.cross(impulse);
    Vector3 localTorqueImpulse = transform->quaternion.RotateConjugated(torqueImpulse);
    Vector3 local_delta_w = localTorqueImpulse * invertInertia;
    Vector3 ang_impulse = transform->quaternion.Rotate(local_delta_w);

    if (!freezeRotation.x) {
        angularVelocity.x += ang_impulse.x;
    }
    if (!freezeRotation.y) {
        angularVelocity.y += ang_impulse.y;
    }
    if (!freezeRotation.z) {
        angularVelocity.z += ang_impulse.z;
    }
}

void Rigidbody::ApplyAngularImpulse(const Vector3& angularImpulse) {
    Vector3 localImpulse =
        transform->quaternion.RotateConjugated(angularImpulse);

    Vector3 localDeltaW = localImpulse * invertInertia;

    Vector3 deltaW =
        transform->quaternion.Rotate(localDeltaW);

    if (!freezeRotation.x)
        angularVelocity.x += deltaW.x;

    if (!freezeRotation.y)
        angularVelocity.y += deltaW.y;

    if (!freezeRotation.z)
        angularVelocity.z += deltaW.z;
}

void Rigidbody::attach(GameObject& obj) {
    Component::attach(obj);
    transform = &obj.transform;
    cache = &obj.cache;
    double hx = transform->scale.x;
    double hy = transform->scale.y;
    double hz = transform->scale.z;

    inertia = Vector3(
                (1 / 12.0) * mass * (std::pow(hy, 2) + std::pow(hz, 2)),
                (1 / 12.0) * mass * (std::pow(hx, 2) + std::pow(hz, 2)),
                (1 / 12.0) * mass * (std::pow(hy, 2) + std::pow(hx, 2))
            );
    invertInertia = inertia.Inverse();
    invertInertiaMetrix[0][0] = invertInertia.x;
    invertInertiaMetrix[1][1] = invertInertia.y;
    invertInertiaMetrix[2][2] = invertInertia.z;
    UpdateInertiaWorld();
}

void Rigidbody::SolveFrictionImpulse(Rigidbody &rb1, Rigidbody &rb2, const Vector3 &contact_point,
    const Vector3 &normal, double dt) {
    auto result = FindImpulse(rb1, rb2, contact_point, normal, dt);
    if (!result) {
        return;
    }

    auto& [J, r1, r2, relative_vel, inverseMass] = *result;

    ApplyFrictionImpulse(rb1, rb2, relative_vel, normal, J, r1, r2);

}

void Rigidbody::ResetToDefault() {
    acceleration.Zero();
    velocity.Zero();
    angularVelocity.Zero();
    angularAcceleration.Zero();
    cache->SetDirty();
}







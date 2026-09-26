//
// Created by yaly on 29/06/2026.
//

#include "Rigidbody.h"
#include <cmath>
#include <unordered_map>
#include "Contact.h"
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

    // R maps world -> local (R^T local -> world), so Iinv_world = R^T * Iinv_local * R.
    // It was R * Iinv_local * R^T: a rotated body's inertia turned the other way - a bar tilted 30 deg
    // pushed about its long axis spun 57 deg off it, and the same motion differed with the heading.
    // temp = R^T * inverse_inertia
    std::array<std::array<double, 3>, 3> temp{};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            temp[i][j] = 0.0;
            for (int k = 0; k < 3; k++) {
                temp[i][j] += R[k][i] * invertInertiaMetrix[k][j];
            }
        }
    }

    // Iinv_world = temp * R
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            InvertWorld[i][j] = 0.0;
            for (int k = 0; k < 3; k++) {
                InvertWorld[i][j] += temp[i][k] * R[k][j];
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
    if (customInertia) {
        rigidbody->SetInertia(inertia);
    }
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

    // the exact rotation about angDisp (axis-angle); Euler angles of it came out different with the heading
    if (angDisp.magnitude() > 0.0) {
        transform->quaternion *= Quaternion::AxisAngle(angDisp, angDisp.magnitude());
    }
    if (angDisp.magnitude() > 0) {
        transform->rotation = transform->quaternion.ToEuler();
        cache->rotationDirty = true;
        cache->rotationDirtyAbs = true;
        cache->aabbDirty = true;
        // After marking the rotation dirty, so the world inertia uses this tick's rotation, not the last one.
        UpdateInertiaWorld();
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

double Rigidbody::EffectiveInverseMass(Rigidbody &rb1, Rigidbody &rb2, const Vector3 &r1, const Vector3 &r2,
                                       const Vector3 &direction) {
    // Same form as FindImpulse's normal term: 1/m + d . ((r x d) Iinv) x r, for each non-kinematic body.
    double k = 0.0;
    if (!rb1.isKinematic) {
        k += rb1.invMass + direction.dot((r1.cross(direction).MatrixMultiplication(*rb1.GetInvertWorld())).cross(r1));
    }
    if (!rb2.isKinematic) {
        k += rb2.invMass + direction.dot((r2.cross(direction).MatrixMultiplication(*rb2.GetInvertWorld())).cross(r2));
    }
    return k;
}

void Rigidbody::SolveContact(Contact &c, double dt) {
    // Standard sequential impulses (Box2D style). The old solver recomputed each iteration from scratch:
    // friction only acted on the iteration where the contact was approaching and was capped by that
    // iteration's normal impulse alone, so it came out ~30% weak (feet slid) and restitution was
    // re-applied every iteration. Here impulses accumulate per contact over the tick's iterations.
    constexpr double kRestitutionThreshold = 0.5;  // m/s: slower impacts don't bounce
    constexpr double kBaumgarte = 0.2;              // fraction of the penetration recovered per tick
    constexpr double kSlop = 0.002;                 // m of penetration allowed without correction

    Rigidbody &rb1 = c.rb1;
    Rigidbody &rb2 = c.rb2;
    rb1.ForceIntegrate(dt);
    rb2.ForceIntegrate(dt);

    const Vector3 r1 = c.contact_point - rb1.transform->position;
    const Vector3 r2 = c.contact_point - rb2.transform->position;
    auto relativeVelocity = [&]() {
        return (rb2.velocity - rb2.angularVelocity.cross(r2)) - (rb1.velocity - rb1.angularVelocity.cross(r1));
    };
    const Vector3 &n = c.normal;

    if (!c.prepared) {
        c.prepared = true;
        const double vn0 = relativeVelocity().dot(n);
        const double restitution = FindRestitution(rb1, rb2, vn0);
        const double bounce = vn0 < -kRestitutionThreshold ? -restitution * vn0 : 0.0;
        const double recovery = kBaumgarte / dt * std::max(c.penetration - kSlop, 0.0);
        c.velocityBias = std::max(bounce, recovery);
        const Vector3 helper = std::abs(n.x) < 0.9 ? Vector3(1, 0, 0) : Vector3(0, 1, 0);
        c.tangent1 = n.cross(helper).normalized();
        c.tangent2 = n.cross(c.tangent1).normalized();
        c.normalMass = EffectiveInverseMass(rb1, rb2, r1, r2, n);
        c.tangentMass1 = EffectiveInverseMass(rb1, rb2, r1, r2, c.tangent1);
        c.tangentMass2 = EffectiveInverseMass(rb1, rb2, r1, r2, c.tangent2);
    }

    if (c.normalMass > 0.0) {
        const double vn = relativeVelocity().dot(n);
        double delta = -(vn - c.velocityBias) / c.normalMass;
        const double previous = c.normalImpulse;
        c.normalImpulse = std::max(previous + delta, 0.0);
        delta = c.normalImpulse - previous;
        if (delta != 0.0) {
            ApplyImpulsePair(rb1, rb2, n * delta, r1, r2);
        }
    }

    const double maxFriction = GetFrictionCoefficient(rb1, rb2) * c.normalImpulse;
    auto solveFriction = [&](const Vector3 &tangent, double mass, double &accumulated) {
        if (mass <= 0.0) {
            return;
        }
        double delta = -relativeVelocity().dot(tangent) / mass;
        const double previous = accumulated;
        accumulated = std::max(-maxFriction, std::min(previous + delta, maxFriction));
        delta = accumulated - previous;
        if (delta != 0.0) {
            ApplyImpulsePair(rb1, rb2, tangent * delta, r1, r2);
        }
    };
    solveFriction(c.tangent1, c.tangentMass1, c.tangentImpulse1);
    solveFriction(c.tangent2, c.tangentMass2, c.tangentImpulse2);
}

void Rigidbody::SolveContactsJacobi(std::vector<Contact> &contacts, double dt, int passes) {
    // Order-independent (parallel, Jacobi) solving: in each pass every contact's impulse is found from the same
    // velocities, then all are applied together, each shared among the contacts between the same two bodies (a
    // face touching with 4 corners must not push 4 times). Solving them one after another let the first-solved
    // corner of a landing foot take the whole impact, and which corner came first was fixed in world
    // coordinates - so the same motion on the left and on the right foot came out different.
    constexpr double kRestitutionThreshold = 0.5;
    constexpr double kBaumgarte = 0.2;
    constexpr double kSlop = 0.002;
    constexpr double kMaxRecovery = 0.5;            // m/s: World::CorrectPositions also pushes out, and this
                                                    // velocity stays on the body - a deep overlap must not launch it
    const size_t count = contacts.size();
    auto relativeVelocity = [](const Contact &c) {
        return (c.rb2.velocity - c.rb2.angularVelocity.cross(c.r2)) - (c.rb1.velocity - c.rb1.angularVelocity.cross(c.r1));
    };
    // Contacts per moving body: each contact's impulse is found as if it alone stopped the body, so it is shared
    // among all the contacts pushing that body - not only those with the same partner (a box across two floor
    // tiles got the full push from each tile, overshot, and flipped back every pass).
    thread_local std::unordered_map<const Rigidbody*, int> contactsPerBody;
    if (count > 0 && !contacts[0].prepared) {
        contactsPerBody.clear();
        for (const Contact &c : contacts) {
            if (!c.rb1.isKinematic) ++contactsPerBody[&c.rb1];
            if (!c.rb2.isKinematic) ++contactsPerBody[&c.rb2];
        }
    }
    for (size_t i = 0; i < count; ++i) {
        Contact &c = contacts[i];
        c.rb1.ForceIntegrate(dt);
        c.rb2.ForceIntegrate(dt);
        if (c.prepared) {
            continue;
        }
        c.prepared = true;
        const int k = std::max(c.rb1.isKinematic ? 1 : contactsPerBody[&c.rb1],
                               c.rb2.isKinematic ? 1 : contactsPerBody[&c.rb2]);
        c.share = 1.0 / k;
        c.r1 = c.contact_point - c.rb1.transform->position;
        c.r2 = c.contact_point - c.rb2.transform->position;
        const Vector3 &n = c.normal;
        const double vn0 = relativeVelocity(c).dot(n);
        const double restitution = FindRestitution(c.rb1, c.rb2, vn0);
        const double bounce = vn0 < -kRestitutionThreshold ? -restitution * vn0 : 0.0;
        const double recovery = std::min(kBaumgarte / dt * std::max(c.penetration - kSlop, 0.0), kMaxRecovery);
        c.velocityBias = std::max(bounce, recovery);
        // tangent1 along the slip, so the friction doesn't depend on how the world axes lie; a fixed one when
        // the contact isn't sliding (then friction is solved exactly with the full 2x2 mass, any basis will do)
        const Vector3 v0 = relativeVelocity(c);
        const Vector3 slip = v0 - n * v0.dot(n);
        if (slip.magnitude() > 1e-3) {
            c.tangent1 = slip.normalized();
        } else {
            const Vector3 helper = std::abs(n.x) < 0.9 ? Vector3(1, 0, 0) : Vector3(0, 1, 0);
            c.tangent1 = n.cross(helper).normalized();
        }
        c.tangent2 = n.cross(c.tangent1).normalized();
        c.normalMass = EffectiveInverseMass(c.rb1, c.rb2, c.r1, c.r2, n);
        c.tangentMass1 = EffectiveInverseMass(c.rb1, c.rb2, c.r1, c.r2, c.tangent1);
        c.tangentMass2 = EffectiveInverseMass(c.rb1, c.rb2, c.r1, c.r2, c.tangent2);
        // the cross term, from the same form along the unit diagonal u = (t1 + t2) / sqrt 2: K(u) = (K11 + K22) / 2 + K12
        c.tangentMass12 = EffectiveInverseMass(c.rb1, c.rb2, c.r1, c.r2, (c.tangent1 + c.tangent2).normalized())
                          - 0.5 * (c.tangentMass1 + c.tangentMass2);
    }
    thread_local std::vector<double> delta;
    delta.assign(count, 0.0);
    for (int pass = 0; pass < passes; ++pass) {
        for (size_t i = 0; i < count; ++i) {       // normal impulses
            Contact &c = contacts[i];
            delta[i] = 0.0;
            if (c.normalMass > 0.0) {
                const double previous = c.normalImpulse;
                const double vn = relativeVelocity(c).dot(c.normal);
                c.normalImpulse = std::max(previous - c.share * (vn - c.velocityBias) / c.normalMass, 0.0);
                delta[i] = c.normalImpulse - previous;
            }
        }
        for (size_t i = 0; i < count; ++i) {
            if (delta[i] != 0.0) {
                ApplyImpulsePair(contacts[i].rb1, contacts[i].rb2, contacts[i].normal * delta[i], contacts[i].r1, contacts[i].r2);
            }
        }
        // Friction, both tangents together: the impulse that stops the sliding is K^-1 v with the full 2x2 tangent
        // mass K, capped by each contact's own load (|impulse| <= mu N). The tangents were always fixed to the
        // world axes and solved and capped each alone (ignoring K12, ~40% more grip diagonally), so the same
        // motion came out different with the heading.
        thread_local std::vector<double> delta2;
        delta2.assign(count, 0.0);
        for (size_t i = 0; i < count; ++i) {
            Contact &c = contacts[i];
            delta[i] = 0.0;
            delta2[i] = 0.0;
            if (c.tangentMass1 <= 0.0 || c.tangentMass2 <= 0.0) {
                continue;
            }
            const Vector3 v = relativeVelocity(c);
            const double maxFriction = GetFrictionCoefficient(c.rb1, c.rb2) * c.normalImpulse;
            const double p1 = c.tangentImpulse1, p2 = c.tangentImpulse2;
            const double v1 = v.dot(c.tangent1), v2 = v.dot(c.tangent2);
            const double det = c.tangentMass1 * c.tangentMass2 - c.tangentMass12 * c.tangentMass12;
            double a1, a2;
            if (det > 1e-12 * c.tangentMass1 * c.tangentMass2) {
                a1 = p1 - c.share * (c.tangentMass2 * v1 - c.tangentMass12 * v2) / det;
                a2 = p2 - c.share * (c.tangentMass1 * v2 - c.tangentMass12 * v1) / det;
            } else {
                a1 = p1 - c.share * v1 / c.tangentMass1;
                a2 = p2 - c.share * v2 / c.tangentMass2;
            }
            const double limit2 = maxFriction * maxFriction;
            if (a1 * a1 + a2 * a2 > limit2) {
                // sliding: the whole allowed friction along tangent1 first - the slip direction when the tick
                // started (a circle cap of K^-1 v points partly sideways off the body's centre, and left too
                // little friction along the slide) - and what the circle leaves for the other tangent
                a1 = std::max(-maxFriction, std::min(a1, maxFriction));
                const double rest = std::sqrt(std::max(limit2 - a1 * a1, 0.0));
                a2 = std::max(-rest, std::min(a2, rest));
            }
            c.tangentImpulse1 = a1;
            c.tangentImpulse2 = a2;
            delta[i] = a1 - p1;
            delta2[i] = a2 - p2;
        }
        for (size_t i = 0; i < count; ++i) {
            if (delta[i] != 0.0 || delta2[i] != 0.0) {
                ApplyImpulsePair(contacts[i].rb1, contacts[i].rb2,
                                 contacts[i].tangent1 * delta[i] + contacts[i].tangent2 * delta2[i],
                                 contacts[i].r1, contacts[i].r2);
            }
        }
    }
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
    Vector3 localTorqueImpulse = transform->quaternion.Rotate(torqueImpulse);          // world -> local
    Vector3 local_delta_w = localTorqueImpulse * invertInertia;
    Vector3 ang_impulse = transform->quaternion.RotateConjugated(local_delta_w);      // local -> world

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
        transform->quaternion.Rotate(angularImpulse);                                  // world -> local

    Vector3 localDeltaW = localImpulse * invertInertia;

    Vector3 deltaW =
        transform->quaternion.RotateConjugated(localDeltaW);

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
    if (customInertia) {
        SetInertia(inertia);   // set by hand before the body was attached: not the box formula
        return;
    }
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

void Rigidbody::SetInertia(const Vector3& principal) {
    customInertia = true;
    inertia = principal;
    invertInertia = inertia.Inverse();
    invertInertiaMetrix[0][0] = invertInertia.x;
    invertInertiaMetrix[1][1] = invertInertia.y;
    invertInertiaMetrix[2][2] = invertInertia.z;
    if (transform != nullptr) {
        UpdateInertiaWorld();
    }
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
    force.Zero();
    torque.Zero();
    cache->SetDirty();
    // The transform was just reset: refresh the world inertia too (it still held the pre-reset rotation).
    UpdateInertiaWorld();
}







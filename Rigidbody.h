//
// Created by yaly on 29/06/2026.
//

#ifndef BERESHITCORE_RIGIDBODY_H
#define BERESHITCORE_RIGIDBODY_H

#include <vector>
#include "Component.h"
#include "Transform.h"
#include "Vector3.h"
class World;
#include <optional>

class Rigidbody : public Component {
private:

    double InvertWorld[3][3];
    static double GetFrictionCoefficient(const Rigidbody&, const Rigidbody&);
    void UpdateInertiaWorld();
    double restitution = 0.6;
    static void ApplyFrictionImpulse(Rigidbody& rb1, Rigidbody& rb2,const Vector3& relativeVelocity,
        const Vector3& normal , double J, const Vector3& r1, const Vector3& r2);

    static std::optional<std::tuple<double, Vector3, Vector3, Vector3, double>> FindImpulse(Rigidbody& rb1, Rigidbody& rb2, const Vector3& contact_point, const Vector3& normal, double dt);

    Transform* transform = nullptr;
    Cache* cache = nullptr;
    double invertInertiaMetrix[3][3] = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0}
    };
    Vector3 invertInertia;
    Vector3 inertia;
    bool isKinematic = false;
    double mass = 1.0;
    double invMass = 1.0;
    double drag = 0.98;
    double energy = 0.0;
    double frictionCoefficient = 0.6;
    bool useGravity = true;

public:
    Rigidbody *Copy() const override;
    void ApplyGravity(const Vector3& gravity);

    void SetIsKinematic(bool state);

    void SetUseGravity(bool state);

    [[nodiscard]] double GetRestitution() const {
        return restitution;
    }
    [[nodiscard]] double GetFrictionCoefficient() const {
        return frictionCoefficient;
    }
    [[nodiscard]] bool IsKinematic() const {
        return isKinematic;
    }
    [[nodiscard]] double GetMass() const {
        return mass;
    }
    [[nodiscard]] Vector3 GetInertia() const {
        return inertia;
    }
    // Override the box inertia computed in attach() (principal moments in the body frame, kg*m^2).
    // Call after the body was added to its GameObject.
    void SetInertia(const Vector3& principal);
    [[nodiscard]] double GetInvMass() const {
        return invMass;
    }


    std::string material;



    Vector3 center_of_mass;
    Vector3 velocity;
    Vector3 acceleration;
    Vector3 angularAcceleration;
    Vector3 torque;
    Vector3 force;



    Vector3 forward;
    Vector3 up;
    Vector3 freezeRotation;

    Vector3 angularVelocity;
    Vector3 normal_force;

    Rigidbody(float mass= 1, bool isKinematic = false, Vector3 initialVelocity = Vector3(), Vector3 initialAngularVelocity = Vector3(),
        bool useGravity = true, float frictionCoefficient=0.6, float restitution=0.6,
        Vector3 freezeRotation = Vector3());

    void PhysicsUpdate(double dt) override;
    void PhysicsUpdateFirstIteration(double dt) override;
    void integrate(double dt);
    void ForceIntegrate(double dt);
    static void PositionalCorrection(const Rigidbody&, const Rigidbody&, double,const Vector3&, double);
    static void SolveImpulse(Rigidbody& rb1, Rigidbody& rb2, const Vector3& contact_point, const Vector3& normal, double penetration, double dt);
    // Sequential impulses with per-contact accumulation (call once per solver iteration).
    static void SolveContact(struct Contact& contact, double dt);
    static void SolveContactsJacobi(std::vector<struct Contact>& contacts, double dt, int passes);
    static double EffectiveInverseMass(Rigidbody& rb1, Rigidbody& rb2, const Vector3& r1, const Vector3& r2, const Vector3& direction);
    double (*GetInvertWorld())[3][3] {
        return &InvertWorld;
    }
    static double FindRestitution(const Rigidbody& rb1, const Rigidbody& rb2, double normalVelocity);
    static void ApplyImpulsePair(Rigidbody& rb1, Rigidbody& rb2, const Vector3& impulseVec, const Vector3& r1, const Vector3& r2);
    void ApplyTorqueImpulse(Vector3 impulse, Vector3 r);
    void ApplyAngularImpulse(const Vector3& angularImpulse);
    void attach(GameObject& obj) override;

    static void SolveFrictionImpulse(Rigidbody& rb1, Rigidbody& rb2, const Vector3& contact_point, const Vector3& normal, double dt);
    void ResetToDefault() override;
};


#endif //BERESHITCORE_RIGIDBODY_H

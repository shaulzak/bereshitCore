//
// Created by yaly on 19/07/2026.
//

#ifndef BERESHITCORE_JOINT_H
#define BERESHITCORE_JOINT_H


#include "Component.h"
#include <iostream>

class GameObject;
class Transform;
#include "Vector3.h"
#include "Vector2.h"
#include "Quaternion.h"
#include "Collider.h"


class Joint : public Component{
    protected:
        bool hasWorldAnchor = false;
        void RemapReferences(const GameObjectMap& objectMap) override;

        Transform* transformA;
        Transform* transformB;
        Rigidbody* rbB = nullptr;
        Rigidbody* rbA = nullptr;
        GameObject* bodyB;
        GameObject* bodyA = nullptr;
        Vector3 localAnchorB;
        Vector3 localAnchorA;
        Quaternion initialRelativeRotation;
        Vector3 worldAnchor;
        double beta;
        double invMassArray[3];
        Vector3 angularImpulse;
        double K[3][3];

        void AddMatrix(const std::array<std::array<double, 3>, 3> &IA, const std::array<std::array<double, 3>, 3> &IB);
        void AddMatrix(const double (&IA)[3][3], const double (&IB)[3][3]);

        void BuildEffectiveMassMatrix(double invertMass, const Vector3& rA, const Vector3& rB, const double (&IinvA)[3][3],
            const double (&IinvB)[3][3]);

        void SetAngular(const Vector3& R, const double (&I)[3][3]);
        void AddAngular(const Vector3& R, const double (&I)[3][3]);
        static void CrossInertiaCross(const Vector3& R, const double (&I)[3][3], double (&M)[3][3]);

        Vector3 Solve3x3(const Vector3&);
        Vector2 Solve2x2(const Vector2 &beta, Vector2(&K)[2]);

    private:

        inline static bool printedLinear = true;
        inline static bool printedAngular = true;

        virtual void CastAnchorDefault();

        virtual void SolveLinear(double dt) {
            if (printedLinear) {
                std::cerr << GetName() << ".SolveLinear(dt) was not implemented" << '\n';
                printedLinear = false;
            }
        }
        virtual void SolveAngular(double dt) {
            if (printedAngular) {
                std::cerr << GetName() << ".SolveAngular(dt) was not implemented" << '\n';
                printedAngular = false;
            }
        }

    public:
        virtual Joint* Copy() const override;
        Joint(GameObject* bodyB, Vector3* anchor = nullptr, double beta = 0.2);


        void attach(GameObject &obj) override;
        void CastAnchor();
        void CastAnchor(Vector3 anchor);
        Vector3 GetAnchor() {
            return worldAnchor;
        }
        GameObject* GetBodyB() const { return bodyB; }
        void SetBodyB(GameObject* body);   // only before attach: a copy is pointed at the copied body


    virtual void Solve(double dt);
    void PhysicsUpdate(double dt) override;
};


#endif //BERESHITCORE_JOINT_H

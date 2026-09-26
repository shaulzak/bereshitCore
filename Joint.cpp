//
// Created by yaly on 19/07/2026.
//

#include "Joint.h"
#include "GameObject.h"
#include "Rigidbody.h"
#include "Physics.h"
#include <stdexcept>

void Joint::RemapReferences(const GameObjectMap& objectMap)
{
    if (bodyB == nullptr) {
        return;
    }

    auto it = objectMap.find(bodyB);

    if (it != objectMap.end()) {
        bodyB = it->second;
    } else {
        // bodyB was outside the copied hierarchy.
        bodyB = nullptr;
    }
}

void Joint::AddMatrix(const std::array<std::array<double, 3>, 3> &IA,
                      const std::array<std::array<double, 3>, 3> &IB) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K[i][j] = (IA)[i][j] + (IB)[i][j];
        }
    }
}

void Joint::AddMatrix(const double(&IA)[3][3], const double(&IB)[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K[i][j] = (IA)[i][j] + (IB)[i][j];
        }
    }
}

void Joint::BuildEffectiveMassMatrix(double invertMass, const Vector3& rA, const Vector3& rB, const double (&IinvA)[3][3],
                                     const double (&IinvB)[3][3]) {

    K[0][0] = invertMass;
    K[1][1] = invertMass;
    K[2][2] = invertMass;



    SetAngular(rA, IinvA);
    AddAngular(rB, IinvB);
}

void Joint::SetAngular(const Vector3& R, const double (&I)[3][3]) {
    // Cross-product matrix columns:
    //
    // cx = (0,  rz, -ry)
    // cy = (-rz, 0,  rx)
    // cz = (ry, -rx, 0)

    // I * cx
    double ixx = I[0][0];
    double ixy = I[0][1];
    double ixz = I[0][2];

    double iyx = I[1][0];
    double iyy = I[1][1];
    double iyz = I[1][2];

    double izx = I[2][0];
    double izy = I[2][1];
    double izz = I[2][2];


    (void)ixx; (void)ixy; (void)ixz; (void)iyx; (void)iyy; (void)iyz; (void)izx; (void)izy; (void)izz;
    // K = invMass * I3 (diagonal, set by the caller) + C^T * I * C. The full product is needed: the old
    // hand-expanded version dropped terms that are only zero when I is diagonal in world, i.e. for an
    // unrotated body, so every bent knee got a wrong effective mass.
    double M[3][3];
    CrossInertiaCross(R, I, M);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K[i][j] = (i == j ? K[i][j] : 0.0) + M[i][j];
        }
    }
}

void Joint::CrossInertiaCross(const Vector3& R, const double (&I)[3][3], double (&M)[3][3]) {
    // C = columns cx = (0, rz, -ry), cy = (-rz, 0, rx), cz = (ry, -rx, 0); M = C^T * I * C.
    const double C[3][3] = {
        {0.0, -R.z, R.y},
        {R.z, 0.0, -R.x},
        {-R.y, R.x, 0.0},
    };
    double IC[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            IC[i][j] = I[i][0] * C[0][j] + I[i][1] * C[1][j] + I[i][2] * C[2][j];
        }
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            M[i][j] = C[0][i] * IC[0][j] + C[1][i] * IC[1][j] + C[2][i] * IC[2][j];
        }
    }
}

void Joint::AddAngular(const Vector3& R, const double (&I)[3][3]) {
    // Cross-product matrix columns:
    //
    // cx = (0,  rz, -ry)
    // cy = (-rz, 0,  rx)
    // cz = (ry, -rx, 0)

    // I * cx
    double ixx = I[0][0];
    double ixy = I[0][1];
    double ixz = I[0][2];

    double iyx = I[1][0];
    double iyy = I[1][1];
    double iyz = I[1][2];

    double izx = I[2][0];
    double izy = I[2][1];
    double izz = I[2][2];


    (void)ixx; (void)ixy; (void)ixz; (void)iyx; (void)iyy; (void)iyz; (void)izx; (void)izy; (void)izz;
    double M[3][3];
    CrossInertiaCross(R, I, M);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K[i][j] += M[i][j];
        }
    }
}

Vector3 Joint::Solve3x3(const Vector3 &b) {
        double a = K[0][0];
        double b1 = K[0][1];
        double c = K[0][2];

        double d = K[1][0];
        double e = K[1][1];
        double f = K[1][2];

        double g = K[2][0];
        double h = K[2][1];
        double i = K[2][2];

        // determinant
        double det = (
                a * (e * i - f * h)
                - b1 * (d * i - f * g)
                + c * (d * h - e * g)
        );

        if (std::abs(det) < 1e-12) {
            throw std::runtime_error("Singular matrix");
        }

        double inv_det = 1.0 / det;

        // inverse matrix entries
        double m00 = (e * i - f * h) * inv_det;
        double m01 = (c * h - b1 * i) * inv_det;
        double m02 = (b1 * f - c * e) * inv_det;

        double m10 = (f * g - d * i) * inv_det;
        double m11 = (a * i - c * g) * inv_det;
        double m12 = (c * d - a * f) * inv_det;

        double m20 = (d * h - e * g) * inv_det;
        double m21 = (b1 * g - a * h) * inv_det;
        double m22 = (a * e - b1 * d) * inv_det;

        return  {m00 * b[0] + m01 * b[1] + m02 * b[2], m10 * b[0] + m11 * b[1] + m12 * b[2], m20 * b[0] + m21 * b[1] + m22 * b[2]};

}

Vector2 Joint::Solve2x2(const Vector2 &beta, Vector2(&K)[2]) {
    Vector2 vec;
    double a = K[0].x;
    double c = K[0].y;

    double d = K[1].x;
    double e = K[1].y;

    double det = a * e - c * d;

    if (std::abs(det) < 1e-12) {
        throw std::runtime_error("Singular matrix");
    }

    double inv_det = 1.0 / det;

    // inverse(K) * b
    vec.x = (e * beta.x - c * beta.y) * inv_det;
    vec.y = (-d * beta.x + a * beta.y) * inv_det;
    
    return vec;
}


void Joint::CastAnchorDefault() {
    worldAnchor = transformB->position;
}

Joint* Joint::Copy() const
{
    Joint* joint = new Joint(bodyB, nullptr, beta);
    joint->CastAnchor(worldAnchor);
    return joint;

}


Joint::Joint(GameObject* bodyB, Vector3* anchor, double beta): bodyB(bodyB), worldAnchor(anchor ? *anchor : Vector3()),
                                                                beta(beta), hasWorldAnchor(anchor != nullptr)
{

}



void Joint::attach(GameObject &obj) {
    bodyA = &obj;
    rbA = bodyA->GetComponent<Rigidbody>();
    rbB = bodyB->GetComponent<Rigidbody>();
    transformA = &bodyA->transform;
    transformB = &bodyB->transform;

    if (!hasWorldAnchor) {
        CastAnchor();
        hasWorldAnchor = true;
    } else {
        // An explicit anchor was given: CastAnchor() isn't called, so set up what it would have.
        initialRelativeRotation = (bodyA->transform.quaternion.Inverse() * transformB->quaternion);
        // Rotate() maps world -> local (RotateConjugated is local -> world, as the solvers use it).
        localAnchorA = transformA->quaternion.Rotate(worldAnchor - transformA->position);
        localAnchorB = transformB->quaternion.Rotate(worldAnchor - transformB->position);
    }
}

void Joint::CastAnchor() {
    auto hit = Physics::RayCast(transformA->position, (transformB->position - transformA->position), bodyB->GetComponent<Collider>());

    if (hit.collider != nullptr) {
        worldAnchor = hit.point;
        hasWorldAnchor = true;

    }else {
        CastAnchorDefault();
    }
    initialRelativeRotation = (bodyA->transform.quaternion.Inverse() * transformB->quaternion);

    // Rotate() maps world -> local; the solvers map back with RotateConjugated(). Using
    // RotateConjugated here too was only right for bodies that start unrotated.
    localAnchorA = transformA->quaternion.Rotate(worldAnchor - transformA->position);
    localAnchorB = transformB->quaternion.Rotate(worldAnchor - transformB->position);
}

void Joint::CastAnchor(Vector3 anchor) {
    worldAnchor = anchor;
    hasWorldAnchor = true;
}

void Joint::Solve(double dt) {
    rbA->ForceIntegrate(dt);
    rbB->ForceIntegrate(dt);
    SolveLinear(dt);
    SolveAngular(dt);
}

void Joint::PhysicsUpdate(double dt) {

}




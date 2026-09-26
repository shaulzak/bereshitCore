//
// Created by yaly on 08/07/2026.
//

#ifndef BERESHITCORE_CONTACT_H
#define BERESHITCORE_CONTACT_H

#pragma once
#include "Vector3.h"
class Rigidbody;

struct Contact {
    Rigidbody& rb1;
    Rigidbody& rb2;
    Vector3 normal;
    double penetration;
    Vector3 contact_point;

    // Sequential-impulse state, built on the first solve of the tick and kept across iterations.
    bool prepared = false;
    double velocityBias = 0.0;     // target separating speed: restitution bounce or penetration recovery
    double normalImpulse = 0.0;    // accumulated this tick, never negative (contacts only push)
    double tangentImpulse1 = 0.0;  // accumulated friction along tangent1 / tangent2, |.| <= mu * normalImpulse
    double tangentImpulse2 = 0.0;
    Vector3 tangent1;
    Vector3 tangent2;
    double normalMass = 0.0;       // inverse effective masses along the three directions
    double tangentMass1 = 0.0;
    double tangentMass2 = 0.0;
    double tangentMass12 = 0.0;    // and the tangents' cross term (friction is solved as one 2x2 system)
    Vector3 r1;                    // contact point relative to each body's centre (fixed during the tick)
    Vector3 r2;
    double share = 1.0;            // 1 / number of contacts between the same two bodies (parallel solving)
};


#endif //BERESHITCORE_CONTACT_H

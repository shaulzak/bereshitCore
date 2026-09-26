//
// Created by User on 20/07/2026.
//

#ifndef BERESHITCORE_FIXEDJOINT_H
#define BERESHITCORE_FIXEDJOINT_H

#include "Joint.h"

class FixedJoint : public Joint {
private:
    void SolveLinear(double dt) override;
    void SolveAngular(double dt) override;

public:
    using Joint::Joint;
    FixedJoint* Copy() const override;
};


#endif //BERESHITCORE_FIXEDJOINT_H

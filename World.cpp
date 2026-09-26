//
// Created by yaly on 30/06/2026.
//

#include <iostream>
#include <exception>

#include "World.h"
#include "GameObject.h"
#include "Rigidbody.h"
#include "Collider.h"
#include "Joint.h"

void World::ApplyGravityToAll(const Vector3 &gravity) {
    for (Rigidbody* rigidbody : GetAllRigidbodys()) {
        rigidbody->ApplyGravity(gravity);
    }
}

void World::IntegrateAll(double dt) {
    for (Rigidbody* rigidbody : GetAllRigidbodys()) {
        rigidbody->integrate(dt);
    }
}

void World::GetAllChildren(std::vector<GameObject *> &result) {
    for (GameObject* child : children) {
        result.push_back(child);
        child->GetAllChildren(result);
    }
}
void World::GetAllChildrenPhysics(std::vector<GameObject *> &result) {
    for (GameObject* child : children) {
        if (child->GetComponent<Rigidbody>() && child->GetComponent<Collider>()) {
            result.push_back(child);
        }

        child->GetAllChildrenPhysics(result);
    }
}

void World::GetAllColliders(std::vector<Collider*> &result) {
    for (GameObject* child : GetAllChildrenPhysics()) {
            result.push_back(child->GetComponent<Collider>());
    }
}

void World::GetAllRigidbodys(std::vector<Rigidbody *> &result) {
    for (GameObject* child : GetAllChildrenPhysics()) {
        result.push_back(child->GetComponent<Rigidbody>());
    }
}

void World::GetAllJoints(std::vector<Joint*> &result) {
    for (GameObject* child : GetAllChildrenPhysics()) {
        auto joints = child->GetComponents<Joint>();
        for (auto* joint : joints) {
            result.push_back(joint);
        }
    }
}

void World::SetCacheAllChildren() {
    cacheAllChildren.clear();
    GetAllChildren(cacheAllChildren);
    allChildrenDirty = false;
}

void World::SetCachePhysicsChildren() {
    cachePhysicsChildren.clear();
    GetAllChildrenPhysics(cachePhysicsChildren);
    physicsChildrenDirty = false;
}

void World::SetCacheColliders() {
    cacheColliders.clear();
    GetAllColliders(cacheColliders);
    CollidersDirty = false;
}

void World::SetCacheRigidbodys() {
    cacheRigidbodys.clear();
    GetAllRigidbodys(cacheRigidbodys);
    CollidersDirty = false;

}

void World::SetCacheJoints() {
    cacheJoints.clear();
    GetAllJoints(cacheJoints);
    CollidersDirty = false;
}

void World::SetCache() {
    SetCacheAllChildren();
    SetCachePhysicsChildren();
    SetCacheColliders();
    SetCacheRigidbodys();
    SetCacheJoints();
}





World::World(bool *running_flag, std::vector<GameObject *> children, GameObject* gizmos, Vector3 gravity,
             double tick, double speed, int physics_epochs) : gravity(gravity), tick(tick), physics_epochs(physics_epochs), speed(speed), gizmos(gizmos), children(children) {

    this->gizmos = (new GameObject(Vector3(100000,100000,10000),Vector3(),Vector3(0.1,0.1,0.1)));

    for (int i = 0; i < 100; i++) {
        this->gizmos->GetChildren().push_back(new GameObject(
    Vector3(100000,100000,10000),
    Vector3(),
    Vector3(0.1,0.1,0.1)));
        }
    SetCache();
    for (auto child : getAllChildren()) {
        child->setWorld(this);
    }
}

void World::AddChild(GameObject *child) {
    // child was just linked under an object of this world. Rebuild every list with the same rules as the
    // constructor: adding it piecemeal missed its Rigidbody (no gravity, never integrated), took colliders
    // without a Rigidbody (dereferenced by the contact search) and joints of objects outside the physics.
    SetChildWorld(child);
    SetCache();
}

void World::SetChildWorld(GameObject *object) {
    object->setWorld(this);
    for (auto child : object->GetAllChildren()) {
        child->setWorld(this);
    }
}

std::vector<GameObject*>& World::getAllChildren(){
    return cacheAllChildren;
}
std::vector<GameObject *>& World::GetAllChildrenPhysics(){
    return cachePhysicsChildren;
}

std::vector<Collider*>& World::GetAllColliders() {
    return cacheColliders;
}

std::vector<Rigidbody *> & World::GetAllRigidbodys() {
    return cacheRigidbodys;
}
std::vector<Joint*> & World::GetAllJoints() {
    return cacheJoints;
}

std::vector<GameObject *> World::getGizmos() const {
    return gizmos->GetChildren();
}

std::vector<GameObject*> World::search_by_component(std::string name) const {
    std::vector<GameObject*> results;

    for (GameObject* child : children) {
        auto child_results = child->SearchByComponent(name);
        results.insert(results.end(),
                       child_results.begin(),
                       child_results.end());
    }

    return results;
}

void World::SolveCollections(std::vector<Contact> &contacts, double dt) {
    // In parallel, not one contact after another (see Rigidbody::SolveContactsJacobi): the result must not
    // depend on the contacts' order, or mirror-image motions come out different. One parallel pass converges
    // slower than a sequential one (a robot left standing kept rocking forward/back), 4 match it.
    constexpr int kContactPasses = 4;
    Rigidbody::SolveContactsJacobi(contacts, dt, kContactPasses);
}

void World::CorrectPositions(const std::vector<Contact> &contacts) {
    for (const auto& contact : contacts) {
        // Linear-only effective mass: the correction is a pure translation.
        double invMassSum = contact.rb1.GetInvMass() + contact.rb2.GetInvMass();
        if (invMassSum == 0.0) {
            continue;
        }
        Rigidbody::PositionalCorrection(contact.rb1, contact.rb2, contact.penetration, contact.normal, invMassSum);
    }
}

void World::SolveJoints(const std::vector<Joint*>& joints, double dt) {
    for (auto& joint : joints) {
        try {
            joint->Solve(dt);
        } catch (const std::exception&) {
            // e.g. a singular effective-mass matrix in a degenerate pose: skip this joint for this
            // iteration instead of aborting the whole step (and the Python training with it).
        }
    }
}


void World::Start() {
    auto children = getAllChildren();
    for (const GameObject* child : children) {
        for (auto& component : child->GetComponents()) {
            component->Start();
        }
    }
}



void World::CallChildrenUpdate(const std::vector<GameObject *> &list, double dt, void (Component::*func)(double)) {
    for (const auto& child : list) {
        for (auto component : child->GetComponents()) {
            try {
                (component->*func)(dt);
            }
            catch (const std::exception& e) {
                std::cerr << "Error in component Update: " << component->GetName() << ": " << e.what() << std::endl;
            }
            catch (...) {
                std::cerr << "Unknown error in component Update" << std::endl;
            }
        }
    }
}


std::vector<Contact> World::SolveCollectionsFirstIteration(const std::vector<Collider*>& colliders, double dt) const {
    std::vector<Contact> contacts;

    auto candidate_pairs = Collider::SweepAndPrune(colliders);

    for (const auto& [Collider1, Collider2] : candidate_pairs) {
        auto rb1 = Collider1->GetParent()->GetComponent<Rigidbody>();
        auto rb2 = Collider2->GetParent()->GetComponent<Rigidbody>();

         if (rb1->IsKinematic() && rb2->IsKinematic()) {
             continue;
         }
        auto result = Collider1->CheckCollision(Collider2);

        if (result.contact_points.empty()){
            continue;
        }
        Vector3 normal = result.normal;
        for (size_t i = 0; i < result.contact_points.size(); ++i) {
            const Vector3& contact_point = result.contact_points[i];
            double depth = result.depth[i];

            // Rigidbody::SolveFrictionImpulse(*rb1, *rb2, contact_point, normal, dt);
            contacts.push_back({*rb1, *rb2, normal, depth, contact_point});
        }

        
    }

    return contacts;
}


void World::Update(bool updateComponen) {
    if (updateComponen) {
        CallChildrenUpdate(getAllChildren(), tick, &Component::Update);
    }
    const auto& PhysicsChildren = GetAllChildrenPhysics();
    CallChildrenUpdate(PhysicsChildren, tick, &Component::PhysicsUpdate);

    ApplyGravityToAll(gravity);

    auto collections = SolveCollectionsFirstIteration(GetAllColliders(), tick);

    for (int i =0; i < physics_epochs + 1; i++) {
        SolveCollections(collections, tick);
        SolveJoints(GetAllJoints(), tick);
    }

    CorrectPositions(collections);
    IntegrateAll(tick);
}



void World::SetGizmos(const std::vector<Contact>& contacts) {
    auto gizmoObjects = gizmos->GetAllChildren();

    auto gizmoIt = gizmoObjects.begin();
    auto contactIt = contacts.begin();

    while (gizmoIt != gizmoObjects.end() && contactIt != contacts.end()) {
        (*gizmoIt)->transform.position = contactIt->contact_point;
        ++gizmoIt;
        ++contactIt;
    }
}







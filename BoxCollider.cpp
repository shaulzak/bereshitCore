//
// Created by yaly on 15/07/2026.
//

#include "BoxCollider.h"
#include <algorithm>
#include "GameObject.h"
#include <numeric>

std::pair<Vector3, Vector3> BoxCollider::GetEdgeSegment(const Vector3& center,const std::array<Vector3, 3> &axes, const Vector3& half,int axis_index,const Vector3 &normal) {


    Vector3 axis = axes[axis_index];
    Vector3 dir1 = axes[(axis_index + 1) % 3];
    Vector3 dir2 = axes[(axis_index + 2) % 3];

   Vector3 offset = (
           axis * double(half[axis_index]) +
           dir1 * double(half[(axis_index + 1) % 3]) * (dir1.dot(normal) > 0 ? 1 : -1) +
           dir2 * double(half[(axis_index + 2) % 3]) * (dir2.dot(normal) > 0 ? 1 : -1)
   );

   Vector3 p = center + offset - axis * double(half[axis_index]);
   Vector3 q = center + offset + axis * double(half[axis_index]);

   return {p, q};

}

std::array<Vector3, 4> BoxCollider::
GetFaceVertices(const Vector3 &center,const std::array<Vector3, 3> &axes, const Vector3 &half, int axis_index, const Vector3 &normal) {

    Vector3 u = axes[(axis_index + 1) % 3];
    Vector3 v = axes[(axis_index + 2) % 3];

    double hu = double(half[(axis_index + 1) % 3]);
    double hv = double(half[(axis_index + 2) % 3]);
    double hn = double(half[axis_index]);

    Vector3 face_center = center + normal * hn;

    return {
       face_center + u * hu + v * hv,
       face_center - u * hu + v * hv,
       face_center - u * hu - v * hv,
       face_center + u * hu - v * hv,
       };
}

std::array<Vector3, 4> BoxCollider::GetIncidentFace(const Vector3 &center, const std::array<Vector3, 3> &axes, const Vector3 &half_sizes, const Vector3 &collision_normal) {
    int best_index = 0;
    double best_dot = axes[0].dot(collision_normal);
    double d;
    for (int i = 1; i < 3; i++) {
        d = axes[i].dot(collision_normal);
        if (std::abs(d) > std::abs(best_dot)) {
            best_dot = d;
            best_index = i;
        }
    }


    Vector3 face_normal = axes[best_index];
    if (face_normal.dot(collision_normal) > 0) {
        face_normal.NegativeSelf();
    }

    return GetFaceVertices(
        center,
        axes,
        half_sizes,
        best_index,
        face_normal
    );
}



std::vector<Vector3> BoxCollider::ClipPolygon(const std::vector<Vector3>& poly,const Vector3& planePoint, const Vector3& planeNormal){
    std::vector<Vector3> result;

    for (size_t i = 0; i < poly.size(); ++i) {
        const Vector3& a = poly[i];
        const Vector3& b = poly[(i + 1) % poly.size()];

        double da = (a - planePoint).dot(planeNormal);
        double db = (b - planePoint).dot(planeNormal);

        if (da <= 0.0)
            result.push_back(a);

        if (da * db < 0.0) {
            double t = da / (da - db);
            result.push_back(a + (b - a) * t);
        }
    }

    return result;
}

BoxCollider * BoxCollider::Copy() const {
    BoxCollider* box_collider = new BoxCollider(isTrigger);
    return box_collider;
}

ContactPoints BoxCollider::CheckCollision(Collider* collider2)  {
    ContactPoints contact_points;

    if (typeid(*collider2) == typeid(BoxCollider)) {
        bool aabb_hit = AabbCollision(collider2);
        if (!aabb_hit) {
            HandleCollisionExit();
            collider2->HandleCollisionExit();
            return contact_points;

        }
        auto sat_result = Sat(collider2);
        if (!sat_result) {
            HandleCollisionExit();
            collider2->HandleCollisionExit();
            return contact_points;
        }
        SatResult result = *sat_result;
        contact_points = GenerateContacts(result);

        HandleCollisionEvents(collider2, &contact_points);
        collider2->HandleCollisionEvents(this, &contact_points);

        if (GetIsTrigger() || collider2->GetIsTrigger()) {
            return contact_points;
        }
        return contact_points;
    }
    return contact_points;
}

bool BoxCollider::AabbCollision(const Collider* collider2) {
    auto [min1, max1] = GetAabb();
    auto [min2, max2] = collider2->GetAabb();

    return (min1.x <= max2.x && max1.x >= min2.x)
            &(min1.y <= max2.y && max1.y >= min2.y)
            &(min1.z <= max2.z && max1.z >= min2.z);

}





std::optional<Collider::SatResult> BoxCollider::Sat(const Collider* otherCollider) const {
    Vector3 ACenter = GetPosition();
    Vector3 BCenter = otherCollider->GetPosition();

    auto aAxes = GetAxes(GetQuaternion(), GetParent()->cache);
    auto bAxes = GetAxes(otherCollider->GetQuaternion().Conjugate());

    Vector3 aHalf = GetSize() * 0.5;
    Vector3 bHalf = otherCollider->GetSize() * 0.5;
    Axes axes_to_test[15];
    int k = 0;
    for (int i = 0; i < 3; i++) {
        axes_to_test[k] = Axes(Source(0), i, aAxes[i]);
        axes_to_test[k+1] = Axes(Source(1), i, bAxes[i]);
        k += 2;
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Vector3 cross = aAxes[i].cross(bAxes[j]);
            if (cross.magnitude() > 1e-6) {
                axes_to_test[k] = Axes(Source(2), std::pair<int, int>(i, j), cross.normalized());
                k += 1;
            }
        }
    }



        double smallest_overlap = std::numeric_limits<double>::max();
        Vector3 collision_axis;
        Source collision_type;
        std::variant<int, std::pair<int,int>> collision_axis_indices;
    for (size_t i = 0; i < k; ++i) {
        auto& [source, indices, axis] = axes_to_test[i];

        auto proj_a = ProjectBox(ACenter, aAxes, aHalf, axis);
        auto proj_b = ProjectBox(BCenter, bAxes, bHalf, axis);

        if (!OverlapOnAxis(proj_a, proj_b)) {
            return std::nullopt;
        }

        double overlap = std::min(proj_a[1], proj_b[1]) - std::max(proj_a[0], proj_b[0]);
        if (overlap < smallest_overlap) {
            smallest_overlap = overlap;
            collision_axis = axis;
            collision_type = source;
            collision_axis_indices = indices;
        }
    }
    if ((BCenter - ACenter).dot(collision_axis) < 0) {
        collision_axis = -collision_axis;
    }
    return SatResult(collision_axis, smallest_overlap,
                collision_type, collision_axis_indices, *this, *otherCollider);

}

std::array<std::array<double, 3>, 3> Transpose(const std::array<std::array<double, 3>, 3>& R) {
    std::array<std::array<double, 3>, 3> Rt{};

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            Rt[i][j] = R[j][i];

    return Rt;
}



RayCastHit BoxCollider::RayObbIntersection(const Vector3 &rayOrigin, const Vector3 &rayDirection,
    const Vector3 &boxCenter,const std::array<std::array<double, 3>, 3>& rotationMatrix, const Vector3 &halfSize) const {

    auto invRot = Transpose(rotationMatrix);

    Vector3 localOrigin = (rayOrigin - boxCenter).MatrixMultiplication(invRot);

    Vector3 local_dir = rayDirection.MatrixMultiplication(invRot);

    Vector3 box_min = -halfSize;
    Vector3 box_max = halfSize;

    return RayBoxIntersection(localOrigin, local_dir, box_min, box_max);

}

RayCastHit BoxCollider::RayBoxIntersection(const Vector3 &rayOrigin, const Vector3 &rayDirection,
    const Vector3 &box_min, const Vector3 &box_max) const {
    double tmin = std::numeric_limits<double>::lowest();
    double tmax = std::numeric_limits<double>::max();
    // double origin;
    // double direction;
    // double bmin;
    // double bmax;
    for (int i =0; i<3;i++) {
        double origin = rayOrigin[i];
        double direction = rayDirection[i];
        double bmin = box_min[i];
        double bmax = box_max[i];
        if (std::abs(direction) < 1e-8) {
            if (origin < bmin || origin > bmax) {
                return {};
            }
        }
        else{
            double t1 = (bmin - origin) / direction;
            double t2 = (bmax - origin) / direction;
            double t_near = std::min(t1, t2);
            double t_far = std::max(t1, t2);
            tmin = std::max(tmin, t_near);
            tmax = std::min(tmax, t_far);
            if (tmin > tmax)
                return {};
        }
    }
    // don't ray cast object behind the object
    if (tmax < 0) {
        return {};
    }
    double t_hit = tmin >= 0 ? tmin : tmax;
    Vector3 hit_point(rayOrigin.x + rayDirection.x * t_hit,
            rayOrigin.y + rayDirection.y * t_hit,
            rayOrigin.z + rayDirection.z * t_hit);
    RayCastHit hit;
    hit.point = hit_point + GetPosition();
    hit.collider = this;
    return hit;
}

// The box's own axes in the world. R maps world -> local, so they are R's rows (the columns of R^T, the
// same as GetAxes(quaternion.Conjugate()) below). This returned the columns - the axes of the inverse rotation.
std::array<Vector3, 3> BoxCollider::GetAxes(const Quaternion &quaternion, Cache &cache) {
    auto R = quaternion.ToMatrix3(&cache);
    return {
        Vector3(R[0][0], R[0][1], R[0][2]).normalized(),
        Vector3(R[1][0], R[1][1], R[1][2]).normalized(),
        Vector3(R[2][0], R[2][1], R[2][2]).normalized()
    };
}

std::array<Vector3, 3> BoxCollider::GetAxes(const Quaternion &quaternion) {
    std::array<std::array<double, 3>, 3> R;
    quaternion.ToMatrix3(R);
    return {
        Vector3(R[0][0], R[1][0], R[2][0]).normalized(),
        Vector3(R[0][1], R[1][1], R[2][1]).normalized(),
        Vector3(R[0][2], R[1][2], R[2][2]).normalized()
    };
}

std::array<double, 2> BoxCollider::ProjectBox(const Vector3& center,const std::array<Vector3, 3>& axes, const Vector3& half_sizes, const Vector3& axis) {
    double c = center.x * axis.x + center.y * axis.y + center.z * axis.z;

    Vector3 a0 = axes[0];
    Vector3 a1 = axes[1];
    Vector3 a2 = axes[2];

    double d0 = axis.x * a0.x + axis.y * a0.y + axis.z * a0.z;
    double d1 = axis.x * a1.x + axis.y * a1.y + axis.z * a1.z;
    double d2 = axis.x * a2.x + axis.y * a2.y + axis.z * a2.z;

    double r = std::abs(d0) * half_sizes.x + std::abs(d1) * half_sizes.y + std::abs(d2) * half_sizes.z;

    return {c - r, c + r};

}

std::pair<Vector3, Vector3> BoxCollider::ClosestPointsBetweenSegments(const Vector3& p1,const Vector3& q1,const Vector3& p2, const Vector3& q2){
    Vector3 d1 = q1 - p1;
    Vector3 d2 = q2 - p2;
    Vector3 r = p1 - p2;

    double a = d1.dot(d1);
    double e = d2.dot(d2);
    double f = d2.dot(r);

    constexpr double EPSILON = 1e-6;

    if (a <= EPSILON && e <= EPSILON)
    {
        return {p1, p2};
    }

    if (a <= EPSILON)
    {
        double t = std::clamp(f / e, 0.0, 1.0);
        return {p1, p2 + d2 * t};
    }

    double c = d1.dot(r);

    if (e <= EPSILON)
    {
        double s = std::clamp(-c / a, 0.0, 1.0);
        return {p1 + d1 * s, p2};
    }

    double b = d1.dot(d2);
    double denom = a * e - b * b;

    double s;
    if (denom != 0.0)
    {
        s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
    }
    else
    {
        s = 0.0;
    }

    double t = (b * s + f) / e;

    if (t < 0.0)
    {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
    }
    else if (t > 1.0)
    {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
    }

    return {
        p1 + d1 * s,
        p2 + d2 * t
    };
}

ContactPoints BoxCollider::GenerateContacts(SatResult &sat_result) const {
    Vector3 normal = sat_result.normal;
    double penetration = sat_result.penetration;
    Source collision_type = sat_result.type;
    std::variant<int, std::pair<int, int>> axis_index = sat_result.collisionAxisIndices;

    const Collider& A = sat_result.A;
    const Collider& B = sat_result.B;

    Vector3 ACenter = A.GetPosition();
    Vector3 BCenter = B.GetPosition();

    auto aAxes = GetAxes(A.GetQuaternion(), A.GetParent()->cache);
    auto bAxes = GetAxes(B.GetQuaternion(), B.GetParent()->cache);

    Vector3 aHalf = A.GetSize() * 0.5;
    Vector3 bHalf = B.GetSize() * 0.5;

    if (static_cast<int>(collision_type) == 2) {
        auto [i, j] = std::get<std::pair<int, int>>(axis_index);

        auto [p1, q1] = GetEdgeSegment(ACenter, aAxes, aHalf, i, normal);
        auto [p2, q2] = GetEdgeSegment(BCenter, bAxes, bHalf, j, -normal);

        auto [c1, c2] = ClosestPointsBetweenSegments(p1, q1, p2, q2);

        Vector3 contact_point = (c1 + c2) * 0.5;
        return ContactPoints({contact_point}, normal, {penetration});
    }
    int ref_axis_index = std::get<int>(axis_index);
    bool flip;
    const Collider* ref;
    const Collider* inc;
    if (static_cast<int>(collision_type) == 0) {
        ref = &A;
        inc = &B;
        flip = false;

    }else {
        ref = &B;
        inc = &A;
        flip = true;
        normal = -normal;
    }

    Vector3 ref_center = ref->GetPosition();
    auto ref_axes = GetAxes(ref->GetQuaternion().Conjugate());
    Vector3 ref_half = ref->GetSize() * 0.5;

    Vector3 inc_center = inc->GetPosition();
    auto inc_axes = GetAxes(inc->GetQuaternion().Conjugate());
    Vector3 inc_half = inc->GetSize() * 0.5;

    Vector3 ref_normal = ref_axes[ref_axis_index];
    if (ref_normal.dot(normal) < 0) {
        ref_normal = -ref_normal;
    }

    auto ref_face = GetFaceVertices(ref_center, ref_axes, ref_half, ref_axis_index,ref_normal);

    auto inc_face = GetIncidentFace(inc_center, inc_axes, inc_half, normal);

    std::vector<Vector3> clipped(inc_face.begin(), inc_face.end());

    for (int i = 0; i < 4; ++i) {
        const Vector3& p1 = ref_face[i];
        const Vector3& p2 = ref_face[(i + 1) % 4];

        Vector3 edge = p2 - p1;
        Vector3 planeNormal = edge.cross(ref_normal).normalized();

        Vector3 toCenter = ref_center - p1;

        if (planeNormal.dot(toCenter) < 0.0) {
            planeNormal.NegativeSelf();
        }

        clipped = ClipPolygon(clipped, p1, -planeNormal);

        if (clipped.empty()) {
            break;
        }
    }

    std::vector<Vector3> contacts;
    std::vector<double> depths;
    double ref_plane_d = ref_normal.dot(ref_face[0]);

    for (auto& p : clipped) {
        double depth = ref_plane_d - ref_normal.dot(p);
        if (depth >= 0) {
            Vector3 projected_p = p + ref_normal * depth;
            contacts.push_back(projected_p);
            depths.push_back(depth);
        }
    }
    if (!contacts.empty()) {
        Vector3 averagePoint = Vector3::Average(contacts);
        double averageDepth = std::accumulate(depths.begin(), depths.end(), 0.0)
             / depths.size();
            contacts.insert(contacts.begin(), averagePoint);
            depths.insert(depths.begin(), averageDepth);
    }

    return ContactPoints(contacts, sat_result.normal, depths);

}

RayCastHit BoxCollider::RayCast(const Vector3 &origin, const Vector3 &direction, double maxDistance) const {
    // return Collider::RayCast(origin, direction, maxDistance);
    return RayObbIntersection(origin, direction, GetPosition(), GetQuaternion().ToMatrix3(&GetParent()->cache), GetSize() * 0.5);

}

void BoxCollider::attach(GameObject &obj) {
    Collider::attach(obj);
    SetName("BoxCollider");
}






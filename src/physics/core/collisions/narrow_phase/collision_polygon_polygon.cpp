#include "physics/core/collisions/narrow_phase/collision_polygon_polygon.h"

#include "physics/core/shape.h"
#include "physics/math/matrix2x2.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace PhysicsEngine {
namespace {

struct FaceSeparation {
    float separation = -std::numeric_limits<float>::max();
    std::size_t faceIndex = 0;
};

std::vector<Vector2> GetWorldVertices(const RigidBody* body) {
    const auto* polygon = static_cast<const Polygon*>(body->shape);
    const Matrix2x2 rotation = Matrix2x2::rotation(body->GetOrientation());
    std::vector<Vector2> result;
    result.reserve(polygon->getVertices().size());
    for (const Vector2& vertex : polygon->getVertices()) {
        result.push_back(body->GetPosition() + rotation * vertex);
    }
    return result;
}

Vector2 OutwardNormal(
    const std::vector<Vector2>& vertices,
    std::size_t faceIndex
) {
    const Vector2 edge = vertices[(faceIndex + 1) % vertices.size()]
        - vertices[faceIndex];
    float signedDoubleArea = 0.0f;
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        signedDoubleArea += vertices[i].cross(
            vertices[(i + 1) % vertices.size()]
        );
    }
    return signedDoubleArea >= 0.0f
        ? Vector2(edge.y, -edge.x).normalized()
        : Vector2(-edge.y, edge.x).normalized();
}

FaceSeparation FindMaximumSeparation(
    const std::vector<Vector2>& reference,
    const std::vector<Vector2>& incident
) {
    FaceSeparation best;
    for (std::size_t face = 0; face < reference.size(); ++face) {
        const Vector2 normal = OutwardNormal(reference, face);
        float minimum = std::numeric_limits<float>::max();
        for (const Vector2& vertex : incident) {
            minimum = std::min(
                minimum,
                normal.dot(vertex - reference[face])
            );
        }
        if (minimum > best.separation) {
            best = FaceSeparation{minimum, face};
        }
    }
    return best;
}

std::size_t FindIncidentFace(
    const std::vector<Vector2>& vertices,
    const Vector2& referenceNormal
) {
    float minimumDot = std::numeric_limits<float>::max();
    std::size_t result = 0;
    for (std::size_t face = 0; face < vertices.size(); ++face) {
        const float alignment = OutwardNormal(vertices, face).dot(referenceNormal);
        if (alignment < minimumDot) {
            minimumDot = alignment;
            result = face;
        }
    }
    return result;
}

std::uint8_t ClipToPlane(
    const std::array<Vector2, 2>& input,
    std::uint8_t inputCount,
    const Vector2& normal,
    float offset,
    std::array<Vector2, 2>& output
) {
    if (inputCount == 0) {
        return 0;
    }

    const float firstDistance = normal.dot(input[0]) - offset;
    const float secondDistance = inputCount > 1
        ? normal.dot(input[1]) - offset
        : firstDistance;
    std::uint8_t outputCount = 0;

    if (firstDistance <= 0.0f) {
        output[outputCount++] = input[0];
    }
    if (inputCount > 1 && secondDistance <= 0.0f) {
        output[outputCount++] = input[1];
    }
    if (inputCount > 1 && firstDistance * secondDistance < 0.0f) {
        const float fraction = firstDistance / (firstDistance - secondDistance);
        output[outputCount++] = input[0] + (input[1] - input[0]) * fraction;
    }
    return std::min<std::uint8_t>(outputCount, 2);
}

std::uint32_t MakeFeatureId(
    bool referenceIsCanonicalB,
    std::size_t referenceFace,
    std::size_t incidentFace,
    std::uint8_t ordinal
) {
    return 0x80000000u
        | (static_cast<std::uint32_t>(referenceIsCanonicalB) << 30)
        | (static_cast<std::uint32_t>(referenceFace & 0x3fffu) << 16)
        | (static_cast<std::uint32_t>(incidentFace & 0x3fffu) << 2)
        | static_cast<std::uint32_t>(ordinal + 1);
}

CollisionManifold ComputeCanonicalManifold(RigidBody* a, RigidBody* b) {
    CollisionManifold manifold;
    manifold.A = a;
    manifold.B = b;

    const std::vector<Vector2> verticesA = GetWorldVertices(a);
    const std::vector<Vector2> verticesB = GetWorldVertices(b);
    const FaceSeparation separationA = FindMaximumSeparation(verticesA, verticesB);
    if (separationA.separation >= 0.0f) {
        return manifold;
    }
    const FaceSeparation separationB = FindMaximumSeparation(verticesB, verticesA);
    if (separationB.separation >= 0.0f) {
        return manifold;
    }

    // Canonical body order makes tie-breaking and feature IDs independent of
    // the order supplied by a caller.
    constexpr float referenceBias = 1e-5f;
    const bool referenceIsB = separationB.separation
        > separationA.separation + referenceBias;
    const std::vector<Vector2>& reference = referenceIsB ? verticesB : verticesA;
    const std::vector<Vector2>& incident = referenceIsB ? verticesA : verticesB;
    const std::size_t referenceFace = referenceIsB
        ? separationB.faceIndex
        : separationA.faceIndex;
    const Vector2 referenceNormal = OutwardNormal(reference, referenceFace);
    const std::size_t incidentFace = FindIncidentFace(incident, referenceNormal);

    const Vector2 referenceFirst = reference[referenceFace];
    const Vector2 referenceSecond = reference[(referenceFace + 1) % reference.size()];
    const Vector2 sideNormal = (referenceSecond - referenceFirst).normalized();
    std::array<Vector2, 2> incidentEdge{
        incident[incidentFace],
        incident[(incidentFace + 1) % incident.size()]
    };
    std::array<Vector2, 2> firstClip{};
    std::array<Vector2, 2> secondClip{};
    const std::uint8_t firstCount = ClipToPlane(
        incidentEdge,
        2,
        sideNormal * -1.0f,
        (sideNormal * -1.0f).dot(referenceFirst),
        firstClip
    );
    if (firstCount < 1) {
        return manifold;
    }
    const std::uint8_t secondCount = ClipToPlane(
        firstClip,
        firstCount,
        sideNormal,
        sideNormal.dot(referenceSecond),
        secondClip
    );
    if (secondCount < 1) {
        return manifold;
    }

    std::sort(
        secondClip.begin(),
        secondClip.begin() + secondCount,
        [&](const Vector2& left, const Vector2& right) {
            return sideNormal.dot(left) < sideNormal.dot(right);
        }
    );

    const float referenceOffset = referenceNormal.dot(referenceFirst);
    for (std::uint8_t i = 0; i < secondCount && manifold.contactCount < 2; ++i) {
        const float separation = referenceNormal.dot(secondClip[i]) - referenceOffset;
        if (separation <= 1e-5f) {
            const std::uint8_t contactIndex = manifold.contactCount++;
            manifold.contacts[contactIndex] = ContactPoint{
                secondClip[i],
                std::max(-separation, 0.0f),
                MakeFeatureId(referenceIsB, referenceFace, incidentFace, contactIndex)
            };
        }
    }
    if (manifold.contactCount == 0) {
        return manifold;
    }

    manifold.hasCollision = true;
    manifold.normal = referenceIsB ? referenceNormal * -1.0f : referenceNormal;
    manifold.penetration = 0.0f;
    for (std::uint8_t i = 0; i < manifold.contactCount; ++i) {
        manifold.penetration = std::max(
            manifold.penetration,
            manifold.contacts[i].penetration
        );
    }
    manifold.contactPoint = manifold.contacts[0].position;
    return manifold;
}

} // namespace

CollisionManifold CollisionPolygonPolygon(RigidBody* a, RigidBody* b) {
    if (a->GetId() <= b->GetId()) {
        return ComputeCanonicalManifold(a, b);
    }

    CollisionManifold manifold = ComputeCanonicalManifold(b, a);
    std::swap(manifold.A, manifold.B);
    manifold.normal = manifold.normal * -1.0f;
    return manifold;
}

} // namespace PhysicsEngine

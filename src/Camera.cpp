#include "Camera.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace DirectX;

Camera::Camera(float aspectRatio)
{
    SetAspectRatio(aspectRatio);
}

void Camera::SetAspectRatio(
    float aspectRatio)
{
    if (aspectRatio <= 0.0f)
    {
        throw std::invalid_argument(
            "Camera aspect ratio must be positive.");
    }

    aspectRatio_ = aspectRatio;
}

void Camera::SetPosition(
    const XMFLOAT3& position)
{
    position_ = position;
}

void Camera::Rotate(
    float yawDelta,
    float pitchDelta)
{
    yaw_ += yawDelta;
    pitch_ += pitchDelta;

    constexpr float pitchLimit =
        XM_PIDIV2 - 0.01f;

    pitch_ = std::clamp(
        pitch_,
        -pitchLimit,
        pitchLimit);
}

XMVECTOR Camera::GetForwardVector() const
{
    const float cosinePitch =
        std::cos(pitch_);

    return XMVector3Normalize(
        XMVectorSet(
            cosinePitch * std::sin(yaw_),
            std::sin(pitch_),
            cosinePitch * std::cos(yaw_),
            0.0f));
}

XMVECTOR Camera::GetRightVector() const
{
    constexpr XMVECTORF32 worldUp{
        0.0f,
        1.0f,
        0.0f,
        0.0f
    };

    return XMVector3Normalize(
        XMVector3Cross(
            worldUp,
            GetForwardVector()));
}

void Camera::MoveLocal(
    float right,
    float up,
    float forward)
{
    XMVECTOR position =
        XMLoadFloat3(&position_);

    constexpr XMVECTORF32 worldUp{
        0.0f,
        1.0f,
        0.0f,
        0.0f
    };

    position = XMVectorAdd(
        position,
        XMVectorScale(
            GetRightVector(),
            right));

    position = XMVectorAdd(
        position,
        XMVectorScale(
            worldUp,
            up));

    position = XMVectorAdd(
        position,
        XMVectorScale(
            GetForwardVector(),
            forward));

    XMStoreFloat3(
        &position_,
        position);
}

void Camera::LookAt(
    const XMFLOAT3& target)
{
    const XMVECTOR position =
        XMLoadFloat3(&position_);

    const XMVECTOR targetPosition =
        XMLoadFloat3(&target);

    const XMVECTOR difference =
        XMVectorSubtract(
            targetPosition,
            position);

    const float distanceSquared =
        XMVectorGetX(
            XMVector3LengthSq(
                difference));

    if (distanceSquared <= 0.000001f)
    {
        throw std::invalid_argument(
            "Camera position and target "
            "cannot be the same.");
    }

    const XMVECTOR direction =
        XMVector3Normalize(
            difference);

    XMFLOAT3 directionValues{};

    XMStoreFloat3(
        &directionValues,
        direction);

    yaw_ =
        std::atan2(
            directionValues.x,
            directionValues.z);

    pitch_ =
        std::asin(
            std::clamp(
                directionValues.y,
                -1.0f,
                1.0f));
}

void Camera::FrameBounds(
    const XMFLOAT3& minimum,
    const XMFLOAT3& maximum,
    float padding)
{
    if (minimum.x > maximum.x
        || minimum.y > maximum.y
        || minimum.z > maximum.z)
    {
        throw std::invalid_argument(
            "Cannot frame invalid bounds.");
    }

    if (padding < 1.0f)
    {
        throw std::invalid_argument(
            "Camera framing padding "
            "must be at least 1.");
    }

    const XMFLOAT3 center{
        minimum.x
            + (maximum.x - minimum.x) * 0.5f,

        minimum.y
            + (maximum.y - minimum.y) * 0.5f,

        minimum.z
            + (maximum.z - minimum.z) * 0.5f
    };

    const float halfWidth =
        (maximum.x - minimum.x) * 0.5f;

    const float halfHeight =
        (maximum.y - minimum.y) * 0.5f;

    const float halfDepth =
        (maximum.z - minimum.z) * 0.5f;

    float radius =
        std::sqrt(
            halfWidth * halfWidth
            + halfHeight * halfHeight
            + halfDepth * halfDepth);

    radius = std::max(
        radius,
        1.0f);

    const float halfVerticalFieldOfView =
        verticalFieldOfView_ * 0.5f;

    const float halfHorizontalFieldOfView =
        std::atan(
            std::tan(
                halfVerticalFieldOfView)
            * aspectRatio_);

    const float limitingHalfFieldOfView =
        std::min(
            halfVerticalFieldOfView,
            halfHorizontalFieldOfView);

    const float distance =
        padding
        * radius
        / std::sin(
            limitingHalfFieldOfView);

    // A slightly elevated diagonal view makes the shape of
    // terrain easier to understand than a straight top view.
    const XMVECTOR viewDirection =
        XMVector3Normalize(
            XMVectorSet(
                1.0f,
                -0.55f,
                1.0f,
                0.0f));

    const XMVECTOR centerVector =
        XMLoadFloat3(&center);

    const XMVECTOR position =
        XMVectorSubtract(
            centerVector,
            XMVectorScale(
                viewDirection,
                distance));

    XMStoreFloat3(
        &position_,
        position);

    // Adapt the depth range to the loaded dataset.
    nearPlane_ =
        std::max(
            radius * 0.001f,
            0.05f);

    farPlane_ =
        std::max(
            radius * 10.0f,
            distance + radius * 2.0f);

    LookAt(center);
}

XMMATRIX Camera::GetViewMatrix() const
{
    const XMVECTOR position =
        XMLoadFloat3(&position_);

    constexpr XMVECTORF32 worldUp{
        0.0f,
        1.0f,
        0.0f,
        0.0f
    };

    return XMMatrixLookToLH(
        position,
        GetForwardVector(),
        worldUp);
}

XMMATRIX Camera::GetProjectionMatrix() const
{
    return XMMatrixPerspectiveFovLH(
        verticalFieldOfView_,
        aspectRatio_,
        nearPlane_,
        farPlane_);
}

XMMATRIX
Camera::GetViewProjectionMatrix() const
{
    return GetViewMatrix()
        * GetProjectionMatrix();
}

const XMFLOAT3&
Camera::GetPosition() const
{
    return position_;
}
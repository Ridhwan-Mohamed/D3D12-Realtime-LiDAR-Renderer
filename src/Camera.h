#pragma once

#include <DirectXMath.h>

class Camera
{
public:
    explicit Camera(float aspectRatio);

    void SetAspectRatio(float aspectRatio);

    void SetPosition(
        const DirectX::XMFLOAT3& position);

    void Rotate(
        float yawDelta,
        float pitchDelta);

    void MoveLocal(
        float right,
        float up,
        float forward);

    void LookAt(
        const DirectX::XMFLOAT3& target);

    void FrameBounds(
        const DirectX::XMFLOAT3& minimum,
        const DirectX::XMFLOAT3& maximum,
        float padding = 1.15f);

    [[nodiscard]]
    DirectX::XMMATRIX GetViewMatrix() const;

    [[nodiscard]]
    DirectX::XMMATRIX GetProjectionMatrix() const;

    [[nodiscard]]
    DirectX::XMMATRIX
        GetViewProjectionMatrix() const;

    [[nodiscard]]
    const DirectX::XMFLOAT3&
        GetPosition() const;

private:
    [[nodiscard]]
    DirectX::XMVECTOR GetForwardVector() const;

    [[nodiscard]]
    DirectX::XMVECTOR GetRightVector() const;

    DirectX::XMFLOAT3 position_{
        0.0f,
        1.5f,
        -5.0f
    };

    float yaw_ = 0.0f;
    float pitch_ = 0.0f;

    float verticalFieldOfView_ =
        DirectX::XM_PI / 3.0f;

    float aspectRatio_ = 16.0f / 9.0f;
    float nearPlane_ = 0.1f;
    float farPlane_ = 10000.0f;
};
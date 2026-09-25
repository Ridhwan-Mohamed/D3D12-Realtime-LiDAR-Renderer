#pragma once

#include <DirectXMath.h>

#include <cstdint>

struct GpuPoint
{
    DirectX::XMFLOAT3 position;
    std::uint32_t color;
};

static_assert(
    sizeof(GpuPoint) == 16,
    "GpuPoint must remain 16 bytes");

constexpr std::uint32_t PackRgba8(
    std::uint8_t red,
    std::uint8_t green,
    std::uint8_t blue,
    std::uint8_t alpha = 255)
{
    return
        static_cast<std::uint32_t>(red)
        | (static_cast<std::uint32_t>(green) << 8)
        | (static_cast<std::uint32_t>(blue) << 16)
        | (static_cast<std::uint32_t>(alpha) << 24);
}
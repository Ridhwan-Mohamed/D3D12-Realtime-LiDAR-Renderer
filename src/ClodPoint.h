#pragma once

#include <DirectXMath.h>

#include <cstdint>

struct ClodPoint
{
    DirectX::XMFLOAT3 position{};

    // Bytes from least to most significant:
    // red, green, blue, hierarchy level.
    std::uint32_t colorAndLevel = 0;
};

static_assert(
    sizeof(ClodPoint) == 16,
    "ClodPoint must remain 16 bytes");

inline constexpr std::uint32_t
ClodRgbMask =
0x00FF'FFFFu;

inline constexpr std::uint32_t
ClodOpaqueAlpha =
0xFF00'0000u;

[[nodiscard]]
constexpr std::uint32_t
PackClodColorAndLevel(
    std::uint32_t packedRgba,
    std::uint8_t level) noexcept
{
    return
        (packedRgba & ClodRgbMask)
        | (static_cast<std::uint32_t>(level)
            << 24);
}

[[nodiscard]]
constexpr std::uint8_t
GetClodPointLevel(
    const ClodPoint& point) noexcept
{
    return static_cast<std::uint8_t>(
        point.colorAndLevel >> 24);
}

[[nodiscard]]
constexpr std::uint32_t
GetClodPointColor(
    const ClodPoint& point) noexcept
{
    return
        (point.colorAndLevel & ClodRgbMask)
        | ClodOpaqueAlpha;
}
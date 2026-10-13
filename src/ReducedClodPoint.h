#pragma once

#include <DirectXMath.h>

#include <cstdint>

struct ReducedClodPoint
{
    DirectX::XMFLOAT3 position{};

    std::uint32_t colorAndLevel = 0;

    // Exact randomized world-space spacing calculated
    // during CLOD reduction.
    float pointSpacing = 0.0f;
};

static_assert(
    sizeof(ReducedClodPoint) == 20,
    "ReducedClodPoint must remain 20 bytes.");
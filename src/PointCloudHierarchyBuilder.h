#pragma once

#include "PointCloudHierarchy.h"

#include <DirectXMath.h>

#include <cstdint>
#include <span>

struct PointHierarchyBuildSettings
{
    std::uint32_t maximumDepth = 8;

    std::uint32_t maximumPointsPerLeaf =
        50'000;

    std::uint32_t
        maximumRepresentativePointsPerNode =
        4'096;

    float minimumNodeExtent = 0.01f;
};

[[nodiscard]]
PointCloudHierarchy BuildPointCloudHierarchy(
    std::span<const GpuPoint> sourcePoints,
    const DirectX::XMFLOAT3& boundsMinimum,
    const DirectX::XMFLOAT3& boundsMaximum,
    const PointHierarchyBuildSettings& settings = {});
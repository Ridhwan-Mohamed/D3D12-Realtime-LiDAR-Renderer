#pragma once

#include "PointCloudHierarchy.h"

#include <DirectXMath.h>
#include <cstdint>
#include <span>
#include <stop_token>

struct PointHierarchyBuildSettings
{
    std::uint32_t maximumDepth = 8;

    std::uint32_t maximumPointsPerLeaf =
        50'000;

    std::uint32_t
        maximumRepresentativePointsPerNode =
        4'096;

    float minimumNodeExtent = 0.01f;

    // Potree's leaf store limit from the paper-era
    // converter. A CLOD node becomes an internal
    // sampling node once it reaches this count.
    std::uint32_t clodStoreLimit =
        20'000;

    // Safety limit for malformed data, coincident
    // points, and floating-point precision.
    std::uint32_t clodMaximumDepth =
        24;

    // A positive value uses an explicitly selected
    // world-space root spacing.
    //
    // Zero derives it from the cubic bounds.
    float clodRootSpacing =
        0.0f;

    // Potree's default:
    //
    // rootSpacing = cubic diagonal / 200
    float clodSpacingDiagonalFraction =
        200.0f;

};

[[nodiscard]]
PointCloudHierarchy
BuildDiscretePointCloudHierarchy(
    std::span<const GpuPoint> sourcePoints,
    const DirectX::XMFLOAT3& boundsMinimum,
    const DirectX::XMFLOAT3& boundsMaximum,
    const PointHierarchyBuildSettings& settings = {},
    std::stop_token cancellationToken = {});

void BuildClodRepresentation(
    PointCloudHierarchy& hierarchy,
    std::span<const GpuPoint> sourcePoints,
    const DirectX::XMFLOAT3& boundsMinimum,
    const DirectX::XMFLOAT3& boundsMaximum,
    const PointHierarchyBuildSettings& settings = {},
    std::stop_token cancellationToken = {});

[[nodiscard]]
PointCloudHierarchy BuildPointCloudHierarchy(
    std::span<const GpuPoint> sourcePoints,
    const DirectX::XMFLOAT3& boundsMinimum,
    const DirectX::XMFLOAT3& boundsMaximum,
    const PointHierarchyBuildSettings& settings = {},
    std::stop_token cancellationToken = {});
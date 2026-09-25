#pragma once

#include "PointCloudData.h"

#include <DirectXMath.h>

#include <cstdint>
#include <filesystem>
#include <span>
#include <vector>

struct PointCloudDatasetInfo
{
    std::vector<PointCloudMetadata>
        tileMetadata;

    Bounds3d sourceBounds;

    Double3 localOrigin;

    DirectX::XMFLOAT3 localBoundsMinimum{};
    DirectX::XMFLOAT3 localBoundsMaximum{};

    std::uint64_t totalPointCount = 0;
};

[[nodiscard]]
PointCloudDatasetInfo InspectPointCloudDataset(
    std::span<
    const std::filesystem::path>
    paths);
#pragma once

#include "PointCloudData.h"

#include <filesystem>

class LasPointCloudLoader
{
public:
    [[nodiscard]]
    static PointCloudMetadata ReadMetadata(
        const std::filesystem::path& path);

    [[nodiscard]]
    static PointCloudData Load(
        const std::filesystem::path& path);

    [[nodiscard]]
    static PointCloudData Load(
        const PointCloudMetadata& metadata,
        const Double3& localOrigin);
};
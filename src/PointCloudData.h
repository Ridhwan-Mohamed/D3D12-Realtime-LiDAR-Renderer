#pragma once

#include "GpuPoint.h"

#include <DirectXMath.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>
#include <array>

struct Double3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Bounds3d
{
    Double3 minimum;
    Double3 maximum;
};

struct PointAttributeAvailability
{
    bool hasIntensity = false;
    bool hasReturns = false;
    bool hasClassification = false;
    bool hasGpsTime = false;
    bool hasRgb = false;
    bool hasNearInfrared = false;
};

struct PointCloudMetadata
{
    std::filesystem::path sourcePath;

    std::uint8_t versionMajor = 0;
    std::uint8_t versionMinor = 0;

    std::uint8_t pointFormat = 0;
    std::uint16_t pointRecordLength = 0;

    std::uint64_t pointCount = 0;

    bool compressed = false;

    bool isCopc = false;

    Double3 scale;
    Double3 fileOffset;

    Bounds3d sourceBounds;

    PointAttributeAvailability attributes;

    std::string coordinateReferenceSystem;
};

struct PointCloudData
{
    PointCloudMetadata metadata;

    // World-space point used as the renderer's local (0, 0, 0).
    Double3 localOrigin;

    DirectX::XMFLOAT3 localBoundsMinimum{};
    DirectX::XMFLOAT3 localBoundsMaximum{};

    std::array<std::uint64_t, 256> classificationCounts{};

    std::vector<GpuPoint> points;
};

[[nodiscard]]
std::wstring FormatPointCloudMetadata(
    const PointCloudMetadata& metadata);

[[nodiscard]]
std::wstring FormatPointCloudData(
    const PointCloudData& data);
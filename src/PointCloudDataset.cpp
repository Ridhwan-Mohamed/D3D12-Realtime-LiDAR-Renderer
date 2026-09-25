#include "PointCloudDataset.h"

#include "LasPointCloudLoader.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
    float CheckedFloat(
        double value)
    {
        const double maximum =
            static_cast<double>(
                std::numeric_limits<float>::max());

        if (!std::isfinite(value)
            || value < -maximum
            || value > maximum)
        {
            throw std::runtime_error(
                "Dataset-local coordinate "
                "cannot be represented as a float.");
        }

        return static_cast<float>(value);
    }
}

PointCloudDatasetInfo InspectPointCloudDataset(
    std::span<
    const std::filesystem::path>
    paths)
{
    if (paths.empty())
    {
        throw std::invalid_argument(
            "A point-cloud dataset must "
            "contain at least one tile.");
    }

    PointCloudDatasetInfo dataset{};

    dataset.tileMetadata.reserve(
        paths.size());

    bool firstTile = true;

    for (const std::filesystem::path& path :
        paths)
    {
        PointCloudMetadata metadata =
            LasPointCloudLoader::ReadMetadata(
                path);

        if (!firstTile)
        {
            const std::string& expectedCrs =
                dataset.tileMetadata.front()
                .coordinateReferenceSystem;

            if (metadata.coordinateReferenceSystem
                != expectedCrs)
            {
                throw std::runtime_error(
                    "Point-cloud tiles do not "
                    "use the same coordinate system.");
            }

            dataset.sourceBounds.minimum.x =
                std::min(
                    dataset.sourceBounds.minimum.x,
                    metadata.sourceBounds.minimum.x);

            dataset.sourceBounds.minimum.y =
                std::min(
                    dataset.sourceBounds.minimum.y,
                    metadata.sourceBounds.minimum.y);

            dataset.sourceBounds.minimum.z =
                std::min(
                    dataset.sourceBounds.minimum.z,
                    metadata.sourceBounds.minimum.z);

            dataset.sourceBounds.maximum.x =
                std::max(
                    dataset.sourceBounds.maximum.x,
                    metadata.sourceBounds.maximum.x);

            dataset.sourceBounds.maximum.y =
                std::max(
                    dataset.sourceBounds.maximum.y,
                    metadata.sourceBounds.maximum.y);

            dataset.sourceBounds.maximum.z =
                std::max(
                    dataset.sourceBounds.maximum.z,
                    metadata.sourceBounds.maximum.z);
        }
        else
        {
            dataset.sourceBounds =
                metadata.sourceBounds;

            firstTile = false;
        }

        if (metadata.pointCount
            > std::numeric_limits<std::uint64_t>::max()
            - dataset.totalPointCount)
        {
            throw std::overflow_error(
                "Dataset point count overflow.");
        }

        dataset.totalPointCount +=
            metadata.pointCount;

        dataset.tileMetadata.push_back(
            std::move(metadata));
    }

    const Bounds3d& bounds =
        dataset.sourceBounds;

    // Centre the horizontal dimensions.
    // Place the dataset's lowest elevation at local Y = 0.
    dataset.localOrigin = {
        bounds.minimum.x
            + (bounds.maximum.x
                - bounds.minimum.x) * 0.5,

        bounds.minimum.y
            + (bounds.maximum.y
                - bounds.minimum.y) * 0.5,

        bounds.minimum.z
    };

    // Source Z becomes renderer Y because the renderer is Y-up.
    dataset.localBoundsMinimum = {
        CheckedFloat(
            bounds.minimum.x
            - dataset.localOrigin.x),

        CheckedFloat(
            bounds.minimum.z
            - dataset.localOrigin.z),

        CheckedFloat(
            bounds.minimum.y
            - dataset.localOrigin.y)
    };

    dataset.localBoundsMaximum = {
        CheckedFloat(
            bounds.maximum.x
            - dataset.localOrigin.x),

        CheckedFloat(
            bounds.maximum.z
            - dataset.localOrigin.z),

        CheckedFloat(
            bounds.maximum.y
            - dataset.localOrigin.y)
    };

    return dataset;
}
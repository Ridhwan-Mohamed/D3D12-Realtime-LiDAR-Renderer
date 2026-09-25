#include "LasPointCloudLoader.h"

#include <laszip_api.h>

#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string_view>

namespace
{
    [[noreturn]]
    void ThrowLaszipError(
        laszip_POINTER reader,
        const char* operation)
    {
        std::string message = operation;

        laszip_CHAR* laszipMessage = nullptr;

        if (reader != nullptr
            && laszip_get_error(
                reader,
                &laszipMessage) == 0
            && laszipMessage != nullptr)
        {
            message += ": ";
            message += laszipMessage;
        }

        throw std::runtime_error(message);
    }

    void CheckLaszip(
        laszip_POINTER reader,
        laszip_I32 result,
        const char* operation)
    {
        if (result != 0)
        {
            ThrowLaszipError(
                reader,
                operation);
        }
    }

    class LaszipReader
    {
    public:
        LaszipReader()
        {
            if (laszip_create(&handle_) != 0)
            {
                throw std::runtime_error(
                    "Could not create LASzip reader.");
            }
        }

        ~LaszipReader()
        {
            if (isOpen_)
            {
                laszip_close_reader(handle_);
            }

            if (handle_ != nullptr)
            {
                laszip_destroy(handle_);
            }
        }

        LaszipReader(const LaszipReader&) = delete;

        LaszipReader& operator=(
            const LaszipReader&) = delete;

        void Open(
            const std::filesystem::path& path)
        {
            const std::string narrowPath =
                path.string();

            laszip_BOOL compressed = 0;

            CheckLaszip(
                handle_,
                laszip_open_reader(
                    handle_,
                    narrowPath.c_str(),
                    &compressed),
                "Open LAS/LAZ reader");

            isOpen_ = true;
            isCompressed_ = compressed != 0;
        }

        [[nodiscard]]
        laszip_header* GetHeader() const
        {
            laszip_header* header = nullptr;

            CheckLaszip(
                handle_,
                laszip_get_header_pointer(
                    handle_,
                    &header),
                "Get LAS header");

            if (header == nullptr)
            {
                throw std::runtime_error(
                    "LASzip returned a null header.");
            }

            return header;
        }

        [[nodiscard]]
        bool IsCompressed() const
        {
            return isCompressed_;
        }

        [[nodiscard]]
        laszip_point* GetPoint() const
        {
            laszip_point* point = nullptr;

            CheckLaszip(
                handle_,
                laszip_get_point_pointer(
                    handle_,
                    &point),
                "Get LAS point pointer");

            if (point == nullptr)
            {
                throw std::runtime_error(
                    "LASzip returned a null point pointer.");
            }

            return point;
        }

        void ReadPoint()
        {
            CheckLaszip(
                handle_,
                laszip_read_point(handle_),
                "Read LAS point");
        }

    private:
        laszip_POINTER handle_ = nullptr;
        bool isOpen_ = false;
        bool isCompressed_ = false;
    };

    PointAttributeAvailability
        DetermineAttributes(
            std::uint8_t pointFormat)
    {
        PointAttributeAvailability attributes{};

        // These exist in every standard LAS point format.
        attributes.hasIntensity = true;
        attributes.hasReturns = true;
        attributes.hasClassification = true;

        attributes.hasGpsTime =
            pointFormat == 1
            || pointFormat == 3
            || pointFormat == 4
            || pointFormat == 5
            || pointFormat >= 6;

        attributes.hasRgb =
            pointFormat == 2
            || pointFormat == 3
            || pointFormat == 5
            || pointFormat == 7
            || pointFormat == 8
            || pointFormat == 10;

        attributes.hasNearInfrared =
            pointFormat == 8
            || pointFormat == 10;

        return attributes;
    }

    std::uint8_t GetClassification(
        const laszip_point& point,
        std::uint8_t pointFormat)
    {
        if (pointFormat >= 6)
        {
            return point.extended_classification;
        }

        return point.classification;
    }

    std::uint32_t GetClassificationColor(
        std::uint8_t classification)
    {
        switch (classification)
        {
        case 2: // Ground
            return PackRgba8(
                166, 124, 82);

        case 3: // Low vegetation
            return PackRgba8(
                130, 190, 90);

        case 4: // Medium vegetation
            return PackRgba8(
                70, 155, 70);

        case 5: // High vegetation
            return PackRgba8(
                30, 110, 50);

        case 6: // Building
            return PackRgba8(
                190, 190, 195);

        case 7: // Low noise
            return PackRgba8(
                220, 80, 220);

        case 9: // Water
            return PackRgba8(
                55, 125, 210);

        case 17: // Bridge deck
            return PackRgba8(
                225, 150, 55);

        case 18: // High noise
            return PackRgba8(
                230, 65, 65);

        case 0: // Never classified
        case 1: // Unclassified
        default:
            return PackRgba8(
                220, 220, 220);
        }
    }

    bool FixedStringEquals(
        const char* value,
        std::size_t capacity,
        std::string_view expected)
    {
        std::size_t length = 0;

        while (length < capacity
            && value[length] != '\0')
        {
            ++length;
        }

        return std::string_view(
            value,
            length) == expected;
    }

    bool IsCopc(
        const laszip_header& header)
    {
        if (header.vlrs == nullptr)
        {
            return false;
        }

        for (laszip_U32 index = 0;
            index < header.number_of_variable_length_records;
            ++index)
        {
            const laszip_vlr& vlr =
                header.vlrs[index];

            if (FixedStringEquals(
                vlr.user_id,
                LAS_VLR_USER_ID_CHAR_LEN,
                "copc")
                && vlr.record_id == 1)
            {
                return true;
            }
        }

        return false;
    }

    std::string ReadCoordinateSystem(
        const laszip_header& header)
    {
        if (header.vlrs == nullptr)
        {
            return {};
        }

        for (laszip_U32 index = 0;
            index < header.number_of_variable_length_records;
            ++index)
        {
            const laszip_vlr& vlr =
                header.vlrs[index];

            const bool isProjectionRecord =
                FixedStringEquals(
                    vlr.user_id,
                    LAS_VLR_USER_ID_CHAR_LEN,
                    "LASF_Projection");

            // Record 2112 contains coordinate-system WKT.
            if (!isProjectionRecord
                || vlr.record_id != 2112
                || vlr.data == nullptr
                || vlr.record_length_after_header == 0)
            {
                continue;
            }

            std::size_t textLength =
                vlr.record_length_after_header;

            while (textLength > 0
                && vlr.data[textLength - 1] == 0)
            {
                --textLength;
            }

            return std::string(
                reinterpret_cast<const char*>(
                    vlr.data),
                textLength);
        }

        return {};
    }

    void ValidateDecodedData(
        const PointCloudData& data)
    {
        if (data.points.size()
            != data.metadata.pointCount)
        {
            throw std::runtime_error(
                "Decoded point count does not match "
                "the LAS header.");
        }

        std::uint64_t classifiedPointCount = 0;

        for (const std::uint64_t count :
        data.classificationCounts)
        {
            classifiedPointCount += count;
        }

        if (classifiedPointCount
            != data.metadata.pointCount)
        {
            throw std::runtime_error(
                "Classification counts do not match "
                "the decoded point count.");
        }

        if (data.points.empty())
        {
            return;
        }

        const Bounds3d& source =
            data.metadata.sourceBounds;

        const DirectX::XMFLOAT3 expectedMinimum{
            static_cast<float>(
                source.minimum.x
                - data.localOrigin.x),

            static_cast<float>(
                source.minimum.z
                - data.localOrigin.z),

            static_cast<float>(
                source.minimum.y
                - data.localOrigin.y)
        };

        const DirectX::XMFLOAT3 expectedMaximum{
            static_cast<float>(
                source.maximum.x
                - data.localOrigin.x),

            static_cast<float>(
                source.maximum.z
                - data.localOrigin.z),

            static_cast<float>(
                source.maximum.y
                - data.localOrigin.y)
        };

        const double largestScale =
            std::max({
                data.metadata.scale.x,
                data.metadata.scale.y,
                data.metadata.scale.z
                });

        // Allow two source quantization steps plus a little
        // float-conversion tolerance.
        const float tolerance =
            static_cast<float>(
                largestScale * 2.0 + 0.001);

        const auto validateAxis =
            [tolerance](
                float actual,
                float expected,
                const char* description)
            {
                if (std::abs(actual - expected)
            > tolerance)
                {
                    throw std::runtime_error(
                        std::string(
                            "Decoded bounds do not match "
                            "the LAS header: ")
                        + description);
                }
            };

        validateAxis(
            data.localBoundsMinimum.x,
            expectedMinimum.x,
            "minimum X");

        validateAxis(
            data.localBoundsMinimum.y,
            expectedMinimum.y,
            "minimum Y");

        validateAxis(
            data.localBoundsMinimum.z,
            expectedMinimum.z,
            "minimum Z");

        validateAxis(
            data.localBoundsMaximum.x,
            expectedMaximum.x,
            "maximum X");

        validateAxis(
            data.localBoundsMaximum.y,
            expectedMaximum.y,
            "maximum Y");

        validateAxis(
            data.localBoundsMaximum.z,
            expectedMaximum.z,
            "maximum Z");
    }
}

PointCloudMetadata
LasPointCloudLoader::ReadMetadata(
    const std::filesystem::path& path)
{
    if (!std::filesystem::exists(path))
    {
        throw std::runtime_error(
            "Point cloud file does not exist: "
            + path.string());
    }

    if (!std::filesystem::is_regular_file(path))
    {
        throw std::runtime_error(
            "Point cloud path is not a regular file: "
            + path.string());
    }

    LaszipReader reader;
    reader.Open(path);

    const laszip_header* header =
        reader.GetHeader();

    PointCloudMetadata metadata{};

    metadata.sourcePath = path;
    metadata.compressed = reader.IsCompressed();

    metadata.versionMajor =
        header->version_major;

    metadata.versionMinor =
        header->version_minor;

    // The top two bits may contain LAZ compression flags.
    // The bottom six bits contain the actual LAS point format.
    metadata.pointFormat =
        static_cast<std::uint8_t>(
            header->point_data_format & 0x3f);

    metadata.pointRecordLength =
        header->point_data_record_length;

    if (header->extended_number_of_point_records != 0)
    {
        metadata.pointCount =
            header->
            extended_number_of_point_records;
    }
    else
    {
        metadata.pointCount =
            header->number_of_point_records;
    }

    metadata.scale = {
        header->x_scale_factor,
        header->y_scale_factor,
        header->z_scale_factor
    };

    metadata.fileOffset = {
        header->x_offset,
        header->y_offset,
        header->z_offset
    };

    metadata.sourceBounds.minimum = {
        header->min_x,
        header->min_y,
        header->min_z
    };

    metadata.sourceBounds.maximum = {
        header->max_x,
        header->max_y,
        header->max_z
    };

    if (metadata.pointFormat > 10)
    {
        throw std::runtime_error(
            "Unsupported LAS point format: "
            + std::to_string(
                metadata.pointFormat));
    }

    if (metadata.scale.x <= 0.0
        || metadata.scale.y <= 0.0
        || metadata.scale.z <= 0.0)
    {
        throw std::runtime_error(
            "LAS file contains an invalid coordinate scale.");
    }

    if (metadata.sourceBounds.minimum.x
            > metadata.sourceBounds.maximum.x
        || metadata.sourceBounds.minimum.y
            > metadata.sourceBounds.maximum.y
        || metadata.sourceBounds.minimum.z
            > metadata.sourceBounds.maximum.z)
    {
        throw std::runtime_error(
            "LAS file contains invalid bounds.");
    }

    metadata.attributes =
        DetermineAttributes(
            metadata.pointFormat);

    metadata.isCopc =
        IsCopc(*header);

    metadata.coordinateReferenceSystem =
        ReadCoordinateSystem(*header);

    return metadata;
}

PointCloudData
LasPointCloudLoader::Load(
    const std::filesystem::path& path)
{
    const PointCloudMetadata metadata =
        ReadMetadata(path);

    const Bounds3d& bounds =
        metadata.sourceBounds;

    const Double3 localOrigin{
        bounds.minimum.x
            + (bounds.maximum.x
                - bounds.minimum.x) * 0.5,

        bounds.minimum.y
            + (bounds.maximum.y
                - bounds.minimum.y) * 0.5,

        bounds.minimum.z
    };

    return Load(
        metadata,
        localOrigin);
}

PointCloudData
LasPointCloudLoader::Load(
    const PointCloudMetadata& metadata,
    const Double3& localOrigin)
{
    if (!std::isfinite(localOrigin.x)
        || !std::isfinite(localOrigin.y)
        || !std::isfinite(localOrigin.z))
    {
        throw std::invalid_argument(
            "Point-cloud local origin "
            "must be finite.");
    }

    PointCloudData data{};

    data.metadata = metadata;
    data.localOrigin = localOrigin;

    if (data.metadata.pointCount
        > std::numeric_limits<std::uint32_t>::max())
    {
        throw std::runtime_error(
            "The point cloud exceeds the current "
            "32-bit draw-count limit.");
    }

    LaszipReader reader;
    reader.Open(data.metadata.sourcePath);

    laszip_point* sourcePoint =
        reader.GetPoint();

    const Bounds3d& sourceBounds =
        data.metadata.sourceBounds;

    const std::size_t pointCount =
        static_cast<std::size_t>(
            data.metadata.pointCount);

    if (pointCount > data.points.max_size())
    {
        throw std::runtime_error(
            "The point cloud is too large for "
            "a CPU vector on this system.");
    }

    data.points.resize(pointCount);

    const float largestFloat =
        std::numeric_limits<float>::max();

    data.localBoundsMinimum = {
        largestFloat,
        largestFloat,
        largestFloat
    };

    data.localBoundsMaximum = {
        -largestFloat,
        -largestFloat,
        -largestFloat
    };

    for (std::size_t index = 0;
        index < pointCount;
        ++index)
    {
        reader.ReadPoint();

        // LAS stores integer XYZ values. The header's scale and
        // offset turn them into real geospatial coordinates.
        const double worldX =
            static_cast<double>(sourcePoint->X)
            * data.metadata.scale.x
            + data.metadata.fileOffset.x;

        const double worldY =
            static_cast<double>(sourcePoint->Y)
            * data.metadata.scale.y
            + data.metadata.fileOffset.y;

        const double worldZ =
            static_cast<double>(sourcePoint->Z)
            * data.metadata.scale.z
            + data.metadata.fileOffset.z;

        if (!std::isfinite(worldX)
            || !std::isfinite(worldY)
            || !std::isfinite(worldZ))
        {
            throw std::runtime_error(
                "Point cloud contains a non-finite coordinate "
                "at point "
                + std::to_string(index));
        }

        // Convert geospatial coordinates into the renderer's
        // local, Y-up coordinate system.
        const float localX =
            static_cast<float>(
                worldX - data.localOrigin.x);

        const float localY =
            static_cast<float>(
                worldZ - data.localOrigin.z);

        const float localZ =
            static_cast<float>(
                worldY - data.localOrigin.y);

        const std::uint8_t classification =
            GetClassification(
                *sourcePoint,
                data.metadata.pointFormat);

        ++data.classificationCounts[
            classification];

        data.points[index] = {
            {
                localX,
                localY,
                localZ
            },
            GetClassificationColor(
                classification)
        };

        data.localBoundsMinimum.x =
            std::min(
                data.localBoundsMinimum.x,
                localX);

        data.localBoundsMinimum.y =
            std::min(
                data.localBoundsMinimum.y,
                localY);

        data.localBoundsMinimum.z =
            std::min(
                data.localBoundsMinimum.z,
                localZ);

        data.localBoundsMaximum.x =
            std::max(
                data.localBoundsMaximum.x,
                localX);

        data.localBoundsMaximum.y =
            std::max(
                data.localBoundsMaximum.y,
                localY);

        data.localBoundsMaximum.z =
            std::max(
                data.localBoundsMaximum.z,
                localZ);
    }

    if (data.points.empty())
    {
        data.localBoundsMinimum = {};
        data.localBoundsMaximum = {};
    }

    ValidateDecodedData(data);

    return data;
}
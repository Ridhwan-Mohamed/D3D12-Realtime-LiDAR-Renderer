#include "PointCloudData.h"

#include <iomanip>
#include <sstream>

std::wstring FormatPointCloudMetadata(
    const PointCloudMetadata& metadata)
{
    std::wostringstream output;

    output
        << L"Source: "
        << metadata.sourcePath.wstring()
        << L'\n';

    output
        << L"Container: ";

    if (metadata.isCopc)
    {
        output << L"COPC / ";
    }

    output
        << (metadata.compressed
            ? L"LAZ, compressed"
            : L"LAS, uncompressed")
        << L'\n';

    output
        << L"LAS version: "
        << static_cast<unsigned>(
            metadata.versionMajor)
        << L'.'
        << static_cast<unsigned>(
            metadata.versionMinor)
        << L'\n';

    output
        << L"Point format: "
        << static_cast<unsigned>(
            metadata.pointFormat)
        << L'\n';

    output
        << L"Point record length: "
        << metadata.pointRecordLength
        << L" bytes\n";

    output
        << L"Point count: "
        << metadata.pointCount
        << L'\n';

    output
        << std::setprecision(12);

    output
        << L"Scale: "
        << metadata.scale.x << L", "
        << metadata.scale.y << L", "
        << metadata.scale.z
        << L'\n';

    output
        << L"File offset: "
        << metadata.fileOffset.x << L", "
        << metadata.fileOffset.y << L", "
        << metadata.fileOffset.z
        << L'\n';

    output
        << L"Minimum: "
        << metadata.sourceBounds.minimum.x << L", "
        << metadata.sourceBounds.minimum.y << L", "
        << metadata.sourceBounds.minimum.z
        << L'\n';

    output
        << L"Maximum: "
        << metadata.sourceBounds.maximum.x << L", "
        << metadata.sourceBounds.maximum.y << L", "
        << metadata.sourceBounds.maximum.z
        << L'\n';

    output
        << L"Attributes:"
        << L" intensity="
        << metadata.attributes.hasIntensity
        << L", returns="
        << metadata.attributes.hasReturns
        << L", classification="
        << metadata.attributes.hasClassification
        << L", GPS-time="
        << metadata.attributes.hasGpsTime
        << L", RGB="
        << metadata.attributes.hasRgb
        << L", NIR="
        << metadata.attributes.hasNearInfrared
        << L'\n';

    if (!metadata.coordinateReferenceSystem.empty())
    {
        output
            << L"Coordinate system:\n"
            << std::wstring(
                metadata.coordinateReferenceSystem.begin(),
                metadata.coordinateReferenceSystem.end())
            << L'\n';
    }

    return output.str();
}

std::wstring FormatPointCloudData(
    const PointCloudData& data)
{
    std::wostringstream output;

    output
        << FormatPointCloudMetadata(
            data.metadata);

    output
        << L"Local origin: "
        << data.localOrigin.x << L", "
        << data.localOrigin.y << L", "
        << data.localOrigin.z
        << L'\n';

    output
        << L"Local minimum: "
        << data.localBoundsMinimum.x << L", "
        << data.localBoundsMinimum.y << L", "
        << data.localBoundsMinimum.z
        << L'\n';

    output
        << L"Local maximum: "
        << data.localBoundsMaximum.x << L", "
        << data.localBoundsMaximum.y << L", "
        << data.localBoundsMaximum.z
        << L'\n';

    const double memoryMiB =
        static_cast<double>(
            data.points.size()
            * sizeof(GpuPoint))
        / (1024.0 * 1024.0);

    output
        << std::fixed
        << std::setprecision(2)
        << L"Prepared GPU data: "
        << memoryMiB
        << L" MiB\n";

    output
        << L"Classifications:\n";

    for (std::size_t classification = 0;
        classification
        < data.classificationCounts.size();
        ++classification)
    {
        const std::uint64_t count =
            data.classificationCounts[
                classification];

        if (count == 0)
        {
            continue;
        }

        output
            << L"  "
            << classification
            << L": "
            << count
            << L'\n';
    }

    return output.str();
}
#include "TileLoadWorker.h"

#include "LasPointCloudLoader.h"
#include "PointCloudHierarchyBuilder.h"

#include <stdexcept>
#include <utility>
#include <algorithm>

TileLoadWorker::TileLoadWorker(
    std::size_t maximumPendingRequests)
    : maximumPendingRequests_(
        maximumPendingRequests)
{
    if (maximumPendingRequests_ == 0)
    {
        throw std::invalid_argument(
            "Tile load worker queue capacity "
            "must be greater than zero.");
    }
}

TileLoadWorker::~TileLoadWorker()
{
    if (!workerThread_.joinable())
    {
        return;
    }

    {
        std::lock_guard lock(mutex_);

        acceptingRequests_ = false;
    }

    workerThread_.request_stop();

    requestAvailable_.notify_all();
}

void TileLoadWorker::Start()
{
    {
        std::lock_guard lock(mutex_);

        if (acceptingRequests_
            || workerThread_.joinable())
        {
            throw std::logic_error(
                "Tile load worker is already running.");
        }

        acceptingRequests_ = true;
    }

    try
    {
        workerThread_ =
            std::jthread(
                [this](
                    std::stop_token stopToken)
                {
                    WorkerMain(stopToken);
                });
    }
    catch (...)
    {
        std::lock_guard lock(mutex_);

        acceptingRequests_ = false;

        throw;
    }
}

void TileLoadWorker::Stop()
{
    {
        std::lock_guard lock(mutex_);

        acceptingRequests_ = false;
    }

    if (workerThread_.joinable())
    {
        workerThread_.request_stop();

        requestAvailable_.notify_all();

        workerThread_.join();
    }

    {
        std::lock_guard lock(mutex_);

        pendingRequests_.clear();
        pendingClodBuildRequests_.clear();

        startedEvents_.clear();

        completedResults_.clear();
        completedClodBuildResults_.clear();
    }
}

bool TileLoadWorker::TryEnqueue(
    TileLoadRequest request)
{
    {
        std::lock_guard lock(mutex_);

        const std::size_t pendingWorkCount =
            pendingRequests_.size()
            + pendingClodBuildRequests_.size();

        if (!acceptingRequests_
            || pendingWorkCount
            >= maximumPendingRequests_)
        {
            return false;
        }

        pendingRequests_.push_back(
            std::move(request));
    }

    requestAvailable_.notify_one();

    return true;
}

bool TileLoadWorker::TryEnqueueClodBuild(
    ClodBuildRequest& request)
{
    {
        std::lock_guard lock(mutex_);

        const std::size_t pendingWorkCount =
            pendingRequests_.size()
            + pendingClodBuildRequests_.size();

        if (!acceptingRequests_
            || pendingWorkCount
            >= maximumPendingRequests_)
        {
            return false;
        }

        pendingClodBuildRequests_.push_back(
            std::move(request));
    }

    requestAvailable_.notify_one();

    return true;
}

bool TileLoadWorker::TryCancelPendingClodBuild(
    std::size_t tileStateIndex,
    std::uint64_t requestGeneration)
{
    std::lock_guard lock(mutex_);

    const auto requestIterator =
        std::find_if(
            pendingClodBuildRequests_.begin(),
            pendingClodBuildRequests_.end(),
            [tileStateIndex, requestGeneration](
                const ClodBuildRequest& request)
            {
                return
                    request.tileStateIndex
                    == tileStateIndex
                    && request.requestGeneration
                    == requestGeneration;
            });

    if (requestIterator
        == pendingClodBuildRequests_.end())
    {
        return false;
    }

    pendingClodBuildRequests_.erase(
        requestIterator);

    return true;
}

std::optional<ClodBuildResult>
TileLoadWorker::TryTakeCompletedClodBuild()
{
    std::lock_guard lock(mutex_);

    if (completedClodBuildResults_.empty())
    {
        return std::nullopt;
    }

    ClodBuildResult result =
        std::move(
            completedClodBuildResults_.front());

    completedClodBuildResults_.pop_front();

    return std::optional<ClodBuildResult>{
        std::move(result)
    };
}

bool TileLoadWorker::TryCancelPending(
    std::size_t tileStateIndex,
    std::uint64_t requestGeneration)
{
    std::lock_guard lock(mutex_);

    const auto requestIterator =
        std::find_if(
            pendingRequests_.begin(),
            pendingRequests_.end(),
            [tileStateIndex, requestGeneration](
                const TileLoadRequest& request)
            {
                return
                    request.tileStateIndex
                    == tileStateIndex
                    && request.requestGeneration
                    == requestGeneration;
            });

    if (requestIterator
        == pendingRequests_.end())
    {
        return false;
    }

    pendingRequests_.erase(
        requestIterator);

    return true;
}

std::optional<TileLoadStarted>
TileLoadWorker::TryTakeStarted()
{
    std::lock_guard lock(mutex_);

    if (startedEvents_.empty())
    {
        return std::nullopt;
    }

    TileLoadStarted started =
        std::move(
            startedEvents_.front());

    startedEvents_.pop_front();

    return std::optional<TileLoadStarted>{
        std::move(started)
    };
}

std::optional<TileLoadResult>
TileLoadWorker::TryTakeCompleted()
{
    std::lock_guard lock(mutex_);

    if (completedResults_.empty())
    {
        return std::nullopt;
    }

    TileLoadResult result =
        std::move(
            completedResults_.front());

    completedResults_.pop_front();

    return std::optional<TileLoadResult>{
        std::move(result)
    };
}

std::size_t
TileLoadWorker::GetPendingRequestCount() const
{
    std::lock_guard lock(mutex_);

    return
        pendingRequests_.size()
        + pendingClodBuildRequests_.size();
}

void TileLoadWorker::WorkerMain(
    std::stop_token stopToken)
{
    while (!stopToken.stop_requested())
    {
        std::optional<TileLoadRequest>
            tileLoadRequest;

        std::optional<ClodBuildRequest>
            clodBuildRequest;

        {
            std::unique_lock lock(mutex_);

            const bool requestReady =
                requestAvailable_.wait(
                    lock,
                    stopToken,
                    [this]()
                    {
                        return
                            !pendingRequests_.empty()
                            || !pendingClodBuildRequests_.
                            empty();
                    });

            if (!requestReady
                || stopToken.stop_requested())
            {
                return;
            }

            // Always prioritize loading discrete data.
            //
            // This ensures a CLOD build cannot prevent a
            // newly visible tile from getting its basic
            // representation.
            if (!pendingRequests_.empty())
            {
                tileLoadRequest.emplace(
                    std::move(
                        pendingRequests_.front()));

                pendingRequests_.pop_front();

                startedEvents_.push_back({
                    tileLoadRequest->tileStateIndex,
                    tileLoadRequest->requestGeneration
                    });
            }
            else
            {
                clodBuildRequest.emplace(
                    std::move(
                        pendingClodBuildRequests_.front()));

                pendingClodBuildRequests_.pop_front();
            }
        }

        if (tileLoadRequest.has_value())
        {
            std::stop_callback cancelOnWorkerStop(
                stopToken,
                [&tileLoadRequest]() noexcept
                {
                    tileLoadRequest->
                        cancellationSource.
                        request_stop();
                });

            TileLoadResult result =
                ProcessRequest(
                    *tileLoadRequest);

            if (stopToken.stop_requested())
            {
                return;
            }

            {
                std::lock_guard lock(mutex_);

                completedResults_.push_back(
                    std::move(result));
            }

            continue;
        }

        if (!clodBuildRequest.has_value())
        {
            throw std::logic_error(
                "Tile worker woke without a request.");
        }

        std::stop_callback cancelOnWorkerStop(
            stopToken,
            [&clodBuildRequest]() noexcept
            {
                clodBuildRequest->
                    cancellationSource.
                    request_stop();
            });

        ClodBuildResult result =
            ProcessClodBuild(
                std::move(
                    *clodBuildRequest));

        if (stopToken.stop_requested())
        {
            return;
        }

        {
            std::lock_guard lock(mutex_);

            completedClodBuildResults_.push_back(
                std::move(result));
        }
    }
}

TileLoadResult
TileLoadWorker::ProcessRequest(
    const TileLoadRequest& request)
{
    TileLoadResult result{};

    const std::stop_token cancellationToken =
        request.cancellationSource.get_token();

    result.tileStateIndex =
        request.tileStateIndex;

    result.requestGeneration =
        request.requestGeneration;

    try
    {
        PointCloudData tile =
            LasPointCloudLoader::Load(
                request.metadata,
                request.localOrigin,
                cancellationToken);

        if (cancellationToken.stop_requested())
        {
            throw std::runtime_error(
                "Point-cloud hierarchy build was cancelled.");
        }

        PointCloudHierarchy hierarchy =
            BuildDiscretePointCloudHierarchy(
                tile.points,
                tile.localBoundsMinimum,
                tile.localBoundsMaximum,
                PointHierarchyBuildSettings{},
                cancellationToken);

        result.hierarchy =
            std::move(hierarchy);
    }
    catch (...)
    {
        result.failure =
            std::current_exception();
    }

    return result;
}

ClodBuildResult
TileLoadWorker::ProcessClodBuild(
    ClodBuildRequest request)
{
    ClodBuildResult result{};

    result.tileStateIndex =
        request.tileStateIndex;

    result.requestGeneration =
        request.requestGeneration;

    result.sourcePointCount =
        request.sourcePointCount;

    // Preserve ownership even when CLOD construction fails.
    // This lets the application recover or still use the
    // discrete representation.
    result.discretePoints =
        std::move(
            request.discretePoints);

    try
    {
        if (result.sourcePointCount == 0)
        {
            throw std::invalid_argument(
                "CLOD build has no source points.");
        }

        if (result.sourcePointCount
            > result.discretePoints.size())
        {
            throw std::invalid_argument(
                "CLOD source count exceeds the "
                "packed discrete array.");
        }

        const std::span<const GpuPoint>
            sourcePoints{
                result.discretePoints.data(),
                static_cast<std::size_t>(
                    result.sourcePointCount)
        };

        PointCloudHierarchy clodPayload{};

        clodPayload.sourcePointCount =
            result.sourcePointCount;

        BuildClodRepresentation(
            clodPayload,
            sourcePoints,
            request.boundsMinimum,
            request.boundsMaximum,
            PointHierarchyBuildSettings{},
            request.cancellationSource.get_token());

        result.clodPoints =
            std::move(
                clodPayload.clodPoints);

        result.clodLevelRanges =
            std::move(
                clodPayload.clodLevelRanges);

        result.clodRootSpacing =
            clodPayload.clodRootSpacing;
    }
    catch (...)
    {
        result.failure =
            std::current_exception();
    }

    return result;
}

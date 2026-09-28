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
        startedEvents_.clear();
        completedResults_.clear();
    }
}

bool TileLoadWorker::TryEnqueue(
    TileLoadRequest request)
{
    {
        std::lock_guard lock(mutex_);

        if (!acceptingRequests_
            || pendingRequests_.size()
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

    return pendingRequests_.size();
}

void TileLoadWorker::WorkerMain(
    std::stop_token stopToken)
{
    while (!stopToken.stop_requested())
    {
        TileLoadRequest request;

        {
            std::unique_lock lock(mutex_);

            const bool requestReady =
                requestAvailable_.wait(
                    lock,
                    stopToken,
                    [this]()
                    {
                        return
                            !pendingRequests_.empty();
                    });

            if (!requestReady
                || stopToken.stop_requested())
            {
                return;
            }

            request =
                std::move(
                    pendingRequests_.front());

            pendingRequests_.pop_front();

            startedEvents_.push_back({
                request.tileStateIndex,
                request.requestGeneration
                });
        }

        std::stop_callback cancelRequestOnWorkerStop(
            stopToken,
            [&request]() noexcept
            {
                request.cancellationSource.
                    request_stop();
            });

        TileLoadResult result =
            ProcessRequest(
                request);

        if (stopToken.stop_requested())
        {
            return;
        }

        {
            std::lock_guard lock(mutex_);

            completedResults_.push_back(
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
            BuildPointCloudHierarchy(
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
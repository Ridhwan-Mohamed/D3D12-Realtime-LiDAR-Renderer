#pragma once

#include "PointCloudData.h"
#include "PointCloudHierarchy.h"

#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <exception>
#include <mutex>
#include <optional>
#include <thread>
#include <stop_token>

struct TileLoadRequest
{
    std::size_t tileStateIndex = 0;

    std::uint64_t requestGeneration = 0;

    PointCloudMetadata metadata;

    Double3 localOrigin;

    std::stop_source cancellationSource;
};

struct TileLoadStarted
{
    std::size_t tileStateIndex = 0;

    std::uint64_t requestGeneration = 0;
};

struct TileLoadResult
{
    std::size_t tileStateIndex = 0;

    std::uint64_t requestGeneration = 0;

    PointCloudHierarchy hierarchy;

    std::exception_ptr failure;

    [[nodiscard]]
    bool Succeeded() const noexcept
    {
        return failure == nullptr;
    }
};

class TileLoadWorker final
{
public:
    explicit TileLoadWorker(
        std::size_t maximumPendingRequests = 2);

    ~TileLoadWorker();

    TileLoadWorker(
        const TileLoadWorker&) = delete;

    TileLoadWorker& operator=(
        const TileLoadWorker&) = delete;

    TileLoadWorker(
        TileLoadWorker&&) = delete;

    TileLoadWorker& operator=(
        TileLoadWorker&&) = delete;

    void Start();

    void Stop();

    [[nodiscard]]
    bool TryEnqueue(
        TileLoadRequest request);

    [[nodiscard]]
    bool TryCancelPending(
        std::size_t tileStateIndex,
        std::uint64_t requestGeneration);

    [[nodiscard]]
    std::optional<TileLoadStarted>
        TryTakeStarted();

    [[nodiscard]]
    std::optional<TileLoadResult>
        TryTakeCompleted();

    [[nodiscard]]
    std::size_t GetPendingRequestCount() const;

private:
    void WorkerMain(
        std::stop_token stopToken);

    [[nodiscard]]
    static TileLoadResult ProcessRequest(
        const TileLoadRequest& request);

    const std::size_t
        maximumPendingRequests_;

    mutable std::mutex mutex_;

    std::condition_variable_any
        requestAvailable_;

    std::deque<TileLoadRequest>
        pendingRequests_;

    std::deque<TileLoadStarted>
        startedEvents_;

    std::deque<TileLoadResult>
        completedResults_;

    bool acceptingRequests_ = false;

    std::jthread workerThread_;
};
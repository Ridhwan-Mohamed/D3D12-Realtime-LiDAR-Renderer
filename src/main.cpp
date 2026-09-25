#include "Application.h"

#include <Windows.h>

#include <cstdlib>
#include <exception>
#include <string>
#include <filesystem>
#include <vector>
#include <shellapi.h>

namespace
{
    std::vector<std::filesystem::path>
        GetPointCloudPaths()
    {
        int argumentCount = 0;

        LPWSTR* arguments =
            CommandLineToArgvW(
                GetCommandLineW(),
                &argumentCount);

        if (arguments == nullptr)
        {
            throw std::runtime_error(
                "Could not parse command-line arguments.");
        }

        std::vector<std::filesystem::path>
            pointCloudPaths;

        try
        {
            if (argumentCount > 1)
            {
                pointCloudPaths.reserve(
                    static_cast<std::size_t>(
                        argumentCount - 1));
            }

            for (int index = 1;
                index < argumentCount;
                ++index)
            {
                pointCloudPaths.emplace_back(
                    arguments[index]);
            }
        }
        catch (...)
        {
            LocalFree(arguments);
            throw;
        }

        LocalFree(arguments);

        return pointCloudPaths;
    }
}

int WINAPI wWinMain(
    HINSTANCE instance,
    HINSTANCE,
    PWSTR,
    int showCommand)
{
    try
    {
        Application application(
            instance,
            GetPointCloudPaths());
        return application.Run(showCommand);
    }
    catch (const std::exception& error)
    {
        const std::string message =
            std::string(
                "Point Cloud Renderer failed:\n\n")
            + error.what();

        MessageBoxA(
            nullptr,
            message.c_str(),
            "Fatal Error",
            MB_OK | MB_ICONERROR);

        return EXIT_FAILURE;
    }
}
#pragma once

#include <Windows.h>

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

inline void ThrowIfFailed(
    HRESULT result,
    const char* operation)
{
    if (SUCCEEDED(result))
    {
        return;
    }

    std::ostringstream message;

    message
        << operation
        << " failed with HRESULT 0x"
        << std::hex
        << std::uppercase
        << static_cast<std::uint32_t>(result);

    throw std::runtime_error(message.str());
}
#pragma once

#include <cstdlib>

enum class PageState
{
    unknown = 0,
    visited,
    requestHighResolution,
    requestLowResolution,
};

struct PageEntry
{
    int dataIndex;
    
};
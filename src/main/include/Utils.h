
#pragma once

#include <iostream>

#define OKC_CALL(res)                                                          \
    if (!(res))                                                                \
    {                                                                          \
        std::cerr << "OKC_CALL FAIL [" << __FILE__ << "] line " << __LINE__    \
                  << std::endl;                                                \
        return false;                                                          \
    }

#define OKC_CHECK(check)                                                       \
    if (!(check))                                                              \
    {                                                                          \
        std::cerr << "OKC_CHECK FAIL [" << __FILE__ << "] line " << __LINE__   \
                  << std::endl;                                                \
        return false;                                                          \
    }

#define OKC_CHECK_MSG(check, msg)                                              \
    if (!(check))                                                              \
    {                                                                          \
        std::cerr << "OKC_CHECK FAIL [" << __FILE__ << "] line " << __LINE__   \
                  << ": " << msg << std::endl;                                 \
        return false;                                                          \
    }

#define VOKC_CALL(res)                                                         \
    if (!(res))                                                                \
    {                                                                          \
        std::cerr << "OKC_CALL FAIL [" << __FILE__ << "] line " << __LINE__    \
                  << std::endl;                                                \
        return;                                                                \
    }

#define VOKC_CHECK(check)                                                      \
    if (!(check))                                                              \
    {                                                                          \
        std::cerr << "OKC_CHECK FAIL [" << __FILE__ << "] line " << __LINE__   \
                  << std::endl;                                                \
        return;                                                                \
    }

#define VOKC_CHECK_MSG(check, msg)                                             \
    if (!(check))                                                              \
    {                                                                          \
        std::cerr << "OKC_CHECK FAIL [" << __FILE__ << "] line " << __LINE__   \
                  << ": " << msg << std::endl;                                 \
        return;                                                                \
    }
namespace TeamOKC
{

    bool Clamp(const double &lower, const double &upper, double *value);

}
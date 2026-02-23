// SERAPHINE SOFTWARE 2026

#ifndef LOVE_MATH_WRAP_STRING_H
#define LOVE_MATH_WRAP_STRING_H

// LOVE
#include "common/config.h"
#include "common/runtime.h"

namespace love
{
namespace string
{

extern "C" LOVE_EXPORT int luaopen_love_string(lua_State *L);

} // string
} // love

#endif // LOVE_MATH_WRAP_STRING_H


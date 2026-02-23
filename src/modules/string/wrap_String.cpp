// SERAPHINE SOFTWARE 2026

#include "wrap_String.h"
#include "StringModule.h"

#include <string>
#include <vector>
#include <regex>
#include <cctype>

namespace love
{
namespace string
{

#define instance() (Module::getInstance<StringModule>(Module::M_STRING))

static const char *memfind(const char *s1, size_t l1, const char *s2, size_t l2)
{
	if (l2 == 0) return s1;
	while (l1 >= l2)
	{
		if (memcmp(s1, s2, l2) == 0)
			return s1;
		s1++; l1--;
	}
	return NULL;
}


int w_replace(lua_State *L)
{
	size_t l1, l2, l3;
	const char *src = luaL_checklstring(L, 1, &l1);
	const char *p = luaL_checklstring(L, 2, &l2);
	const char *p2 = luaL_checklstring(L, 3, &l3);
	const char *s2;
	int n = 0;
	int init = 0;

	luaL_Buffer b;
	luaL_buffinit(L, &b);

	while (1)
	{
		s2 = memfind(src + init, l1 - init, p, l2);
		if (s2)
		{
			luaL_addlstring(&b, src + init, s2 - (src + init));
			luaL_addlstring(&b, p2, l3);
			init = init + (s2 - (src + init)) + l2;
			n++;
		}
		else
		{
			luaL_addlstring(&b, src + init, l1 - init);
			break;
		}
	}

	luaL_pushresult(&b);
	lua_pushnumber(L, (lua_Number)n);  /* number of substitutions */
	return 2;
}

bool IsHex(unsigned char c)
{
	return std::isxdigit(c) != 0;
}

int GetHexValue(unsigned char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	c = (unsigned char)std::tolower(c);
	if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
	return -1;
}

int HexToBytes(const std::string &s, size_t i)
{
	int hi = GetHexValue((unsigned char)s[i]);
	int lo = GetHexValue((unsigned char)s[i + 1]);
	return (hi << 4) | lo;
}

void PushColourTableFromHex(const std::string &hexWithHash, lua_State *L)
{
	std::string h = hexWithHash;
	if (!h.empty() && h[0] == '#') h.erase(0, 1);

	float r = HexToBytes(h, 0) / 255.0f;
	float g = HexToBytes(h, 2) / 255.0f;
	float b = HexToBytes(h, 4) / 255.0f;
	float a = (h.size() == 8) ? (HexToBytes(h, 6) / 255.0f) : 1.0f;

	lua_createtable(L, 4, 0);
	lua_pushnumber(L, r); lua_rawseti(L, -2, 1);
	lua_pushnumber(L, g); lua_rawseti(L, -2, 2);
	lua_pushnumber(L, b); lua_rawseti(L, -2, 3);
	lua_pushnumber(L, a); lua_rawseti(L, -2, 4);
}

void PushDefaultColourTable(lua_State *L)
{
	lua_createtable(L, 4, 0);
	lua_pushnumber(L, 1.0); lua_rawseti(L, -2, 1);
	lua_pushnumber(L, 1.0); lua_rawseti(L, -2, 2);
	lua_pushnumber(L, 1.0); lua_rawseti(L, -2, 3);
	lua_pushnumber(L, 1.0); lua_rawseti(L, -2, 4);
}

int w_makeColouredText(lua_State *L)
{
	size_t textLen = 0;
	const char *textC = luaL_checklstring(L, 1, &textLen);
	std::string text(textC, textLen);

	const std::string openPrefix = "<colour:";
	const std::string closeTag = "</colour>";
	bool foundAnyTag = false;
	lua_Integer outIndex = 1;

	lua_newtable(L);
	int retTableIndex = lua_gettop(L);

	size_t cursor = 0;
	while (true)
	{
		size_t startIdx = text.find(openPrefix, cursor);
		if (startIdx == std::string::npos)
		{
			if (!foundAnyTag)
			{
				lua_settop(L, 0);
				lua_pushlstring(L, text.data(), text.size());
				return 1;
			}

			if (cursor < text.size())
			{
				PushDefaultColourTable(L);
				lua_rawseti(L, retTableIndex, outIndex++);

				std::string tail = text.substr(cursor);
				lua_pushlstring(L, tail.data(), tail.size());
				lua_rawseti(L, retTableIndex, outIndex++);
			}
			break;
		}

		size_t p = startIdx + openPrefix.size();
		while (p < text.size() && std::isspace((unsigned char)text[p])) p++;

		if (p >= text.size() || text[p] != '#')
		{
			cursor = startIdx + 1;
			continue;
		}

		size_t hexStart = p;
		p++;
		size_t digitsStart = p;
		size_t digitCount = 0;
		while (p < text.size() && IsHex((unsigned char)text[p]) && digitCount < 8)
		{
			p++;
			digitCount++;
		}

		if (!(digitCount == 6 || digitCount == 8))
		{
			cursor = startIdx + 1;
			continue;
		}

		while (p < text.size() && std::isspace((unsigned char)text[p])) p++;
		if (p >= text.size() || text[p] != '>')
		{
			cursor = startIdx + 1;
			continue;
		}

		size_t endIdx = p;
		size_t afterOpen = endIdx + 1;

		foundAnyTag = true;
		if (startIdx > cursor)
		{
			std::string before = text.substr(cursor, startIdx - cursor);

			PushDefaultColourTable(L);
			lua_rawseti(L, retTableIndex, outIndex++);

			lua_pushlstring(L, before.data(), before.size());
			lua_rawseti(L, retTableIndex, outIndex++);
		}

		size_t closeStart = text.find(closeTag, afterOpen);
		if (closeStart == std::string::npos)
		{
			std::string msg = "textManager:MakeColouredText: missing a closing tag in string: \" " + text + "\"";
			return luaL_error(L, "%s", msg.c_str());
		}

		std::string colouredText = text.substr(afterOpen, closeStart - afterOpen);
		std::string hex = text.substr(hexStart, 1 + digitCount);
		PushColourTableFromHex(hex, L);
		lua_rawseti(L, retTableIndex, outIndex++);
		lua_pushlstring(L, colouredText.data(), colouredText.size());
		lua_rawseti(L, retTableIndex, outIndex++);

		cursor = closeStart + closeTag.size();
	}

	return 1;
}

// List of functions to wrap.
static const luaL_Reg functions[] =
{
	{ "replace", w_replace },
	{ "makeColouredText", w_makeColouredText },
	{ 0, 0 }
};

extern "C" int luaopen_love_string(lua_State *L)
{
	StringModule *inst = instance();
	if (inst == nullptr)
	{
		luax_catchexcept(L, [&](){ inst = new StringModule(); });
	}
	else
	{
		inst->retain();
	}

	WrappedModule w;
	w.module = inst;
	w.name = "string";
	w.type = &Module::type;
	w.functions = functions;
	w.types = 0;

	return luax_register_module(L, w);
}

} // math
} // love

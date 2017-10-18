#pragma once
#include <cstdlib>
#include <string>

wchar_t * ctow(const char* c, wchar_t * w)
{
	size_t len = strlen(c) + 1;
	size_t converted = 0;
	if (w != NULL)free(w);
	w = (wchar_t *)malloc(len*sizeof(wchar_t));
	mbstowcs_s(&converted, w, len, c, _TRUNCATE);
	return w;
}

char * wtoc(const wchar_t* w, char * c)
{
	size_t len = wcslen(w) + 1;
	size_t converted = 0;
	if (c != NULL)free(c);
	c = (char *)malloc(len*sizeof(char));
	wcstombs_s(&converted, c, len, w, _TRUNCATE);
	return c;
}
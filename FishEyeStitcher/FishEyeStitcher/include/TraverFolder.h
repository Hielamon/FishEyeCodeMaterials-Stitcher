#pragma once
#include <iostream>
#include <vector>


#if defined(WIN32) || defined(_WIN32)
#include "char_and_wchar.h"
#include <windows.h>
#endif // WIN32

#ifdef linux
#include <unistd.h>
#include <dirent.h>
#endif // linux


class TraverFolder
{
public:
	TraverFolder();
	TraverFolder(const std::string &folder_path);
	void setFolderPath(const std::string &folder_path);
	void getFileFullPath(std::vector<std::string> &filelist, const std::string& suffix);
	bool IsFind();
	~TraverFolder();

private:
	bool _getNextFileName(char * &filename);

#if defined(WIN32) || defined(_WIN32)
	WIN32_FIND_DATA FindData;
	HANDLE hError;
#endif // WIN32

#ifdef linux
	DIR *dir;
	struct dirent *ptr;
#endif // linux

	std::string folder_path_;
};

TraverFolder::TraverFolder()
{
#if defined(WIN32) || defined(_WIN32)
	hError = INVALID_HANDLE_VALUE;
#endif // WIN32

#ifdef linux
	dir = NULL;
#endif // linux
}

TraverFolder::TraverFolder(const std::string &folder_path)
{
#if defined(WIN32) || defined(_WIN32)
	char end_char = folder_path[folder_path.size() - 1];
	if (end_char != '\\' && end_char != '\/')
	{
		folder_path_ = folder_path + '\\';
	}
	else folder_path_ = folder_path;
	std::string fp_for_search = folder_path_ + "*";
#ifdef UNICODE
	wchar_t *wfp_for_search_ = NULL;
	wfp_for_search_ = ctow(fp_for_search.c_str(), wfp_for_search_);
	hError = FindFirstFile(wfp_for_search_, &FindData);
#else
	hError = FindFirstFile(fp_for_search.c_str(), &FindData);
#endif // UNICODE
#endif // WIN32

#ifdef linux
	folder_path_ = folder_path;
	dir = opendir(folder_path_.c_str());
#endif // linux
}

void TraverFolder::setFolderPath(const std::string &folder_path)
{
#if defined(WIN32) || defined(_WIN32)
	char end_char = folder_path[folder_path.size() - 1];
	if (end_char != '\\' && end_char != '\/')
	{
		folder_path_ = folder_path + '\\';
	}
	else folder_path_ = folder_path;
	std::string fp_for_search = folder_path_ + "*";
#ifdef UNICODE
	wchar_t *wfp_for_search_ = NULL;
	wfp_for_search_ = ctow(fp_for_search.c_str(), wfp_for_search_);
	hError = FindFirstFile(wfp_for_search_, &FindData);
#else
	hError = FindFirstFile(fp_for_search.c_str(), &FindData);
#endif // UNICODE
#endif // WIN32

#ifdef linux
	folder_path_ = folder_path + '/';
	dir = opendir(folder_path_.c_str());
#endif // linux
}

void TraverFolder::getFileFullPath(std::vector<std::string> &filelist, const std::string& suffix = "")
{
	if (IsFind())
	{
		char *FileName = NULL;
		while (_getNextFileName(FileName))
		{
			// ¹ýÂÇ.ºÍ..
			if (strcmp(FileName, ".") == 0
				|| strcmp(FileName, "..") == 0)
			{
				continue;
			}
			std::string filefullpath = folder_path_ + FileName;
			if (suffix == "")filelist.push_back(filefullpath);
			else
			{
				int suffixpos = filefullpath.rfind('.');
				std::string cuttail = filefullpath.substr(suffixpos+1, filefullpath.size() - 1 - suffixpos);
				if (cuttail == suffix)filelist.push_back(filefullpath);
			}
		}
	}
}

bool TraverFolder::IsFind()
{
#if defined(WIN32) || defined(_WIN32)
	return hError == INVALID_HANDLE_VALUE ? false : true;
#endif // WIN32

#ifdef linux
	return dir == NULL ? false : true;
#endif // linux
}

bool TraverFolder::_getNextFileName(char * &filename)
{
#if defined(WIN32) || defined(_WIN32)
	if (!(::FindNextFile(hError, &FindData)))return false;
#ifdef UNICODE
	wchar_t *wFileName = NULL;
	wFileName = FindData.cFileName;
	filename = wtoc(wFileName, filename);
#else
	filename = FindData.cFileName;
#endif // UNICODE

	return true;
#endif // WIN32

#ifdef linux
	if((ptr = readdir(dir)) == NULL)return false;
	filename = ptr->d_name;
	return true;
#endif // linux
}

TraverFolder::~TraverFolder()
{
}

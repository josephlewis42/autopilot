/*
 * File.cpp
 *
 *  Created on: Jun 11, 2013
 *      Author: Joseph Lewis <joehms22@gmail.com>
 *  Copyright 2013 Joseph Lewis <joehms22@gmail.com>
 *
 *  This
 */

#include "File.h"

#include <vector>
#include <fstream>
#include <string>

std::string File::readFile(const std::string& path)
{
	if (!File::exists(path))
	{
		return std::string("");
	}

    std::ifstream ifs(path.c_str(), std::ios::in | std::ios::binary | std::ios::ate);

    std::ifstream::pos_type fileSize = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    std::vector<char> bytes(fileSize);
    ifs.read(&bytes[0], fileSize);

    return std::string(&bytes[0], fileSize);
}

bool File::exists(const std::string& path)
{
	return File::exists(path.c_str());
}

bool File::exists(const char* path)
{
  std::ifstream ifile(path);
  return ifile;
}

/*
 * File.h
 *
 * Filesystem utilities.
 *
 * Copyright 2013 Joseph Lewis <joehms22@gmail.com>
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 */

#ifndef FILE_H_
#define FILE_H_

#include <string>

/**
 * Provides simple file operations.
 */
class File
{
	public:
		/**
		 * Reads the given file and returns it, if does not exist,
		 * returns a blank string.
		 *
		 * @param path the path to the file to read
		 */
		static std::string readFile(const std::string& path);
		
		/**
		 * Tests if a file exists.
		 * @param path - the path to the file.
		 */
		static bool exists(const char* path);

		/**
		 * Tests if a file exists.
		 * @param path - the path to the file.
		 */
		static bool exists(const std::string& path);
};

#endif /* FILE_H_ */

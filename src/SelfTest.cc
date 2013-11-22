/*
 * SelfTest.cpp
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

#include "SelfTest.h"
#include "Configuration.h"
#include "Debug.h"
#include "SystemInformation.h"


SelfTest::SelfTest()
{
	// Show system information.
	message() << "Running on: " <<  SystemInformation::uname_like();

	// TODO Test to make sure files exist


	// Load and check config.
	message() << Configuration::getInstance()->toString();

	sleep(2);
}

SelfTest::~SelfTest()
{
}
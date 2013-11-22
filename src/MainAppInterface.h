/*
 * MainAppInterface.h
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

#ifndef MAINAPPINTERFACE_H_
#define MAINAPPINTERFACE_H_

#include <boost/signals2.hpp>

/*
 *
 */
class MainAppInterface
{
public:
	/// Terminate signal used to tell other threads the program is about to terminate
	//static boost::signals2::signal<void ()> terminate;
	virtual void terminate()
	{

	}
};

#endif /* MAINAPPINTERFACE_H_ */
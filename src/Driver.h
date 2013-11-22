/*
 * Driver.h
 *
 *  Created on: Aug 26, 2013
 *      Author: joseph
 */

#ifndef DRIVER_H_
#define DRIVER_H_

#include <boost/thread.hpp>

class Driver
{
private:
	/// store whether to terminate the thread
	bool _terminate;
	/// serialize access to _terminate
	boost::mutex _terminate_lock;

	// Keeps a list of all drivers so we can terminate them later.
	static boost::mutex _all_drivers_lock;
	static std::list<Driver*> all_drivers;

public:
	Driver();
	virtual ~Driver();
	inline bool terminateRequested() {boost::mutex::scoped_lock(_terminate_lock); return _terminate;}
	inline void terminate() {boost::mutex::scoped_lock(_terminate_lock); _terminate = true;};
	static void terminateAll();

};

#endif /* DRIVER_H_ */

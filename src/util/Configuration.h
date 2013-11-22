/*
 * Configuration.h
 *
 * A simple configuration manager that looks up arbitrary key:value pairs.
 *
 * Copyright (c) 2013 Joseph Lewis <joehms22@gmail.com> | <joseph@josephlewis.net>
 * Licensed under the GPL 3
 *
 * Portions of the code were taken from FieldKit:
 * Copyright (c) 2010-2011 Marcus Wendt <marcus@field.io>
 * Licensed under the GPL 3
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <string>
#include <map>

#include <boost/thread.hpp>
#include <boost/signals2.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "Debug.h"


class Configuration
{
public:

	// Returns the existing instance of Configuration or creates one if it does
	// not yet exist.
	static Configuration* getInstance();

	// Loads the values from the given XML file.
	// Returns true on successful loading, false on error
	bool loadXML(std::string path);
	bool loadProperties(std::string path);


	// Pretty prints the configuration to a string
	std::string toString();

	// Returns a string from the configuration.
	std::string gets(const std::string key, std::string alt="");

	// Returns a bool from the configuration
	bool getb(const std::string key, bool alt=false);

	// Returns an int from the configuration.
	int geti(const std::string key, int alt=0);

	// Returns a float from the configuration
	float getf(const std::string key, float alt=0.0f);

	// Returns a double from the configuration
	double getd(const std::string key, double alt=0.0);

	// Allows you to override the configuration with a vector of
	// -key=value strings as you'd get from a cli
	void overrideWith(const std::vector<std::string>& args);
	void overrideWith(int argc, char* const argv[]);

private:
	static Configuration* _instance;
	static boost::mutex _instance_lock;
	static std::map<std::string, std::string> _configuration;

	Configuration();
	virtual ~Configuration();

	template <typename T>
	T get(const std::string key, const T defaultValue)
	{
		std::map<std::string, std::string>::iterator it;
		it = _configuration.find(key);
		if(it != _configuration.end())
		{
			return fromString<T>((*it).second, defaultValue);
		}
		else
		{
			warning() << "No configuration found for: " << key << " Default value is: " << defaultValue;
			return defaultValue;
		}
	};

	template <typename T>
	T fromString(const std::string value, const T defaultValue)
	{
		try
		{
			return boost::lexical_cast<T>(value);
		}
		catch(boost::bad_lexical_cast &e)
		{
			return defaultValue;
		}
	}
};

#endif /* CONFIGURATION_H_ */

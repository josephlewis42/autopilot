/*
 * Configuration.h
 *
 * A simple configuration manager that looks up arbitrary key:value pairs.
 *
 * Copyright (c) 2013 Joseph Lewis <joehms22@gmail.com> | <joseph@josephlewis.net>
 * Licensed under the GPL 3
 *
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <string>
#include <map>

#include <boost/thread.hpp>
#include <boost/signals2.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include "Debug.h"


class Configuration
{
public:

	// Returns the existing instance of Configuration or creates one if it does
	// not yet exist.
	static Configuration* getInstance();

	// Loads the values from the given properties file.
	void loadProperties(std::string path);

	// Returns a string from the configuration.
	std::string gets(const std::string &key, const std::string &alt="");

	// Returns a bool from the configuration
	bool getb(const std::string &key, bool alt=false);

	// Returns an int from the configuration.
	int geti(const std::string &key, int alt=0);

	// Returns a float from the configuration
	float getf(const std::string &key, float alt=0.0f);

	// Returns a double from the configuration
	double getd(const std::string &key, double alt=0.0);

	// Allows you to override the configuration with a vector of
	// -key=value strings as you'd get from a cli
	void overrideWith(const std::vector<std::string>& args);
	void overrideWith(int argc, char* const argv[]);

	/**
	 * Sets a path to be a value.
	 */
	void set(const std::string &key, const std::string& value);

	/**
	 * Sets a path to be a double value
	 *
	 * @param key - the key to store the value under
	 * @param value - the value of the param
	 */
	void setd(const std::string &key, const double value);

	/**
	 * Sets a path to be an int value
	 *
	 * @param key - the key to store the value under
	 * @param value - the value of the param
	 */
	void seti(const std::string &key, const int value);

private:
	static Configuration* _instance;
	static boost::mutex _instance_lock;
	boost::property_tree::ptree _properties;
	static boost::mutex _propertiesLock;

	Configuration();
	virtual ~Configuration();

	void save();

};

#endif /* CONFIGURATION_H_ */

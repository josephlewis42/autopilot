/*
 * Configuration.cpp
 *
 * A simple configuration manager that looks up arbitrary key:value pairs.
 *
 * Copyright (c) 2013 Joseph Lewis <joehms22@gmail.com> | <joseph@josephlewis.net>
 * Licensed under the GPL 3
 */


#include "Configuration.h"
#include "Debug.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>



#include <map>
#include <vector>
#include <string>
#include <string.h>
#include <utility>




// Static Class variable instantiation
Configuration* Configuration::_instance = NULL;
std::mutex Configuration::_instance_lock;
std::mutex Configuration::_propertiesLock;

// Variables
std::string ROOT_ELEMENT = "configuration.";
std::string DEFAULT_XML_FILE_PATH = "config.xml";
const char NEWLINE_CHARACTER = '\n';

const Logger LOG("Configuration: ");

Configuration::Configuration()
{
    _instance = this;

	try
	{
		read_xml(DEFAULT_XML_FILE_PATH, _properties, boost::property_tree::xml_parser::trim_whitespace);
	}
	catch(boost::exception const& e)
	{
		std::cout << "Could not find configuration file: " << DEFAULT_XML_FILE_PATH << std::endl;
	}
}

Configuration::~Configuration()
{
}

void Configuration::overrideWith(const std::vector<std::string>& args)
{
	for(unsigned int i = 0; i < args.size(); ++i)
	{
		// check if argument is a key=value pair
		std::string arg = args[i];
		std::vector<std::string> strs;
		boost::split(strs, arg, boost::is_any_of("="));

		if(strs.size() == 2)
		{
			std::string key = strs[0];
			// remove leading - if exists
			if(key.substr(0,1) == "-")
			{
				key = key.substr(1);
			}
			set(ROOT_ELEMENT + key, strs[1]);
		}
	}
}

void Configuration::overrideWith(int argc, char* const argv[])
{
    std::vector<std::string> args;
    for(int arg = 0; arg < argc; ++arg)
    {
    	args.push_back(std::string(argv[arg]));
    }

    overrideWith(args);
}


Configuration* Configuration::getInstance()
{
	std::lock_guard<std::mutex> lock(_instance_lock);

	if(NULL == _instance)
	{
		_instance = new Configuration();
	}

	return _instance;
}


std::string Configuration::gets(const std::string &key, const std::string &alt)
{
	std::lock_guard<std::mutex> lock(_propertiesLock);
	try
	{
		return _properties.get<std::string>(ROOT_ELEMENT + key);
	}catch(boost::exception const& e)
	{
		_properties.put(ROOT_ELEMENT + key, alt);
		save();
		return alt;
	}
}


bool Configuration::getb(const std::string &key, bool alt)
{
	std::lock_guard<std::mutex> lock(_propertiesLock);

	try
	{
		return _properties.get<bool>(ROOT_ELEMENT + key);
	}catch(boost::exception const& e)
	{
		_properties.put(ROOT_ELEMENT + key, alt);
		save();
		return alt;
	}
}


int Configuration::geti(const std::string &key, int alt)
{
	std::lock_guard<std::mutex> lock(_propertiesLock);

	try
	{
		return _properties.get<int>(ROOT_ELEMENT + key);
	}catch(boost::exception const& e)
	{
		_properties.put(ROOT_ELEMENT + key, alt);
		save();
		return alt;
	}
}


double Configuration::getd(const std::string &key, double alt)
{
	std::lock_guard<std::mutex> lock(_propertiesLock);

	try
	{
		return _properties.get<double>(ROOT_ELEMENT + key);
	}catch(boost::exception const& e)
	{
		_properties.put(ROOT_ELEMENT + key, alt);
		save();
		return alt;
	}
}


float Configuration::getf(const std::string &key, float alt)
{
	std::lock_guard<std::mutex> lock(_propertiesLock);

	try
	{
		return _properties.get<float>(ROOT_ELEMENT + key);
	}catch(boost::exception const& e)
	{
		_properties.put(ROOT_ELEMENT + key, alt);
		save();
		return alt;
	}
}

void Configuration::set(const std::string &key, const std::string& value)
{
	std::lock_guard<std::mutex> lock(_propertiesLock);

	_properties.put(ROOT_ELEMENT + key, value);
	save();
}

void Configuration::setd(const std::string &key, const double value)
{
	set(key, boost::lexical_cast<std::string>(value));
}

void Configuration::seti(const std::string &key, const int value)
{
	set(key, boost::lexical_cast<std::string>(value));
}

void Configuration::save()
{
	try
	{
		boost::property_tree::xml_writer_settings<char> settings('\t', 1);
		write_xml(DEFAULT_XML_FILE_PATH, _properties, std::locale(), settings);
	}catch(boost::exception const& ex)
	{
		LOG.warning() << "Can't save configuration";
	}
}

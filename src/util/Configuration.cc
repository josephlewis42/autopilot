/*
 * Configuration.cpp
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


#include "Configuration.h"
#include "File.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>



#include <map>
#include <vector>
#include <string>
#include <string.h>
#include <utility>




// Static Class variable instantiation
Configuration* Configuration::_instance = NULL;
boost::mutex Configuration::_instance_lock;
std::map<std::string, std::string> Configuration::_configuration;


// Variables
std::string ROOT_ELEMENT = "configuration.";
std::string DEFAULT_XML_FILE_PATH = "config.xml";
std::string DEFAULT_PROPERTIES_FILE_PATH = "config.properties";
const char NEWLINE_CHARACTER = '\n';

Configuration::Configuration()
{
	//loadXML(DEFAULT_XML_FILE_PATH);
	try{
		read_xml(DEFAULT_XML_FILE_PATH, _properties);
	}
	catch(boost::exception const& e)
	{
		warning() << "Could not find configuration file: " << DEFAULT_XML_FILE_PATH;
	}
	loadProperties(DEFAULT_PROPERTIES_FILE_PATH);

    _instance = this;
}

Configuration::~Configuration()
{
}


std::string Configuration::toString()
{
	std::string str = "Settings: ";
	str += NEWLINE_CHARACTER;

	std::map<std::string, std::string>::iterator it;
	for(it=_configuration.begin(); it != _configuration.end(); it++ )
	{
		str += "* ";
		str += (*it).first;
		str += " = ";
		str += (*it).second;
		str += NEWLINE_CHARACTER;
	}

	return str;
}

void Configuration::loadProperties(std::string path)
{
	std::string contents = File::readFile(path);

	std::vector<std::string> strs;
	boost::split(strs,contents,boost::is_any_of("\n\r"));

	overrideWith(strs);
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
			_properties.put(ROOT_ELEMENT + key, strs[1]);
			_configuration[key] = strs[1];
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
	boost::mutex::scoped_lock lock(_instance_lock);

	if(NULL == _instance)
	{
		_instance = new Configuration();
	}

	return _instance;
}


std::string Configuration::gets(const std::string key, std::string alt)
{
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


bool Configuration::getb(const std::string key, bool alt)
{
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


int Configuration::geti(const std::string key, int alt)
{
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


double Configuration::getd(const std::string key, double alt)
{
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


float Configuration::getf(const std::string key, float alt)
{
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

void Configuration::set(const std::string key, const std::string value)
{
	_properties.put(ROOT_ELEMENT + key, value);
	save();
}

void Configuration::save()
{
	try
	{
		write_xml(DEFAULT_XML_FILE_PATH, _properties);
	}catch(boost::exception const& ex)
	{
		warning() << "Can't save configuration";
	}
}

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

#include <rapidxml/rapidxml.hpp>

#include <boost/thread.hpp>
#include <boost/signals2.hpp>

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
std::string DEFAULT_XML_FILE_PATH = "config.xml";
const char NEWLINE_CHARACTER = '\n';

Configuration::Configuration()
{
	loadXML(DEFAULT_XML_FILE_PATH);
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


bool Configuration::loadXML(std::string path)
{
	try
	{
		std::string text = File::readFile(path);

		char* nText = new char[text.size() + 1];  // Create char buffer to store string copy
		strcpy(nText, text.c_str());             // Copy string into char buffer

		rapidxml::xml_document<> doc;    // character type defaults to char
		doc.parse<0>(nText);    // 0 means default parse flags

		rapidxml::xml_node<>* curNode = doc.first_node();
		while( curNode != NULL )
		{
			std::string title = curNode->name();
			std::string value = curNode->value();

			_configuration.insert(std::pair<std::string,std::string>(title, value) );

			curNode = curNode->next_sibling();
		}
		
		return true;
	} 
	catch(rapidxml::parse_error err)
	{
		return false;
	}
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
    return get<std::string>(key, alt);
}


bool Configuration::getb(const std::string key, bool alt)
{
    return get<bool>(key, alt);
}


int Configuration::geti(const std::string key, int alt)
{
    return get<int>(key, alt);
}


double Configuration::getd(const std::string key, double alt)
{
    return get<double>(key, alt);
}


float Configuration::getf(const std::string key, float alt)
{
    return get<float>(key, alt);
}

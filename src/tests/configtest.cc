// g++ *.cc -lboost_system

#include "Configuration.h"

int main()
{
	// Test getInstance()
	Configuration* instance = Configuration::getInstance();
	Configuration* instance2 = Configuration::getInstance();
	
	assert(instance != NULL);
	assert(instance == instance2);
	
	// Test loadXML
	assert(instance->loadXML("config.xml"));
	assert(! instance->loadXML("invalid.xml"));
	
	// Test override, gets
	char* argv[] = {"-first=param", "-second=param2", "another_arg", "-tail=param3"};
	int argc = 4;
	
	instance->overrideWith(argc, argv);
	
	assert(instance->gets("first") == std::string("param"));
	assert(instance->gets("another_arg", "nothing") == std::string("nothing"));
	assert(instance->gets("tail") == std::string("param3"));
	
	// Test toString
	printf("%s\n", instance->toString().c_str());
	
	// Test getb
	assert(instance->getb("bool", false) == true); // a bool
	assert(instance->getb("string", true) == true); // not a bool
	
	// Test geti
	assert(instance->geti("int", 0) == 33);
	assert(instance->geti("string", 0) == 0);
	
	// Test getf
	assert(instance->getf("float", 0.0f) == 3.14f);
	assert(instance->getf("string", 0.0f) == 0.0f);
	
	// Test getd
	assert(instance->getd("double", 0.0d) == 2.718d);
	assert(instance->getd("string", 0.0d) == 0.0d);
	
	
}

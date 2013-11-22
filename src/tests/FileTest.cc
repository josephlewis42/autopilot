#include "File.h"
#include <assert.h>

int main()
{
	std::string contents = File::readFile("/etc/passwd");
	assert(contents.length() != 0);
	std::string contents2 = File::readFile("doesnotexist");
	assert(contents2.length() == 0);
}

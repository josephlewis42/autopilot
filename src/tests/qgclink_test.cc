#include "MainApp.h"
#include "QGCLink.h"

void MainApp::run()
{
	QGCLink::getInstance();

	for(;;);
}

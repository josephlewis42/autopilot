/**************************************************************************
 * Copyright 2012 Bryan Godbolt
 * Copyright 2013 Joseph Lewis <joehms22@gmail.com>
 *
 * This file is part of ANCL Autopilot.
 *
 *     ANCL Autopilot is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     ANCL Autopilot is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with ANCL Autopilot.  If not, see <http://www.gnu.org/licenses/>.
 *************************************************************************/

/**
   \author Bryan Godbolt <godbolt@ece.ualberta.ca>
   @author Nikos Vitzilaios <nvitzilaios@ualberta.ca>
   @author Aakash Vasudevan <avasudev@ualberta.ca>
   \author Hasitha Senanayake <senanaya@ualberta.ca>
   \author Joseph Lewis III <joseph@josephlewis.net>
   \mainpage
   This project contains the code for the UDenver autopilot system.

   Source code can be found [at the github repo](https://github.com/josephlewis42/autopilot)
 */


/* Boost Headers */
#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>

/* Project Headers */
#include "MainApp.h"
#include "Configuration.h"
#include "SystemInformation.h"
#include "Debug.h"
#include "LogFile.h"

#include <gtest/gtest.h>

// TESTS
TEST(MainTest, Positive)
{
    EXPECT_EQ(1, 1);
    EXPECT_EQ(0, 0);
}

Logger mainLogger("Main");

int main(int argc, char* argv[])
{

    printf("Usage: autopilot [-override_param=value ...]\n");
    printf("Usage: autopilot test\t(for running unittests)\n");
    printf("PID is: %d\n", getpid());
    printf("Autopilot Version: %s %s\n", __DATE__, __TIME__);

    // do unittesting if needed.
    if(argc == 2 && strcmp(argv[1], "test") == 0)
    {
        ::testing::InitGoogleTest(&argc, argv);
        if(RUN_ALL_TESTS() != 0)
        {
            return 1;
        }

        return 0;
    }

    LogFile::getInstance();

    // Set configuration params from CLI if applicable
    Configuration::getInstance()->overrideWith(argc, argv);


    // Show system information.
    mainLogger.message() << "Running on: " <<  SystemInformation::uname_like();
    mainLogger.message() << "Autopilot Version: " << __DATE__ << " " << __TIME__;


    // Start up the main application.
    MainApp* m = MainApp::getInstance();
    m->run();

    printf("\n\n\n%s\n", Configuration::getInstance()->getDescription().c_str());

    return 0;
}

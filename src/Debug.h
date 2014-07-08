/*******************************************************************************
 * Copyright 2012 Bryan Godbolt
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
 ******************************************************************************/

#ifndef DEBUG_H_
#define DEBUG_H_

/* STL Headers */
#include <iosfwd>
#include <sstream>
#include <string>
#include <vector>
#include <mutex>

/* Boost Headers */
#include <boost/numeric/ublas/fwd.hpp>


/** @brief Implements a debugging object similar to QDebug @see http://doc.qt.nokia.com/latest/qdebug.html
 *
 * This class should be used to print all debugging messages created during developement.  Doing so ensures
 * that when the release version is built none of the debugging messages will be printed.  In addition
 * access to stderr is serialized (threadsafe).  A newline character is appended to each message.  Spaces
 * are automatically inserted around numerical types (int and double).
 *
 * When warning() or critical() are used instead of debug() the message is printed to stderr in the release
 * version as well as the debugging version.  In addition, in these modes the message is also sent to the log
 * file messages.log.
 *
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date November 7, 2011
 * @b Example:
 * @code
 * #include "Debug.h"
 *
 * debug() << "Debugging message";
 * debug() << __FILE__ << __LINE__  << ": another message";
 *
 * @endcode
 * @note a new line is automatically appended
 *
 * To print a class which does not have << defined already, just implement the operator<< function as is done in the following example.
 *
 * @code
 * class A
 * {
 * public:
 * A();
 * std::string foo();
 * int bar(double);
 * friend Debug& operator<<(Debug& dgb, const A& a);
 * };
 *
 * Debug& operator<<(Debug& dbg, const A& a)
 * {
 * 	return dbg << "A is " << foo();
 * }
 * @endcode
 */
class Debug
{
public:
    enum DEBUG_LEVEL
    {
        IGNORE=0,
        DEBUG=1,
        MESSAGE=2,
        WARNING=3,
        CRITICAL=4
    };
    explicit Debug(DEBUG_LEVEL debug_level = DEBUG, std::string prefix = "");
    Debug(const Debug& other);
    ~Debug();
    Debug& operator<<(const std::string& s);
    Debug& operator<<(const char* c);
    Debug& operator<<(const int i);
    Debug& operator<<(const unsigned int i);
    Debug& operator<<(const unsigned long i);
    Debug& operator<<(const double d);
    Debug& operator<<(const void* ptr);

    template <typename T, size_t N>
    Debug& operator<<(const std::array<T,N>& a);
    template <typename T>
    Debug& operator<<(const std::vector<T>& v);
    Debug& operator<<(const std::vector<uint8_t>& v);
    Debug& operator<<(std::ios_base& (*pf)(std::ios_base&));
    Debug& operator<<(std::ostream& (*pf)(std::ostream&));
    Debug& operator<<(const boost::numeric::ublas::vector<double>& v);
    Debug& operator<<(const boost::numeric::ublas::matrix<double>& m);
    Debug& operator<<(const boost::numeric::ublas::vector<float>& v);
    Debug& operator<<(const boost::numeric::ublas::matrix<float>& m);

private:
    static std::mutex cerr_lock;
    static int message_count;
    static std::string last_message;

    std::stringstream ss;
    DEBUG_LEVEL debug_level;

    /// appends the string version of the current level to the start of the message.
    void appendLevel();
};

/** Logger is a general-purpose logging mechanism in the style of the Apache Commons
Logging framework.

Loggers must be instantiated before using any of the debugging methods.

Each logger can have an individual prefix which is prepended to all messages coming
from the logger.

Additinally, loggers can have individual levels set, all levels below a given level
are ignored.

**/
class Logger
{
private:
    std::string _prefix;
    Debug::DEBUG_LEVEL _ignore_level; /// all levels less than this won't be printed.

    Debug log(std::string initial_value, Debug::DEBUG_LEVEL desired_level) const
    {
        auto lvl = (desired_level < _ignore_level)? Debug::IGNORE : desired_level;
        auto dbg = Debug(lvl, _prefix);
        dbg << initial_value;
        return dbg;
    }

public:
    /** Constructs a new logger with the given prefix to prepend.

    @param prefix - a prefix to prepend to all messages
    @param level - the default level to start printing messages
    at. Default: DEBUG

    @note prefixes are automatically  seperated from the rest of
    the text using a colon and space.
    **/
    Logger(std::string prefix, Debug::DEBUG_LEVEL level = Debug::DEBUG)
    :_prefix(prefix+ ": "),
     _ignore_level(level)
     {}

    /**
     * Sets the logging level of this logger.
     **/
    void setLoggingLevel(int log_level)
    {
        _ignore_level = (Debug::DEBUG_LEVEL) log_level;
    }

    /** Sends an ignore message. **/
    Debug ignore(std::string init = "") const
    {
        return log(init, Debug::IGNORE);
    }

    /** Sends a trace message, good for things like hex
    dumps.

    **/
    Debug trace(std::string init = "") const
    {
        return log(init, Debug::IGNORE);
    }

    /** Sends a debug message.

    Good for fine-grained messages, like locations in code or notification
    that a method ran.
    **/
    Debug debug(std::string init = "") const
    {
        return log(init, Debug::DEBUG);
    }

    /**
    @deprecated - this message is deprecated in favor of info()
    **/
    Debug message(std::string init = "") const
    {
        return log(init, Debug::MESSAGE);
    }

    /** Logs an info message.
     *
     * Good for notifications that the user generally doesn't care about, but
     * should be noted in case of a larger problem, i.e. names of files that
     * are being opened for settings/reading/writing.
     */
    Debug info(std::string init = "") const
    {
        return log(init, Debug::MESSAGE);
    }

    /** Logs a warning message.
     *
     * Warning messages denote a problem that has occured in the software that
     * may lead to unstable operation.
     */
    Debug warning(std::string init = "") const
    {
        return log(init, Debug::WARNING);
    }

    /** Logs a critical message.
     *
     * Critical messages denote a bad problem that has occured in the software
     * these errors are not recoverable and the system is unstable.
     */
    Debug critical(std::string init = "") const
    {
        return log(init, Debug::CRITICAL);
    }
};


template <typename T, size_t N>
Debug& Debug::operator<<(const std::array<T,N>& a)
{
    ss << "[";
    for (size_t i=0; i<a.size(); i++)
    {
        ss << a[i];
        if (i < a.size()-1)
            ss << ", ";
    }
    ss << "]";
    return *this;
}

template <typename T>
Debug& Debug::operator<<(const std::vector<T>& v)
{
    ss << "[";
    for (size_t i = 0; i<v.size(); i++)
    {
        ss << v[i];
        if (i < v.size() - 1)
            ss << ", ";
    }
    ss << "]";
    return *this;
}
#endif

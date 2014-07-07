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
#include <thread>
#include <boost/array.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
namespace blas = boost::numeric::ublas;
#include <boost/numeric/ublas/io.hpp>
#include <boost/signals2/signal.hpp>




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
    template <typename T>
    Debug& operator<<(const blas::vector<T>& v);
    template <typename T>
    Debug& operator<<(const blas::matrix<T>& m);

    /// emitted when a warning message is created
    static boost::signals2::signal<void (std::string)> warningSignal;
    /// emitted when a critical message is created
    static boost::signals2::signal<void (std::string)> criticalSignal;

private:
    static std::mutex cerr_lock;
    static int message_count;
    static std::string last_message;

    std::stringstream ss;
    DEBUG_LEVEL debug_level;

    /// appends the string version of the current level to the start of the message.
    void appendLevel();
};

class Logger
{
private:
    std::string _prefix;
    Debug::DEBUG_LEVEL _ignore_level; /// all levels less than this won't be printed.

public:
    Logger(std::string prefix)
    :_prefix(prefix),
     _ignore_level(Debug::DEBUG)
     {}

    void setLoggingLevel(int log_level)
    {
        _ignore_level = (Debug::DEBUG_LEVEL) log_level;
    }

    Debug ignore() const
    {
        auto lvl = (Debug::IGNORE < _ignore_level)? Debug::IGNORE : Debug::IGNORE;
        return Debug(lvl, _prefix);
    }

    Debug trace() const
    {
        auto lvl = (Debug::IGNORE < _ignore_level)? Debug::IGNORE : Debug::IGNORE;
        return Debug(lvl, _prefix);
    }

    Debug debug() const
    {
        auto lvl = (Debug::DEBUG < _ignore_level)? Debug::IGNORE : Debug::DEBUG;
        return Debug(lvl,  _prefix);
    }
    Debug message() const
    {
        auto lvl = (Debug::MESSAGE < _ignore_level)? Debug::IGNORE : Debug::MESSAGE;
        return Debug(lvl, _prefix);
    }

    Debug info() const
    {
        return message();
    }

    Debug warning() const
    {
        auto lvl = (Debug::WARNING < _ignore_level)? Debug::IGNORE : Debug::WARNING;
        return Debug(lvl, _prefix);
    }

    Debug critical() const
    {
        auto lvl = (Debug::CRITICAL < _ignore_level)? Debug::IGNORE : Debug::CRITICAL;
        return Debug(lvl, _prefix);
    }

    void ignore(std::string value) const
    {
        auto lvl = (Debug::IGNORE < _ignore_level)? Debug::IGNORE : Debug::IGNORE;
        Debug(lvl, _prefix) << value;
    }
    void debug(std::string value) const
    {
        auto lvl = (Debug::DEBUG < _ignore_level)? Debug::IGNORE : Debug::DEBUG;
        Debug(lvl,  _prefix) << value;
    }
    void message(std::string value) const
    {
        auto lvl = (Debug::MESSAGE < _ignore_level)? Debug::IGNORE : Debug::MESSAGE;
        Debug(lvl, _prefix) << value;
    }
    void info(std::string value) const
    {
        message(value);
    }

    void warning(std::string value) const
    {
        auto lvl = (Debug::WARNING < _ignore_level)? Debug::IGNORE : Debug::WARNING;
        Debug(lvl, _prefix) << value;
    }
    void critical(std::string value) const
    {
        auto lvl = (Debug::CRITICAL < _ignore_level)? Debug::IGNORE : Debug::CRITICAL;
        Debug(lvl, _prefix) << value;
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

template <typename T>
Debug& Debug::operator<<(const blas::vector<T>& v)
{
    ss << v;
    return *this;
}

template <typename T>
Debug& Debug::operator<<(const blas::matrix<T>& m)
{
    ss << m;
    return *this;
}
#endif

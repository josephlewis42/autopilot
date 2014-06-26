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
        DEBUG,
        WARNING,
        CRITICAL,
        MESSAGE,
        IGNORE
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
    Debug& operator<<(const boost::array<T,N>& a);
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

static inline Debug debug()
{
    return Debug(Debug::DEBUG);
}
static inline Debug message()
{
    return Debug(Debug::MESSAGE);
}
static inline Debug warning()
{
    return Debug(Debug::WARNING);
}
static inline Debug critical()
{
    return Debug(Debug::CRITICAL);
}

class Logger
{
private:
    std::string _prefix;

public:
    Logger(std::string prefix)
        :_prefix(prefix) {}

    Debug ignore() const
    {
        return Debug(Debug::IGNORE, _prefix);
    }
    Debug debug() const
    {
        return Debug(Debug::DEBUG,  _prefix);
    }
    Debug message() const
    {
        return Debug(Debug::MESSAGE, _prefix);
    }
    Debug warning() const
    {
        return Debug(Debug::WARNING, _prefix);
    }
    Debug critical() const
    {
        return Debug(Debug::CRITICAL, _prefix);
    }

    void ignore(std::string value) const {}
    void debug(std::string value) const
    {
        Debug(Debug::DEBUG,  _prefix) << value;
    }
    void message(std::string value) const
    {
        Debug(Debug::MESSAGE, _prefix) << value;
    }
    void warning(std::string value) const
    {
        Debug(Debug::WARNING, _prefix) << value;
    }
    void critical(std::string value) const
    {
        Debug(Debug::CRITICAL, _prefix) << value;
    }

};


template <typename T, size_t N>
Debug& Debug::operator<<(const boost::array<T,N>& a)
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

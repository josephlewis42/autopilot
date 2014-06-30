/**
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
 *
 * Stolen from:
 * http://enki-tech.blogspot.com/2012/08/c11-generic-singleton.html

 EXAMPLE
 -------

         class Map: public Singleton<Map>
         {
             friend class Singleton<Map>;
             private:
                 Map(){}
                 ...
          };

**/

#pragma once
#ifndef SINGLETON_H
#define SINGLETON_H

#include <mutex>

template <class T>
class Singleton
{
public:
  template <typename... Args>
  static
  T* getInstance(Args... args)
  {
    std::lock_guard<std::mutex> lock(instance_lock_);

    if (!instance_)
      {
        instance_ = new T(std::forward<Args>(args)...);
      }

    return instance_;
  }

  static
  void destroyInstance()
  {
    delete instance_;
    instance_ = nullptr;
  }

private:
    static T* instance_;
    static std::mutex instance_lock_;
};

template <class T> T*  Singleton<T>::instance_ = nullptr;
template <class T> std::mutex Singleton<T>::instance_lock_;


#endif //SINGLETON_H

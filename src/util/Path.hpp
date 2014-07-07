/**
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
 *
**/

#ifndef PATH_HPP_
#define PATH_HPP_

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <dirent.h>
#include <string.h>

class Path
{
    private:
    std::string _path;

        /// Recusvivlely removes a directory.
    int remove_all(const char* path)
    {

        DIR *d = opendir(path);
        size_t path_len = strlen(path);
        int r = -1;

        if (d)
        {
            struct dirent *p;
            r = 0;

            while (!r && (p=readdir(d)))
            {
                int r2 = -1;
                char *buf;
                size_t len;

                /* Skip the names "." and ".." as we don't want to recurse on them. */
                if (!strcmp(p->d_name, ".") || !strcmp(p->d_name, ".."))
                {
                    continue;
                }

                len = path_len + strlen(p->d_name) + 2;
                buf = (char*) malloc(len);

                if (buf)
                {
                    struct stat statbuf;
                    snprintf(buf, len, "%s/%s", path, p->d_name);
                    if (!stat(buf, &statbuf))
                    {
                        if (S_ISDIR(statbuf.st_mode))
                        {
                            r2 = remove_all(buf);
                        }
                        else
                        {
                            r2 = unlink(buf);
                        }
                    }
                    free(buf);
                }
                r = r2;
            }
            closedir(d);
        }
        if (!r)
        {
            r = rmdir(path);
        }
        return r;
    };

    public:
    Path()
    {
        char cwd[1024];
        if (getcwd(cwd, sizeof(cwd)) != NULL)
            _path = std::string(cwd);
    };

    Path(std::string initial)
        :_path(initial)
        {};

    Path(const char* initial)
        :_path(initial)
        {};

    bool has_extension()
    {
        return get_extension().length() > 0;
    };

    /// Check if the path exists
    bool exists ()
    {
        struct stat buffer;
        return (stat (_path.c_str(), &buffer) == 0);
    };




    /// Recursively removes a directory.
    int remove_all()
    {
        return remove_all(_path.c_str());
    };

    std::string get_extension()
    {
        std::string::size_type idx;

        idx = _path.rfind('.');

        if(idx != std::string::npos)
        {
            return _path.substr(idx+1);
        }

        return "";
    };

    /** Recursively makes a directory.
          * src: http://nion.modprobe.de/blog/archives/357-Recursive-directory-creation.html
          **/
    void create_directories()
    {
        const char* dir = _path.c_str();
        char tmp[256];
        char *p = NULL;
        size_t len;

        snprintf(tmp, sizeof(tmp),"%s",dir);
        len = strlen(tmp);
        if(tmp[len - 1] == '/')
            tmp[len - 1] = 0;
        for(p = tmp + 1; *p; p++)
            if(*p == '/') {
            *p = 0;
            mkdir(tmp, S_IRWXU);
            *p = '/';
        }
        mkdir(tmp, S_IRWXU);
    };

    std::string toString() const
    {
        return "" + _path;
    };

    friend Path operator/(const Path &p, const char* toappend);
    friend Path operator/(const Path &p, const std::string toappend);

};

Path operator/(const Path &p, const char* toappend)
{
    return Path(p._path + "/" + toappend);
};

Path operator/(const Path &p, const std::string toappend)
{
    return Path(p._path + "/" + toappend);
};

#endif // PATH_HPP_

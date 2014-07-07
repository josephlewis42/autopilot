/**
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
 *
**/

#include "Path.hpp"
#include <gtest/gtest.h>


// TESTS
TEST(Path, exists)
{

    EXPECT_EQ(Path("/tmp").exists(), true);
    EXPECT_EQ(Path("//tmp").exists(), true);
    EXPECT_EQ(Path("/tmpasdfasdfasdf").exists(), false);
}

TEST(Path, has_extension)
{

    EXPECT_EQ(Path("/tmp").has_extension(), false);
    EXPECT_EQ(Path("document.docx").has_extension(), true);
}

TEST(Path, get_extension)
{
    EXPECT_EQ(Path("/tmp").get_extension().length(), 0);
    EXPECT_EQ(Path("document.docx").get_extension(), std::string("docx"));
}

TEST(Path, create_rm_directories)
{
    Path("/tmp/a/b/c/d/e").create_directories();
    EXPECT_EQ(Path("/tmp/a/b/c/d/e").exists(), true);

    Path("/tmp/a/").remove_all();
    EXPECT_EQ(Path("/tmp/a/").exists(), false);
}

TEST(Path, append_paths)
{
    Path tmp("/tmp");
    tmp = tmp / "a" / "b" / "c" / "d" / "e";

    EXPECT_EQ(tmp.toString(), std::string("/tmp/a/b/c/d/e"));
}

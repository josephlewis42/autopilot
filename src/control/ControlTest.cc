/*
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
 */
#include "Control.h"
#include "Parameter.h"
#include <gtest/gtest.h>



// TESTS
TEST(Control, SET_PARAM_ROLL_KP)
{
    float first = Control::getInstance()->attitude_pid_controller().get_roll_proportional();
    Parameter p(attitude_pid::PARAM_ROLL_KP, first * .5, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->attitude_pid_controller().get_roll_proportional();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_ROLL_KD)
{
    float first = Control::getInstance()->attitude_pid_controller().get_roll_derivative();
    Parameter p(attitude_pid::PARAM_ROLL_KD, first * .5, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->attitude_pid_controller().get_roll_derivative();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_ROLL_KI)
{
    float first = Control::getInstance()->attitude_pid_controller().get_roll_integral();
    Parameter p(attitude_pid::PARAM_ROLL_KI, first * .5, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->attitude_pid_controller().get_roll_integral();

    EXPECT_NE(first, second);
}


TEST(Control, SET_PARAM_PITCH_KP)
{
    float first = Control::getInstance()->attitude_pid_controller().get_pitch_proportional();
    Parameter p(attitude_pid::PARAM_PITCH_KP, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->attitude_pid_controller().get_pitch_proportional();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_PITCH_KD)
{
    float first = Control::getInstance()->attitude_pid_controller().get_pitch_derivative();
    Parameter p(attitude_pid::PARAM_PITCH_KD, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->attitude_pid_controller().get_pitch_derivative();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_PITCH_KI)
{
    float first = Control::getInstance()->attitude_pid_controller().get_pitch_integral();
    Parameter p(attitude_pid::PARAM_PITCH_KI, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->attitude_pid_controller().get_pitch_integral();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_ROLL_TRIM)
{
    float first = Control::getInstance()->attitude_pid_controller().get_roll_trim_degrees();
    Parameter p(attitude_pid::PARAM_ROLL_TRIM, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->attitude_pid_controller().get_roll_trim_degrees();

    EXPECT_NE(first, second);
}


TEST(Control, SET_PARAM_PITCH_TRIM)
{
    float first = Control::getInstance()->attitude_pid_controller().get_pitch_trim_degrees();
    Parameter p(attitude_pid::PARAM_PITCH_TRIM, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->attitude_pid_controller().get_pitch_trim_degrees();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_X_KP)
{
    float first = Control::getInstance()->translation_pid_controller().get_x_proportional();
    Parameter p(translation_outer_pid::PARAM_X_KP, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->translation_pid_controller().get_x_proportional();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_X_KD)
{
    float first = Control::getInstance()->translation_pid_controller().get_x_derivative();
    Parameter p(translation_outer_pid::PARAM_X_KD, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->translation_pid_controller().get_x_derivative();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_X_KI)
{
    float first = Control::getInstance()->translation_pid_controller().get_x_integral();
    Parameter p(translation_outer_pid::PARAM_X_KI, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->translation_pid_controller().get_x_integral();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_Y_KP)
{
    float first = Control::getInstance()->translation_pid_controller().get_y_proportional();
    Parameter p(translation_outer_pid::PARAM_Y_KP, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->translation_pid_controller().get_y_proportional();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_Y_KD)
{
    float first = Control::getInstance()->translation_pid_controller().get_y_derivative();
    Parameter p(translation_outer_pid::PARAM_Y_KD, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->translation_pid_controller().get_y_derivative();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_Y_KI)
{
    float first = Control::getInstance()->translation_pid_controller().get_y_integral();
    Parameter p(translation_outer_pid::PARAM_Y_KI, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->translation_pid_controller().get_y_integral();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_TRAVEL)
{
    float first = Control::getInstance()->translation_pid_controller().scaled_travel_degrees();
    Parameter p(translation_outer_pid::PARAM_TRAVEL, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->translation_pid_controller().scaled_travel_degrees();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_TRAVEL_SBF)
{
    float first = Control::getInstance()->x_y_sbf_controller.scaled_travel_degrees();
    Parameter p(tail_sbf::PARAM_TRAVEL, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->x_y_sbf_controller.scaled_travel_degrees();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_X_KP_TAIL)
{
    float first = Control::getInstance()->x_y_sbf_controller.get_x_proportional();
    Parameter p(tail_sbf::PARAM_X_KP, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->x_y_sbf_controller.get_x_proportional();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_X_KD_TAIL)
{
    float first = Control::getInstance()->x_y_sbf_controller.get_x_derivative();
    Parameter p(tail_sbf::PARAM_X_KD, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->x_y_sbf_controller.get_x_derivative();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_X_KI_TAIL)
{
    float first = Control::getInstance()->x_y_sbf_controller.get_x_integral();
    Parameter p(tail_sbf::PARAM_X_KI, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->x_y_sbf_controller.get_x_integral();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_Y_KP_TAIL)
{
    float first = Control::getInstance()->x_y_sbf_controller.get_y_proportional();
    Parameter p(tail_sbf::PARAM_Y_KP, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->x_y_sbf_controller.get_y_proportional();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_Y_KD_TAIL)
{
    float first = Control::getInstance()->x_y_sbf_controller.get_y_derivative();
    Parameter p(tail_sbf::PARAM_Y_KD, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->x_y_sbf_controller.get_y_derivative();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_Y_KI_TAIL)
{
    float first = Control::getInstance()->x_y_sbf_controller.get_y_integral();
    Parameter p(tail_sbf::PARAM_Y_KI, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->x_y_sbf_controller.get_y_integral();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_HOVER_TIME_CIRCLE)
{
    float first = Control::getInstance()->circle_trajectory.get_hover_time();
    Parameter p(circle::PARAM_HOVER_TIME, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->circle_trajectory.get_hover_time();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_RADIUS)
{
    float first = Control::getInstance()->circle_trajectory.get_radius();
    Parameter p(circle::PARAM_RADIUS, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->circle_trajectory.get_radius();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_SPEED_CIRCLE)
{
    float first = Control::getInstance()->circle_trajectory.get_speed();
    Parameter p(circle::PARAM_SPEED, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->circle_trajectory.get_speed();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_HOVER_TIME_LINE)
{
    float first = Control::getInstance()->line_trajectory.get_hover_time();
    Parameter p(line::PARAM_HOVER_TIME, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->line_trajectory.get_hover_time();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_SPEED_LINE)
{
    float first = Control::getInstance()->line_trajectory.get_speed();
    Parameter p(line::PARAM_SPEED, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->line_trajectory.get_speed();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_X_TRAVEL)
{
    float first = Control::getInstance()->line_trajectory.get_x_travel();
    Parameter p(line::PARAM_X_TRAVEL, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->line_trajectory.get_x_travel();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_Y_TRAVEL)
{
    float first = Control::getInstance()->line_trajectory.get_y_travel();
    Parameter p(line::PARAM_Y_TRAVEL, first + 1, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->line_trajectory.get_y_travel();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_MIX_ROLL)
{
    float first = Control::getInstance()->get_roll_mix();
    Parameter p(Control::PARAM_MIX_ROLL, first * .5, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->get_roll_mix();

    EXPECT_NE(first, second);
}

TEST(Control, SET_PARAM_MIX_PITCH)
{
    float first = Control::getInstance()->get_pitch_mix();
    Parameter p(Control::PARAM_MIX_PITCH, first * .5, 0);
    EXPECT_TRUE(Control::getInstance()->setParameter(p));
    float second = Control::getInstance()->get_pitch_mix();

    EXPECT_NE(first, second);
}


TEST(Control, EMPTY_PARAM_DOES_NOT_EXIST)
{
    // we should not allow an empty param.
    Parameter p("", 1, 0);
    EXPECT_FALSE(Control::getInstance()->setParameter(p));
}

#!/bin/bash
# ========================================================================
# Apache License 2.0
# Copyright (c) 2024, DYROS Robotics
#
# TOCABI Simulation Installation Script
# This script installs ROS Noetic and TOCABI Simulation inside an 
# Ubuntu 20.04 container (created via Distrobox).
# ========================================================================

set -e  # Stop script on error

echo "[DYROS] HW1 Settings..."

cd ~/catkin_ws/src/dyros_tocabi_v2
git checkout 5b63b2bd010b1a0a185d9b2d28da171639db2e54
cd ~/catkin_ws/src
rm -rf tocabi_cc
mv tocabi_cc_hw1_prob tocabi_cc

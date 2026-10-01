#!/bin/sh
# Host test for the MSC transfer maths (no hardware, no PlatformIO).
set -e
cd "$(dirname "$0")"
c++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -O1 \
	host_test.cpp -o /tmp/msc-poc-host-test
/tmp/msc-poc-host-test

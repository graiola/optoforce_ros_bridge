#!/bin/bash

package_path="$(dirname "$(readlink -f "$0")")"

sudo ln -s $package_path/lib/libOMD.so.1 /usr/lib

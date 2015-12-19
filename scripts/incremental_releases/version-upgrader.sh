#!/bin/sh
#
# Copyright (c) 2015
# Author: Victor Arribas <v.arribas.urjc@gmail.com>
# License: GPLv3 <http://www.gnu.org/licenses/gpl-3.0.html>
#
# Usage:
# version-upgrader.sh <package name|regex> <current version>


# input parameters
pkg=${1-jderobot*}
version=${2-5.3.0}


## Gather all possible versions from repository
versions=$(apt-cache show $pkg 2>&1 | grep Version | awk '{print $2}')

echo "$versions" | grep -q "$version"
if [ $? -ne 0 ]
then
  ## Case 1: this version is not published into repository.
  # Just add '-rc1' because is initial release
  out_version=${version}-rc1
else
  ## Case 2: this version is already into repository.
  # look for '-rcX' prefix and increment it
  repo_last=$(echo "$versions" | grep "$version" | grep -- '-rc' | sort -nr | head -n 1)
  number=${repo_last#${version}-rc}
  number=$((number + 1))
  out_version=${version}-rc${number}
fi

echo -n $out_version

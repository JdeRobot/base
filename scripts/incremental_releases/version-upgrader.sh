#!/bin/sh
#
# Copyright (c) 2015
# Author: Victor Arribas <v.arribas.urjc@gmail.com>
# License: GPLv3 <http://www.gnu.org/licenses/gpl-3.0.html>
#
# Usage:
# version-upgrader.sh <package name|regex> <current version>


# input parameters
pkg=${1}
version=${2}

[ "$pkg" = "" ] && return 2
[ "$version" = "" ] && return 2

status=0

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
  versions_rc_numbers=$(echo "$versions" | grep "$version" | grep -- '-rc' | sed "s,$version-rc\([0-9]+\)*,\1,")
  number=$(echo "$versions_rc_numbers" | sort -nr | head -n 1)
  [ "$number" = "" ] && status=1
  number=$((number + 1))
  out_version=${version}-rc${number}
fi

[ "$out_version" = "" ] && status=1
echo -n $out_version

return $status

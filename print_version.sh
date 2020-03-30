#!/bin/bash

GIT_BRANCH=$(git branch | grep "*" | sed -e 's/^* //')
GIT_VERSION_HASH=$(git describe --tags --long --dirty --always)
GIT_DATE=$(date +"%Y-%m-%d-%H:%M:%S")
export GIT_VERSION_INFO=$GIT_BRANCH/$GIT_VERSION_HASH/$GIT_DATE
echo "#define GIT_VERSION \"$GIT_VERSION_INFO\"" > ../curr_git_version.h

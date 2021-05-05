#!/bin/bash

# CMAKE_SOURCE_DIR with .git folder
SDIR=$1
# CMAKE_CURRENT_BINARY_DIR
BDIR=$2

# Extracting git data
pushd $SDIR > /dev/null
GIT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
GIT_COMMIT_HASH=$(git log -1 --format=%h)
GIT_MOD_UNTRACKED_FILES=$(git status --porcelain | wc -l)
GIT_CLEAN=false
if [ "x$GIT_MOD_UNTRACKED_FILES" == "x0" ]; then
  GIT_CLEAN=true
fi
popd > /dev/null

GIT_INFO_HPP=$BDIR/git_info.hpp
GIT_INFO_TEXT=$BDIR/git_info.txt

# Update status file
GIT_STATUS_FILE=$BDIR/_gitstatus
GIT_OLD_STATUS="GIT_STATUS-NOTFOUND"
if [ -f "$GIT_STATUS_FILE" ]; then
  GIT_OLD_STATUS=`cat $GIT_STATUS_FILE`
fi
GIT_STATUS=$GIT_BRANCH-$GIT_COMMIT_HASH-$GIT_CLEAN-$GIT_MOD_UNTRACKED_FILES

if [ ! -f "$GIT_STATUS_FILE" ] || [ "x$GIT_STATUS" != "x$GIT_OLD_STATUS" ]; then
  #echo "Updating git status to ${GIT_STATUS}"
  echo "$GIT_STATUS" > $GIT_STATUS_FILE

  # Output .hpp
  cat << EOF > $GIT_INFO_HPP
#pragma once
#include <ostream>
#include <string>

class GitInfo
{
  public:
    static const uint32_t    hash_{0x$GIT_COMMIT_HASH};
    static const bool        clean_{$GIT_CLEAN};
    static const std::string branch_;
    static const std::string user_;
    static const std::string hostname_;

    friend std::ostream&
    operator<<(std::ostream& os, const GitInfo& g)
      {
        os << "Version information\n"
           << "  branch : " << g.branch_ << "\n"
           << "  hash   : " << std::hex << g.hash_ << std::dec << "\n"
           << "  clean  : " << (g.clean_ ? "yes" : "no") << "\n"
           << "  build  : " << g.user_ << "@" << g.hostname_ << "\n";
        return os;
      }
};

const std::string GitInfo::branch_ = {"$GIT_BRANCH"};
const std::string GitInfo::user_ = {"$USER"};
const std::string GitInfo::hostname_ = {"$HOSTNAME"};
EOF

  # Output .txt
  if [ "$GIT_CLEAN" == "true" ]; then
    GIT_CLEAN_STRING="yes"
  else
    GIT_CLEAN_STRING="no"
  fi
  cat << EOF > $GIT_INFO_TEXT
Version information
  branch : $GIT_BRANCH
  hash   : $GIT_COMMIT_HASH
  clean  : $GIT_CLEAN_STRING
  build  : $USER@$HOSTNAME
EOF
fi

if [ ! -z $3 ]; then
cat $GIT_INFO_TEXT
fi


# Dump current status
# echo $GIT_STATUS

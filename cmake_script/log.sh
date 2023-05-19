#!/bin/bash

function log_info() {
  echo -e "\033[32m[INFO] $*\033[0m"
}

function log_warning() {
  echo -e "\033[33m[WARNING] $*\033[0m"
}

function log_error() {
  echo -e "\033[31m[ERROR] $*\033[0m"
}
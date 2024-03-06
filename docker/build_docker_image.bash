#!/bin/bash
set -e
docker build -t mav_exp --build-arg SSH_PRIV_KEY="$(cat ~/.ssh/id_rsa)" .

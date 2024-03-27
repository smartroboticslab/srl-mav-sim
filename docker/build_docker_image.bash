#!/bin/bash
set -e
docker build -t srl_mav_sim --build-arg SSH_PRIV_KEY="$(cat ~/.ssh/id_rsa)" .

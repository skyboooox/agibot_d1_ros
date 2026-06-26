#!/usr/bin/env bash
set -eo pipefail

export AGIBOT_D1_LOCAL_IP="${AGIBOT_D1_LOCAL_IP:-127.0.0.1}"
export AGIBOT_D1_ROBOT_IP="${AGIBOT_D1_ROBOT_IP:-127.0.0.1}"

# shellcheck source=setup.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/setup.sh"

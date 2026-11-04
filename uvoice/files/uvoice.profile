# SPDX-License-Identifier: GPL-2.0-only
#
# uvoice - native voice stack for OpenWrt
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
#
# Licensed under GPL-2.0-only with the linking exception stated in LICENSE.
#
# /etc/profile sources this file at login. The helpers of uvoice.sh are
# loaded on demand, because every login shell would otherwise parse them.
# The name follows the uvoice.<verb> namespace of the helpers; busybox ash
# and bash accept it.

# shellcheck disable=SC3033,SC1091
uvoice.load() { . /usr/share/uvoice/uvoice.sh; }

#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only
#
# uvoice - native voice stack for OpenWrt
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
#
# Licensed under GPL-2.0-only with the linking exception stated in LICENSE.
#
# voip-defaults.sh: the default voice config of uvoice. A caller sources
# /lib/functions.sh first, then this file; do not execute it.
#
# Every write is unset-only: a value already in /etc/config/voip stays, and a
# section keeps its type. The writers queue uci batch lines and record each
# value with config_set, so a later config_get of the caller sees it.
# voip_defaults_commit applies the queue in one batch, then commits voip. A
# function returns nonzero when its load or uci call fails, or when it refuses
# a value, so a caller can stop at the first failure.

voip_defaults_queue=''
voip_defaults_loaded=0

voip_defaults_queue_add() {
	voip_defaults_queue="${voip_defaults_queue}$1
"
}

voip_defaults_load() {
	[ "$voip_defaults_loaded" = 1 ] && return 0
	if [ ! -e /etc/config/voip ]; then
		touch /etc/config/voip || return
	fi
	config_load voip || return
	voip_defaults_loaded=1
}

# A single quote or a newline in a value would end its quoted batch line
# early, and the rest would run as batch input.
voip_defaults_value_check() {
	case "$1" in
	*"'"* | *"
"*)
		return 1
		;;
	esac
}

section_default_write() {
	voip_defaults_value_check "$2" || return 1
	voip_defaults_value=''
	config_get voip_defaults_value "$1" TYPE
	[ -n "$voip_defaults_value" ] && return 0
	voip_defaults_queue_add "set voip.$1='$2'"
	config_set "$1" TYPE "$2"
}

# An empty value is never queued: libuci keeps no empty option.
option_default_write() {
	[ -n "$3" ] || return 0
	voip_defaults_value_check "$3" || return 1
	voip_defaults_value=''
	config_get voip_defaults_value "$1" "$2"
	[ -n "$voip_defaults_value" ] && return 0
	voip_defaults_queue_add "set voip.$1.$2='$3'"
	config_set "$1" "$2" "$3"
}

# rows_default_write section name=value...: one option per row. The name ends
# at the first "=".
rows_default_write() {
	voip_defaults_section=$1
	shift
	for voip_defaults_row in "$@"; do
		option_default_write "$voip_defaults_section" \
			"${voip_defaults_row%%=*}" "${voip_defaults_row#*=}" ||
			return
	done
}

base_defaults_write() {
	section_default_write base base || return
	rows_default_write base enabled=0 "channels=$1" local_interface=wan \
		region=US alert_info_prefix=Bellcore-dr mwi_subscribe=0 \
		star_codes_local=0 options_interval=0 \
		active_source_filter_enabled=0 transport=udp srtp_enabled=0 \
		keepalive_enabled=1 keepalive_interval=60 \
		dns_cache_update_mode=1 rtcp_enabled=1 ipv6_enabled=0
}

# The RTP range starts on an even port, as RTP needs (RFC 3550). The digit map
# is the base64 of '*xx.T|x.T'.
channel_defaults_write() {
	voip_defaults_id=$(($1 - 1))
	voip_defaults_rtp_min=$((20000 + 10 * voip_defaults_id))
	section_default_write "channel$1" channel || return
	rows_default_write "channel$1" "id=$voip_defaults_id" "ux_id=$1" \
		enabled=0 sip_registration_expiry=3600 sip_register_retry=5 \
		sip_local_port=5060 \
		"rtp_local_port_min=$voip_defaults_rtp_min" \
		"rtp_local_port_max=$((voip_defaults_rtp_min + 8))" \
		sip_tos=104 rtp_tos=184 rtcp_tos=184 \
		digit_map_base64=Knh4LlR8eC5U digit_esc_enabled=1 \
		sip_dtmf_mode=inband sip_flash_relay=info \
		sip_session_refresher=uac prack=0 fax_t38_enable=0 cn_enable=0 \
		interdigit_timeout=4 interdigit_long=16 start_digit_timer=16 \
		roh_timer_sec=60 ps_rx_gain=-9 ps_tx_gain=-3 half_db_rx_loss=0 \
		half_db_tx_loss=0 local_conf_enabled=1 amwi=1 vmwi=1 \
		warmline_enable=0 warmline_timeout=8 call_waiting_enabled=1 \
		cid_remote_enable=1 cid_outgoing_enable=1
}

# codec_default_write n codec name id bitrate packetization silence priority
# fec: an empty fec writes no fec row.
codec_default_write() {
	section_default_write "channel${1}_$2" codec || return
	rows_default_write "channel${1}_$2" "tr104_name=$3" "tr104_id=$4" \
		"bitrate=$5" "fec=$9" "packetization=$6" \
		"silence_suppression=$7" "priority=$8" "default_priority=$8" \
		enabled=1 "channel=$1"
}

# The HAL codec mask gates the offer, so a backend without a codec ignores
# its section.
codec_defaults_write() {
	codec_default_write "$1" g711u G.711MuLaw 1 64000 10,20,30,40 0 1 '' ||
		return
	codec_default_write "$1" g711a G.711ALaw 2 64000 10,20,30,40 0 2 '' ||
		return
	codec_default_write "$1" g729a G.729a 3 8000 10,20,30,40 1 3 '' ||
		return
	codec_default_write "$1" g726 G.726 4 32000 10,20,30,40 0 4 '' || return
	codec_default_write "$1" g722 G.722 5 64000 10,20,30 0 5 '' || return
	codec_default_write "$1" opus OPUS 6 32000 10,20 0 6 1
}

voip_defaults_commit() {
	if [ -n "$voip_defaults_queue" ]; then
		printf '%s' "$voip_defaults_queue" | uci -q batch || return
		voip_defaults_queue=''
	fi
	uci -q commit voip
}

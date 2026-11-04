#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only
#
# uvoice - native voice stack for OpenWrt
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
#
# Licensed under GPL-2.0-only with the linking exception stated in LICENSE.
#
#
# run_defaults_tests.sh: the voice config defaults library (voip-defaults.sh)
# and the uci-defaults step (uvoice.defaults) against a recording uci.
#
# The step sources /lib/functions.sh and /usr/share/uvoice/voip-defaults.sh,
# reads /tmp/sysinfo/board_name and writes /etc/config/voip; a build host has
# none of them. The suite re-enters itself under "unshare -rm" (a user and a
# mount namespace) and puts an unprivileged overlay over /usr and /etc, so the
# shipped files run from their install paths. The upper-layer directories
# exist before the mount, because the merged view refuses a new directory
# under a lower directory that an unmapped uid owns. /lib is a symlink to
# usr/lib on current hosts, so /lib/functions.sh lands through the /usr
# overlay. A directory this suite owns is bound over /tmp/sysinfo; when the
# host has no /tmp/sysinfo, the suite creates the mount point and removes it
# at the end.
#
# The functions.sh stub keeps the OpenWrt semantics the library relies on:
# config_set exports CONFIG_<section>_<option>, config_get reads it back,
# config_load fills it from the uci state and records the option TYPE of each
# section, and board_name reads /tmp/sysinfo/board_name. config_load fails
# when /etc/config/voip is absent, as "uci export" does.
#
# The uci shim keeps the option state in a file and records every call, and
# each batch line as "batch:<line>". Shim fidelity: a set of an option on a
# missing section fails, an empty value unsets, and a batch line that fails
# prints an error, lands in the record as "error:<line>" and does not stop the
# batch, whose exit status stays 0, as in the uci command line. A commit fails
# when /etc/config/voip is absent. UCI_FAIL=batch or UCI_FAIL=commit makes the
# named call exit 1, the way a uci that cannot run does.
#
# Every leg runs under dash, busybox sh and bash, as far as the host has them.
# The step runs the way uci_apply_defaults runs it: sourced in a subshell from
# /etc/uci-defaults.
#
#   VOIP_DEFAULTS_LIB     the library under test
#                         (default ../../files/voip-defaults.sh)
#   UVOICE_DEFAULTS_STEP  the step under test
#                         (default ../../files/uvoice.defaults)
#
# Output is TAP: one "ok"/"not ok" line per check, a trailing plan line, and a
# non-zero exit when any check fails.

here=$(cd "$(dirname "$0")" && pwd)
uvoice_dir=$(cd "$here/../.." && pwd)
defaults_lib=${VOIP_DEFAULTS_LIB:-$uvoice_dir/files/voip-defaults.sh}
defaults_step=${UVOICE_DEFAULTS_STEP:-$uvoice_dir/files/uvoice.defaults}
export here uvoice_dir defaults_lib defaults_step

if [ "${DEFAULTS_TESTS_INNER:-}" != 1 ]; then
	DEFAULTS_TESTS_INNER=1
	export DEFAULTS_TESTS_INNER
	exec unshare -rm "$0" "$@"
fi

bail() {
	printf 'Bail out! %s\n' "$*"
	exit 1
}

[ -r "$defaults_lib" ] || bail "no defaults library at $defaults_lib"
[ -r "$defaults_step" ] || bail "no defaults step at $defaults_step"
[ "$(readlink /lib)" = usr/lib ] || bail '/lib is not a usr/lib symlink'

tmp=$(mktemp -d) || bail "mktemp failed"
sysinfo_made=0

cleanup() {
	cd /
	umount -l /tmp/sysinfo 2>/dev/null
	[ "$sysinfo_made" = 1 ] && rmdir /tmp/sysinfo
	umount -l /etc 2>/dev/null
	umount -l /usr 2>/dev/null
	rm -rf "$tmp"
}
trap 'cleanup' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

mkdir -p "$tmp/usr-up/lib" "$tmp/usr-up/share/uvoice" "$tmp/usr-work" \
	"$tmp/etc-up/config" "$tmp/etc-up/uci-defaults" "$tmp/etc-work" \
	"$tmp/sysinfo" "$tmp/bin" "$tmp/rec"

UCI_STATE=$tmp/rec/state
UCI_REC=$tmp/rec/uci.argv
LOAD_REC=$tmp/rec/config_load
export UCI_STATE UCI_REC LOAD_REC

cat > "$tmp/usr-up/lib/functions.sh" <<'STUB'
board_name() {
	[ -e /tmp/sysinfo/board_name ] && cat /tmp/sysinfo/board_name ||
		echo generic
}
config_set() {
	export "CONFIG_${1}_${2}=${3}"
}
config_get() {
	case "$2${3:-$1}" in
	*[!A-Za-z0-9_]*) : ;;
	*)
		case "$3" in
		"") eval echo "\"\${CONFIG_${1}_${2}:-\${4}}\"" ;;
		*) eval export -- "${1}=\${CONFIG_${2}_${3}:-\${4}}" ;;
		esac
		;;
	esac
}
config_load() {
	local line key sect
	printf '%s\n' "$1" >> "$LOAD_REC"
	[ -z "$CONFIG_LOAD_FAIL" ] || return 1
	[ -e "/etc/config/$1" ] || return 1
	while IFS= read -r line; do
		key=${line%%=*}
		case "$key" in
		"$1".*.*)
			sect=${key#"$1".}
			config_set "${sect%%.*}" "${sect#*.}" "${line#*=}"
			;;
		"$1".*)
			config_set "${key#"$1".}" TYPE "${line#*=}"
			;;
		esac
	done < "$UCI_STATE"
}
STUB

cat > "$tmp/bin/uci" <<'SHIM'
#!/bin/sh
printf '%s\n' "$*" >> "$UCI_REC"
quiet=
[ "$1" = -q ] && quiet=1 && shift
state_has() {
	awk -v k="$1=" 'index($0, k) == 1 { f = 1 } END { exit !f }' \
		"$UCI_STATE"
}
# state_set key=value: 0 when applied, 1 when the section or the config is
# missing
state_set() {
	k=${1%%=*}
	v=${1#*=}
	case "$v" in
	\'*\') v=${v#\'}; v=${v%\'} ;;
	\"*\") v=${v#\"}; v=${v%\"} ;;
	esac
	[ -e "/etc/config/${k%%.*}" ] || return 1
	case "$k" in
	*.*.*) state_has "${k%.*}" || return 1 ;;
	esac
	awk -v k="$k=" 'index($0, k) != 1' "$UCI_STATE" > "$UCI_STATE.new"
	[ -n "$v" ] && printf '%s=%s\n' "$k" "$v" >> "$UCI_STATE.new"
	mv "$UCI_STATE.new" "$UCI_STATE"
}
case "$1" in
set)
	if ! state_set "$2"; then
		[ "$quiet" = 1 ] || printf 'uci: Invalid argument\n' >&2
		exit 1
	fi
	;;
commit)
	[ "$UCI_FAIL" = commit ] && exit 1
	[ -e "/etc/config/$2" ] || exit 1
	;;
batch)
	[ "$UCI_FAIL" = batch ] && exit 1
	while IFS= read -r line; do
		[ -n "$line" ] || continue
		printf 'batch:%s\n' "$line" >> "$UCI_REC"
		case "$line" in
		'set '*)
			state_set "${line#set }" && continue
			;;
		'commit '*)
			[ -e "/etc/config/${line#commit }" ] && continue
			;;
		esac
		printf 'error:%s\n' "$line" >> "$UCI_REC"
		[ "$quiet" = 1 ] || printf 'uci: Invalid argument\n' >&2
	done
	;;
*)
	exit 2
	;;
esac
SHIM
chmod +x "$tmp/bin/uci"

mount -t overlay overlay \
	-o "lowerdir=/usr,upperdir=$tmp/usr-up,workdir=$tmp/usr-work" /usr ||
	bail 'cannot overlay /usr: the kernel refuses an unprivileged mount'
mount -t overlay overlay \
	-o "lowerdir=/etc,upperdir=$tmp/etc-up,workdir=$tmp/etc-work" /etc ||
	bail 'cannot overlay /etc: the kernel refuses an unprivileged mount'
[ -r /lib/functions.sh ] || bail '/lib/functions.sh is not in the overlay'
if [ ! -d /tmp/sysinfo ]; then
	mkdir /tmp/sysinfo || bail 'cannot create /tmp/sysinfo'
	sysinfo_made=1
fi
mount --bind "$tmp/sysinfo" /tmp/sysinfo || bail 'cannot bind /tmp/sysinfo'

lib_path=/usr/share/uvoice/voip-defaults.sh
hook_path=/usr/share/uvoice/platform.sh
step_path=/etc/uci-defaults/90-uvoice
voip_cfg=/etc/config/voip
cp "$defaults_lib" "$lib_path" || bail 'cannot install the defaults library'

n=0
failed=0

ok() {
	n=$((n + 1))
	printf 'ok %d - %s\n' "$n" "$1"
}

not_ok() {
	n=$((n + 1))
	failed=$((failed + 1))
	printf 'not ok %d - %s\n' "$n" "$1"
}

check() {
	if [ "$2" = "$3" ]; then
		ok "$1"
	else
		not_ok "$1 (got '$2', want '$3')"
	fi
}

state_of() {
	awk -v k="$1=" 'index($0, k) == 1 { print substr($0, length(k) + 1) }' \
		"$UCI_STATE"
}

# The type line and the option lines of one section.
section_lines() {
	grep -c "^voip\.$1[.=]" "$UCI_STATE"
}

section_count() {
	grep -c '^voip\.[^.]*=' "$UCI_STATE"
}

rec_lines() {
	wc -l < "$UCI_REC" | tr -d ' '
}

rec_count() {
	grep -c -E -e "$1" "$UCI_REC"
}

load_calls() {
	wc -l < "$LOAD_REC" | tr -d ' '
}

# order_check label first second: the first record line that matches the
# pattern first comes before the first one that matches second.
order_check() {
	a=$(grep -n -m 1 -e "$2" "$UCI_REC" | cut -d: -f1)
	b=$(grep -n -m 1 -e "$3" "$UCI_REC" | cut -d: -f1)
	if [ -n "$a" ] && [ -n "$b" ] && [ "$a" -lt "$b" ]; then
		ok "$1"
	else
		not_ok "$1 (record lines '$a' and '$b')"
	fi
}

# defaults_fixture board present [preset...]: board GENERIC removes
# /tmp/sysinfo/board_name; present 1 creates /etc/config/voip, 0 removes it;
# every preset is one "key=value" line of prior uci state. The step is
# installed afresh, the platform hook removed and the failure knobs cleared.
defaults_fixture() {
	board=$1
	present=$2
	shift 2

	: > "$UCI_STATE"
	: > "$UCI_REC"
	: > "$LOAD_REC"
	for kv in "$@"; do
		printf '%s\n' "$kv" >> "$UCI_STATE"
	done
	rm -f "$voip_cfg" "$hook_path" /tmp/sysinfo/board_name
	[ "$board" = GENERIC ] ||
		printf '%s\n' "$board" > /tmp/sysinfo/board_name
	[ "$present" = 1 ] && : > "$voip_cfg"
	cp "$defaults_step" "$step_path"
	unset UCI_FAIL CONFIG_LOAD_FAIL
}

# hook_install answer: a platform hook whose uvoice_stack_get answers the
# argument.
hook_install() {
	printf 'uvoice_stack_get() { echo %s; }\n' "$1" > "$hook_path"
}

step_run() {
	shell=$1
	(cd /etc/uci-defaults &&
		PATH="$tmp/bin:$PATH" $shell -c '( . "./90-uvoice" )') \
		>/dev/null 2>&1
	rc=$?
}

# lib_run shell script: sources functions.sh and the installed library, then
# runs the script; out is its output.
lib_run() {
	out=$(PATH="$tmp/bin:$PATH" $1 -c \
		". /lib/functions.sh; . $lib_path; $2" 2>&1)
	rc=$?
}

account_options='username|auth_username|secret|proxy|outbound_proxy'
account_options="$account_options|registrar|domain|realm"

account_writes() {
	rec_count "^batch:set voip\.channel[0-9]+\.sip_($account_options)="
}

base_rows='enabled=0 channels=2 local_interface=wan region=US
alert_info_prefix=Bellcore-dr mwi_subscribe=0 star_codes_local=0
options_interval=0 active_source_filter_enabled=0 transport=udp
srtp_enabled=0 keepalive_enabled=1 keepalive_interval=60
dns_cache_update_mode=1 rtcp_enabled=1 ipv6_enabled=0 ui=desk'

base_check() {
	shell=$1

	check "$shell snom: base section" "$(state_of voip.base)" base
	for kv in $base_rows; do
		check "$shell snom: base ${kv%%=*}" \
			"$(state_of "voip.base.${kv%%=*}")" "${kv#*=}"
	done
	check "$shell snom: base, 18 lines" "$(section_lines base)" 18
}

channel_check() {
	shell=$1
	line=$2

	id=$((line - 1))
	rtp_min=$((20000 + 10 * id))
	check "$shell snom: channel$line section" \
		"$(state_of "voip.channel$line")" channel
	for kv in "id=$id" "ux_id=$line" enabled=0 \
			sip_registration_expiry=3600 sip_register_retry=5 \
			sip_local_port=5060 "rtp_local_port_min=$rtp_min" \
			"rtp_local_port_max=$((rtp_min + 8))" \
			digit_map_base64=Knh4LlR8eC5U sip_dtmf_mode=inband \
			ps_rx_gain=-9 ps_tx_gain=-3 warmline_timeout=8 \
			cid_outgoing_enable=1; do
		check "$shell snom: channel$line ${kv%%=*}" \
			"$(state_of "voip.channel$line.${kv%%=*}")" "${kv#*=}"
	done
	check "$shell snom: channel$line, 36 lines" \
		"$(section_lines "channel$line")" 36
	for codec in g711u g711a g729a g726 g722; do
		check "$shell snom: channel${line}_$codec, 10 lines" \
			"$(section_lines "channel${line}_$codec")" 10
		check "$shell snom: channel${line}_$codec channel" \
			"$(state_of "voip.channel${line}_$codec.channel")" \
			"$line"
	done
	check "$shell snom: channel${line}_opus, 11 lines" \
		"$(section_lines "channel${line}_opus")" 11
	check "$shell snom: channel${line}_opus fec" \
		"$(state_of "voip.channel${line}_opus.fec")" 1
	check "$shell snom: channel${line}_g722 packetization" \
		"$(state_of "voip.channel${line}_g722.packetization")" 10,20,30
	check "$shell snom: channel${line}_g729a silence_suppression" \
		"$(state_of "voip.channel${line}_g729a.silence_suppression")" 1
	order_check "$shell snom: channel$line codecs before channel$line" \
		"^batch:set voip\.channel${line}_opus=" \
		"^batch:set voip\.channel$line="
}

step_legs() {
	shell=$1

	$shell -n "$defaults_lib" && ok "$shell -n accepts the library" ||
		not_ok "$shell -n accepts the library"
	$shell -n "$defaults_step" && ok "$shell -n accepts the step" ||
		not_ok "$shell -n accepts the step"

	defaults_fixture snom,d735 0
	step_run "$shell"
	check "$shell snom: rc 0" "$rc" 0
	[ -f "$voip_cfg" ] && ok "$shell snom: file created" ||
		not_ok "$shell snom: file created"
	base_check "$shell"
	channel_check "$shell" 1
	channel_check "$shell" 2
	check "$shell snom: 15 sections" "$(section_count)" 15
	check "$shell snom: one batch" "$(rec_count '^-q batch$')" 1
	check "$shell snom: one commit" "$(rec_count '^-q commit voip$')" 1
	order_check "$shell snom: batch before commit" '^-q batch$' \
		'^-q commit voip$'
	check "$shell snom: no failed batch line" "$(rec_count '^error:')" 0
	check "$shell snom: no account option" "$(account_writes)" 0
	check "$shell snom: no star code" "$(rec_count 'exten=')" 0
	check "$shell snom: no cwmp_alias" "$(rec_count 'cwmp_alias')" 0
	check "$shell snom: no warmline_number" \
		"$(rec_count 'warmline_number')" 0
	check "$shell snom: one config_load" "$(load_calls)" 1

	: > "$UCI_REC"
	step_run "$shell"
	check "$shell snom repeat: rc 0" "$rc" 0
	check "$shell snom repeat: no batch" "$(rec_count '^-q batch$')" 0
	check "$shell snom repeat: one commit" \
		"$(rec_count '^-q commit voip$')" 1

	defaults_fixture snom,d735 1 voip.base=base voip.base.enabled=1 \
		voip.base.region=DE voip.channel1=channel \
		voip.channel1.sip_username=1001 voip.channel1.ps_rx_gain=-6 \
		voip.channel2=line voip.channel2_g722=codec \
		voip.channel2_g722.priority=9
	step_run "$shell"
	check "$shell preset: rc 0" "$rc" 0
	check "$shell preset: base enabled kept" \
		"$(state_of voip.base.enabled)" 1
	check "$shell preset: base region kept" \
		"$(state_of voip.base.region)" DE
	check "$shell preset: account option kept" \
		"$(state_of voip.channel1.sip_username)" 1001
	check "$shell preset: channel1 gain kept" \
		"$(state_of voip.channel1.ps_rx_gain)" -6
	check "$shell preset: g722 priority kept" \
		"$(state_of voip.channel2_g722.priority)" 9
	check "$shell preset: channel2 type kept" \
		"$(state_of voip.channel2)" line
	opts='base\.enabled|base\.region|channel1\.ps_rx_gain'
	opts="$opts|channel2_g722\.priority"
	check "$shell preset: no set of a preset option" \
		"$(rec_count "^batch:set voip\.($opts)=")" 0
	sects='base|channel1|channel2|channel2_g722'
	check "$shell preset: no set of a preset section" \
		"$(rec_count "^batch:set voip\.($sects)=")" 0
	check "$shell preset: missing base option filled" \
		"$(state_of voip.base.channels)" 2
	check "$shell preset: missing channel option filled" \
		"$(state_of voip.channel1.id)" 0
	check "$shell preset: missing codec section filled" \
		"$(state_of voip.channel2_opus)" codec
	check "$shell preset: no failed batch line" "$(rec_count '^error:')" 0
	check "$shell preset: no account option" "$(account_writes)" 0

	for board in example,other-board GENERIC; do
		defaults_fixture "$board" 0
		step_run "$shell"
		check "$shell board $board: rc 0" "$rc" 0
		check "$shell board $board: no uci call" "$(rec_lines)" 0
		check "$shell board $board: no config_load" "$(load_calls)" 0
		[ -e "$voip_cfg" ] && not_ok "$shell board $board: no file" ||
			ok "$shell board $board: no file"
	done

	defaults_fixture snom,d735 0
	hook_install unknown
	step_run "$shell"
	check "$shell hook unknown: rc 1" "$rc" 1
	check "$shell hook unknown: no uci call" "$(rec_lines)" 0
	[ -e "$voip_cfg" ] && not_ok "$shell hook unknown: no file" ||
		ok "$shell hook unknown: no file"

	for stack in d2 none; do
		defaults_fixture snom,d735 0
		hook_install "$stack"
		step_run "$shell"
		check "$shell hook $stack: rc 0" "$rc" 0
		check "$shell hook $stack: no uci call" "$(rec_lines)" 0
		[ -e "$voip_cfg" ] && not_ok "$shell hook $stack: no file" ||
			ok "$shell hook $stack: no file"
	done

	defaults_fixture snom,d735 0
	hook_install uvoice
	step_run "$shell"
	check "$shell hook uvoice: rc 0" "$rc" 0
	check "$shell hook uvoice: 15 sections" "$(section_count)" 15

	defaults_fixture snom,d735 0
	UCI_FAIL='batch'
	export UCI_FAIL
	step_run "$shell"
	check "$shell failing batch: rc 1" "$rc" 1
	check "$shell failing batch: no commit" \
		"$(rec_count '^-q commit voip$')" 0

	defaults_fixture snom,d735 0
	UCI_FAIL='commit'
	export UCI_FAIL
	step_run "$shell"
	check "$shell failing commit: rc 1" "$rc" 1

	defaults_fixture snom,d735 0
	CONFIG_LOAD_FAIL=1
	export CONFIG_LOAD_FAIL
	step_run "$shell"
	check "$shell failing config_load: rc 1" "$rc" 1
	check "$shell failing config_load: no uci call" "$(rec_lines)" 0
	unset CONFIG_LOAD_FAIL
}

lib_legs() {
	shell=$1

	defaults_fixture GENERIC 0
	lib_run "$shell" 'voip_defaults_load; s1=$?; voip_defaults_load
		echo "$s1 $?"'
	check "$shell library load: status 0 twice" "$out" '0 0'
	check "$shell library load: one config_load" "$(load_calls)" 1
	[ -f "$voip_cfg" ] && ok "$shell library load: file created" ||
		not_ok "$shell library load: file created"

	defaults_fixture GENERIC 0
	CONFIG_LOAD_FAIL=1
	export CONFIG_LOAD_FAIL
	lib_run "$shell" 'voip_defaults_load; echo "$?"'
	check "$shell library load, failing config_load: status 1" "$out" 1
	unset CONFIG_LOAD_FAIL

	defaults_fixture GENERIC 1 voip.base=base
	lib_run "$shell" 'voip_defaults_load
		option_default_write base empty ""; s1=$?
		option_default_write base enabled 0; s2=$?
		section_default_write channel1 channel; s3=$?
		voip_defaults_commit; echo "$s1 $s2 $s3 $?"'
	check "$shell library writers: status 0" "$out" '0 0 0 0'
	check "$shell library writers: no empty option" \
		"$(rec_count '^batch:set voip\.base\.empty=')" 0
	check "$shell library writers: option written" \
		"$(state_of voip.base.enabled)" 0
	check "$shell library writers: section written" \
		"$(state_of voip.channel1)" channel

	defaults_fixture GENERIC 1 voip.base=base
	lib_run "$shell" 'voip_defaults_load
		option_default_write base quote "a'\''b"; s1=$?
		option_default_write base newline "a
		b"; s2=$?
		section_default_write odd "t'\''x"; s3=$?
		voip_defaults_commit; echo "$s1 $s2 $s3 $?"'
	check "$shell library refusal: quote, newline and quoted type" \
		"$out" '1 1 1 0'
	check "$shell library refusal: no batch" "$(rec_count '^-q batch$')" 0
	check "$shell library refusal: no quote option" \
		"$(state_of voip.base.quote)" ''
	check "$shell library refusal: no odd section" "$(state_of voip.odd)" ''

	defaults_fixture GENERIC 1
	lib_run "$shell" 'voip_defaults_load; voip_defaults_commit; echo "$?"'
	check "$shell library commit, empty queue: status 0" "$out" 0
	check "$shell library commit, empty queue: no batch" \
		"$(rec_count '^-q batch$')" 0
	check "$shell library commit, empty queue: one commit" \
		"$(rec_count '^-q commit voip$')" 1

	for call in batch commit; do
		defaults_fixture GENERIC 1
		UCI_FAIL=$call
		export UCI_FAIL
		lib_run "$shell" 'voip_defaults_load
			base_defaults_write 2; s1=$?
			voip_defaults_commit; echo "$s1 $?"'
		check "$shell library, failing $call: commit status 1" \
			"$out" '0 1'
		unset UCI_FAIL
	done
}

ran=0
for shell in dash 'busybox sh' bash; do
	command -v "${shell%% *}" >/dev/null 2>&1 || continue
	step_legs "$shell"
	lib_legs "$shell"
	ran=1
done
[ "$ran" = 1 ] || bail 'no dash, busybox or bash on this host'

printf '1..%d\n' "$n"
[ "$failed" -eq 0 ]

#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Write the copy of a canceller source that lec_replay_coh links.

usage: patch_coh.py <lec.c> <out.c>

The copy writes one row per frame of ec_run to the file that the
environment variable UDSP_LEC_COH_ROWS names, and nothing when the variable
is unset. The program opens the file on the first frame. A file that does
not open, or a row that does not write, prints the reason with the path and
ends the program with status 2, the I/O error status of lec_replay. Every
instance of the process writes to that one file, so a replay with
--residual mixes the rows of its shadow instance into it: run
lec_replay_coh without that option.

The 15 columns of a row: frames, cohere_run, ncc and share (9 and -1 when
cohere_ncc_get cannot read them), the powers mic, estimate and window of the
frame, guard, the talker ratio, active, window active, double talk, trial
arm and trial end of the frame (the numbers of UDSP_EC_NLMS_TRIAL_ARM_* and
UDSP_EC_NLMS_TRIAL_END_*), and the reference power. A power is a mean square
in int16 units. guard is the window that talker_get compares with the
microphone: the larger of the frame's window and the instance's
window_prev before the frame. A reference feed and a reset move
window_prev, so the copy reads it in ec_run itself.

The edits go at three anchors, each of which must occur exactly once:

- after '#include "lec.h"': stdio.h and the file-static coh_window_prev;
- before 'ec->window_prev = t.window_power;' in ec_run: the copy of
  window_prev into coh_window_prev;
- after 'reading_fill(ec, &t);' at the end of ec_run: the row writer.

A missing anchor ends the program with status 1, so the build fails.
"""
import sys

INCLUDE = '#include "lec.h"\n'
STATIC = '#include <stdio.h>\n\nstatic float coh_window_prev;\n'
PREV = '\tec->window_prev = t.window_power;\n'
PREV_SAVE = '\tcoh_window_prev = ec->window_prev;\n'
FILL = '\treading_fill(ec, &t);\n}\n'
ROWS = r'''	reading_fill(ec, &t);
	{
		static FILE *coh_file;
		static const char *coh_path;
		float ncc = 9.0f;
		float share = -1.0f;
		float guard = coh_window_prev > t.window_power ?
			      coh_window_prev : t.window_power;

		if (!coh_path) {
			coh_path = getenv("UDSP_LEC_COH_ROWS");
			if (!coh_path) {
				coh_path = "";
			} else if (!(coh_file = fopen(coh_path, "w"))) {
				perror(coh_path);
				exit(2);
			}
		}
		if (!cohere_ncc_get(ec, &ncc, &share)) {
			ncc = 9.0f;
			share = -1.0f;
		}
		if (coh_file &&
		    (fprintf(coh_file, "%u,%u,%.4f,%.4f,%.6g,%.6g,%.6g,%.6g,"
			     "%.6g,%d,%d,%d,%u,%u,%.6g\n",
			     (unsigned)ec->frames, ec->cohere_run,
			     (double)ncc, (double)share,
			     (double)t.mic_power, (double)t.estimate_power,
			     (double)t.window_power, (double)guard,
			     (double)ec->talker_ratio, (int)t.active,
			     (int)t.window_active, (int)double_talk_get(ec),
			     (unsigned)ec->reading.trial_arm,
			     (unsigned)ec->reading.trial_end,
			     (double)t.ref_power) < 0 ||
		     fflush(coh_file) != 0)) {
			perror(coh_path);
			exit(2);
		}
	}
}
'''


def replace_once(text, old, new):
    """text with its one occurrence of old replaced by new."""
    count = text.count(old)
    if count != 1:
        sys.exit(f'patch_coh.py: anchor {old.strip()[:60]!r} occurs '
                 f'{count} times, 1 expected')
    return text.replace(old, new)


def patch(text):
    """The source text with the row writer."""
    text = replace_once(text, INCLUDE, INCLUDE + STATIC)
    text = replace_once(text, PREV, PREV_SAVE + PREV)
    return replace_once(text, FILL, ROWS)


def main(argv):
    if len(argv) != 3:
        sys.exit('usage: patch_coh.py <lec.c> <out.c>')
    with open(argv[1]) as f:
        text = f.read()
    with open(argv[2], 'w') as f:
        f.write(patch(text))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))

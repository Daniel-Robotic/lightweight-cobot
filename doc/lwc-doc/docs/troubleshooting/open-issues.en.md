# Open issues after workstation test

Recorded: 2026-09-15

## 1. `cobot update` error

On another computer, `cobot update` failed while installing Python libraries.
The full output was not captured, so the cause is unknown. The next
investigation needs the Python version, distribution/environment, and the
complete log starting at the first error.

## 2. Drive knocking in the `controller` branch

While testing branch `controller` on the real KUKA iiwa, motion to known target
points became jerky and the drives audibly knocked/struck internally. A beep
was also heard at the end of the motion.

Observed conditions:

- motion to `work`, speed `0.1`: knocking continued throughout the motion and a
  beep occurred at the end;
- motion to `home`, speed `0.5`: drive knocking was also present;
- branch `dev` did not show knocking under these conditions; it appeared only
  rarely in earlier tests;
- earlier position plots showed isolated short-lived outliers after which the
  drive could not reach the target in time.

The issue may be related to the FRI/ros2_control implementation in
`controller`. Investigation should check the seven-value position format and
order, FRI timestamps and rate, `COMMANDING_WAIT`/`COMMANDING_ACTIVE`
synchronization, rate limiting, and the source of short-lived outliers. Until
investigated, repeat real-robot tests only with operator supervision and a
ready emergency-stop procedure.

No fix is intentionally included in this record.

# Junction-to-line handoff fix

After a junction route completed, the controller resumed LINE requests using
the generation retired by the deliberate junction pause. Buddy2 rejected those
requests as LINE_PAUSED_NEEDS_NEW_GENERATION, leaving the car stationary.

The controller now advances the authority generation after accepting a matching
completed non-obstacle goal and before returning START_LINE_TRACKING. Zero is
skipped on wraparound. Old requests and duplicate results remain stale; failed
or cancelled goals do not grant resume. Buddy2's pause rejection is unchanged.

This is a shared-controller integration fix. Buddy3 and Buddy5 sources, route
selection, probe/settle timing and motor tuning are unchanged. Without a barcode,
an identified crossroad continues straight and an identified T defaults right.
The probe still needs to identify the real junction correctly.

Both integrated-trial overall timers remain disabled. Physical STOP, watchdog,
command TTL, finite-goal deadlines and fault handling remain active. Production
qualification remains false. The earlier notes describing the junction resume
rejection as unresolved are superseded for this particular handoff.

Firmware compilation checked; automated tests skipped at the user's request.
The private network firmware was also compiled with the same controller change.
Hardware confirmation remains pending; no flash or movement performed by agent.

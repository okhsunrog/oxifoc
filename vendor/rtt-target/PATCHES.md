# Local RTT extension

Source is crates.io `rtt-target` 0.6.2, upstream commit
`be1fee4510d277a0044a12f2e6d0c0ac6adeba78`, subdirectory `rtt-target`.
The upstream MIT license is retained in `LICENSE`.

`UpChannel::write_frame` always selects `NoBlockSkip` and publishes a complete
binary frame with one final write-cursor update. Insufficient space discards the
whole frame. Debugger changes to the channel flags cannot enable blocking on
this path. Existing channel APIs retain their upstream behavior.

OxiFOC supplies its own defmt global logger with the `defmt` feature disabled on
this dependency. Queueing, context routing and loss counters belong to the
application logger.

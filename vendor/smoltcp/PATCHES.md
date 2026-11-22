# Configurable minimum TCP retransmission timeout

Source is crates.io `smoltcp` 0.13.1, upstream commit
`e347a1e2d3ac33c5ce2c0c114e24b85ae23c4897`. The upstream 0BSD license is
retained in `LICENSE-0BSD.txt`. Examples, tests, benches and their
dev-dependencies are not vendored, and `Cargo.toml` omits those targets.
It also allows warnings: Cargo caps lints for a registry crate, and a path
dependency would otherwise print upstream's warnings in every build.

`TCP_MIN_RTO_MS` joins the build-time configuration (`build.rs`, the test
`config` module in `src/lib.rs`) and replaces the fixed 1000 ms lower bound of
the retransmission timeout in `src/socket/tcp.rs`. It is set with the
`SMOLTCP_TCP_MIN_RTO_MS` environment variable and defaults to the RFC 6298
value, so an unconfigured build behaves as upstream.

The C5 bridge streams telemetry over one TCP connection and buffers well
under a second of it. A loss that fast retransmit does not repair (the last
segments of a burst, or a lost retransmission) would otherwise stall the
stream for a full second and overflow that buffer.

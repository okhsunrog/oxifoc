# Component provenance

Only selected source is adapted, not the upstream gallery or application.

| Local source | Upstream | Revision | Changes |
| --- | --- | --- | --- |
| `src/components.rs::Button` | https://github.com/rust-ui/ui/blob/4f5a516e76d75a15a587ab2169352cf34fd318d0/app_crates/registry/src/ui/button.rs | `4f5a516e76d75a15a587ab2169352cf34fd318d0` | Reduced variants; semantic OxiFOC CSS; busy state; no router or class-merging dependency; native button semantics retained. |
| `src/components.rs::Separator` | https://github.com/DioxusLabs/dioxus-components/blob/9a758255ea26e2b20c8cecf4c1c946feb9e71da7/primitives/src/separator.rs | `9a758255ea26e2b20c8cecf4c1c946feb9e71da7` | Horizontal-only API; OxiFOC styling; decorative/semantic roles retained. |

MIT notices are in `licenses/`. Other components are implemented locally.
Upstream changes are reviewed per component; there is no automatic overwrite.

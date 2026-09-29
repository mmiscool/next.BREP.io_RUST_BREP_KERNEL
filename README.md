# next.BREP.io Rust BREP kernel

The public source and publishing repository for the BREP crate family.
Release snapshots are maintained here, and public issues and pull requests are
welcome. See [CONTRIBUTING.md](CONTRIBUTING.md) for the contribution workflow.

Start with [BREP](BREP/README.md), the umbrella crate, or use
[BREP_kernel](BREP_kernel/README.md) directly. The workspace also includes surface
recognition, reconstruction, rendering, CAD application, electronics, automation,
PLM and documentation crates. Each crate has its own README.

```sh
cargo check --workspace
cargo run -p BREP_app --bin brep-app
```

The workspace uses the toolchain in `rust-toolchain.toml`. Desktop rendering
requires the platform's graphics libraries. The documentation generator accepts a
separately supplied user-documentation tree; that tree is not bundled here.

This source snapshot may contain unreleased changes. The versions in manifests
are not a claim that these changes have been published to crates.io.

Read [LICENSE.md](LICENSE.md) and
[THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md) before redistributing.

# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

Releases of the form `0.1.n` do not adhere to [Semantic Versioning](https://semver.org/spec/v2.0.0.html),
that is each release may contain incompatible API changes.

Once the API has stabilized this project will adopt semantic versioning, the first release to do so will be `0.2.0`.

## [Possible future]

## [Unreleased]

### Added

### Changed

### Removed

### Deprecated

### Fixed

### Security

## [0.1.5] - 2026-08-04

### Added

- `#[must_use]` attribute to selected functions.

### Changed

- Updated to vqm 0.1.14.
- Updated to signal-filters 0.1.10.
- Updated to pidsk-controller 0.1.8.
- Improved notch filter state machine.
- Refactored motor mixers to match on enum rather than trait.

### Removed

- `katex-header.html`.
- `allow`s from `lib.rs`.

## [0.1.4] - 2026-05-23

### Changed

- Updated to vqm 0.1.8.
- Updated to signal-filters 0.1.6.
- Updated to pidsk-controller 0.1.5.
- made `serde` an optional feature.
- Made constructors `const` where possible.
- Improved documentation.

## [0.1.3] - 2026-05-10

### Changed

- Updated to pidsk-controller 0.1.3.

## [0.1.2] - 2026-05-06

### Changed

- Updated to latest crates.
- Made `new` functions `const` where possible.

## [0.1.1] - 2026-04-26

### Added

- This changelog
- CONTRIBUTING.md

### Changed

- Added documentation.
- Updated to vqm version [0.1.1]

## [0.1.0] - 2026-04-13

Initial release.

# FEMFX Changelog

## [Unreleased]

### Changed

- Grouped sample task system and FEMFX threading utilities into TL library
- Added calls to TL library functions, removed callbacks
- Enabled both asynchronous and synchronous features in TL library
- Simplified async and graph task interface
- Added features to parallel-for
- Changed some task system configuration, parameters
- Removed vectormath Init functions
- Replaced vectormath mul with multiplication operator
- Switched default vectormath implementation to SIMD
- Added default member initialization
- Removed unused functions and variables
- Added GCC compatibility (no SIMD or hardware thread detection)
- Added FM_SIMD_ENABLED define to compile with or without SIMD features
- Added adjustment of shape parameters for narrow tetrahedra
- Split shape parameters into components to simplify some operations
- Renamed "volume matrix" to "shape matrix"
- Switched shape matrix convention in some cases for consistency
- Simplified some operations by translation of tetrahedra
- Moved eigenvalue modifications out of eigen-decomposition function
- Added missing damping forces
- Removed reset of lambda values in constraint solve using CG
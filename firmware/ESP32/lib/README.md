# `lib/` — internal private libraries

This directory is added to PlatformIO's library search path via
`lib_extra_dirs` in `platformio.ini`. Use it for **private** libraries that
should not be published as standalone PlatformIO libraries but should still
benefit from the LDF (Library Dependency Finder) for include resolution.

Suggested layout for a real library:

```
lib/
└── thunders_math/
    ├── library.json          ← PlatformIO manifest (optional but recommended)
    ├── library.properties    ← PIO legacy metadata
    ├── README.md
    ├── examples/
    │   └── foo.ino
    ├── src/
    │   ├── thunders_math.h
    │   └── thunders_math.cpp
    └── test/
        └── test_thunders_math.cpp
```

Libraries placed here are picked up automatically — no `lib_deps` entry
needed in `platformio.ini`.

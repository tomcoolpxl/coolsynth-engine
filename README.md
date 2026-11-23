# CoolSynth engine

The sound engine of CoolSynth and CoolSynth 808: the V3 synthesizer, its circuit models of the TR-808, TR-909 and
TB-303, the arpeggiator and effects, and the flight recorder they log to. C++20 on JUCE 9.

It moved here from CoolSynth's repository on 2026-11-23, with its history (CoolSynth's decision D265).

## Using it

An app takes it in as a git submodule and adds the folder after JUCE and CTest:

```cmake
include(external/coolsynth-engine/cmake/CoolSynthFlags.cmake)
add_subdirectory(external/JUCE)
include(CTest)
add_subdirectory(external/coolsynth-engine)
target_link_libraries(MyApp PRIVATE coolsynth_engine)
```

- `coolsynth_engine` hands its sources to each program that links it, so JUCE's modules are built once per program.
- `cmake/CoolSynthFlags.cmake` holds the settings every CoolSynth program shares: the C++ runtime, and a Debug
  build compiled optimised so it plays in real time.
- `COOLSYNTH_ENGINE_TESTS` (on by default) builds the engine's own test program, `CoolSynthEngineTests`.

## Building and testing it on its own

```powershell
git clone --recursive https://github.com/tomcoolpxl/coolsynth-engine.git
cmake -S coolsynth-engine -B coolsynth-engine/build
cmake --build coolsynth-engine/build --config Debug --parallel
ctest --test-dir coolsynth-engine/build -C Debug --output-on-failure
```

- `COOLSYNTH_TEST_ONLY=<part of a test's name>` runs only the tests whose names contain it.
- The tests that need presets or the plugin, among them the bit-for-bit determinism test over every preset, are in
  CoolSynth's repository: run them there before taking an engine change into a release.

## Where things are

- `src/synth/v3`: the engine, its generators, processors and effects; `src/synth`: the arpeggiator, the global
  effects and the step sequencer of CoolSynth 808 (`StepSequencer`); `src/diagnostics`: the flight recorder and the
  audio-thread allocation alarm.
- `tools/tr808`, `tools/kick808`, `tools/kick909`: the ngspice netlists the circuit models are checked against, and
  their render scripts; `tests/data`: the renders the tests compare with.
- The design and research notes the code cites (`docs/design/DESIGN_V3.md`, `docs/research/...`) stay in
  CoolSynth's repository: https://github.com/tomcoolpxl/CoolSynth/tree/main/docs

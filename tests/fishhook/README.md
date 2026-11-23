fishhook, from https://github.com/facebook/fishhook at commit aadc161ac3b80db07a9908851839a17ba63a9eb1 (2021-10-12),
unchanged. BSD 3-Clause licence (LICENSE). Used only by the test programs' audio-thread watch on macOS
(`../AudioThreadAlarm.cpp`, CoolSynth's D272): it rebinds the program's own imports of the calls that can block. Not
in any shipped binary.

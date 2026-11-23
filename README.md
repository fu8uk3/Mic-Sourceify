# Mic-Sourceify (experimental MVP)

A real-time Windows microphone loop-through that encodes and decodes audio with the Speex narrowband codec, then sends the result to a virtual cable. It is an experimental reconstruction, not a bit-exact copy of any specific Counter-Strike: Source build.

## Signal path

Physical microphone (48 kHz mono) -> 48-to-8 kHz low-pass/decimation -> Speex NB encode/decode (160 samples / 20 ms) -> 8-to-48 kHz interpolation -> VB-CABLE playback endpoint.

The receiving app selects **CABLE Output** as its microphone. This app selects **CABLE Input** as its audio output.

## Dependencies

- Windows 10/11
- VB-CABLE (install separately)
- Visual Studio 2022 C++ workload
- CMake 3.20+
- vcpkg packages: `portaudio` and `speex`

Install dependencies:

```powershell
vcpkg install portaudio speex
```

Configure/build (replace the vcpkg path if needed):

```powershell
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=C:/src/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
```

## Run

List audio devices:

```powershell
.\build\Release\source-voice.exe --list
```

Start (replace indices with the ones shown by `--list`):

```powershell
.\build\Release\source-voice.exe --input 2 --output 5 --quality 3
```

Press Enter to stop. Use the physical mic as input and **CABLE Input** as output. In the target app, choose **CABLE Output**.

## Notes / limitations

- This first build is a console MVP; the graphical device selector and live controls are not included yet.
- The app expects both selected devices to support 48 kHz, mono, float32 full-duplex audio.
- Speex quality 3 is a starting test setting, not a verified Valve setting. Try `--quality 2`, `3`, `4`, `5` and compare at matched loudness.
- The resampler is a compact 12-tap FIR decimator plus linear interpolation. It is adequate for a first listening test but should be replaced with a higher-quality streaming resampler for a polished release.
- The historical codec and server configuration varied by Source build/server. This does not claim exact reproduction of every 2004–2010 CS:S configuration.
- Do not monitor the CABLE Output through speakers while the mic is open; that can create feedback.

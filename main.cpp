#define NOMINMAX
#include <windows.h>
#include <portaudio.h>
#include <speex/speex.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {
constexpr int kDeviceRate = 48000;
constexpr int kCodecRate = 8000;
constexpr int kFrame = 160;                 // Speex NB: 20 ms at 8 kHz
constexpr unsigned long kCallbackFrames = 960; // 20 ms at 48 kHz

struct Engine {
    void* encoder = nullptr;
    void* decoder = nullptr;
    SpeexBits bits{};
    std::array<spx_int16_t, kFrame> codecIn{};
    std::array<spx_int16_t, kFrame> codecOut{};
    std::array<char, 4096> packet{};
    std::array<float, 12> fir{};
    std::array<float, 12> history{};
    bool active = false;
    int quality = 3;

    explicit Engine(int q) : quality(std::clamp(q, 0, 10)) {
        const SpeexMode* mode = &speex_nb_mode;
        encoder = speex_encoder_init(mode);
        decoder = speex_decoder_init(mode);
        if (!encoder || !decoder) return;

        int v = quality;
        speex_encoder_ctl(encoder, SPEEX_SET_QUALITY, &v);
        v = 0; speex_encoder_ctl(encoder, SPEEX_SET_VBR, &v);
        v = 0; speex_encoder_ctl(encoder, SPEEX_SET_DTX, &v);
        v = 3; speex_encoder_ctl(encoder, SPEEX_SET_COMPLEXITY, &v);
        speex_bits_init(&bits);

        // 12-tap windowed-sinc low-pass, cutoff ~3.4 kHz before /6 decimation.
        constexpr double fc = 3400.0 / 48000.0;
        double sum = 0.0;
        for (int i = 0; i < 12; ++i) {
            const double n = i - 5.5;
            const double sinc = (std::abs(n) < 1e-9) ? 2.0 * fc : std::sin(2.0 * 3.141592653589793 * fc * n) / (3.141592653589793 * n);
            const double window = 0.5 - 0.5 * std::cos(2.0 * 3.141592653589793 * i / 11.0);
            fir[i] = static_cast<float>(sinc * window);
            sum += fir[i];
        }
        for (auto& c : fir) c = static_cast<float>(c / sum);
        active = true;
    }

    ~Engine() {
        if (encoder) speex_encoder_destroy(encoder);
        if (decoder) speex_decoder_destroy(decoder);
        if (active) speex_bits_destroy(&bits);
    }

    void process(const float* input, float* output) {
        if (!active) { std::fill(output, output + kCallbackFrames, 0.0f); return; }

        // 48 kHz -> 8 kHz: FIR anti-alias filter followed by integer decimation.
        for (int n = 0; n < kFrame; ++n) {
            const int sourceIndex = n * 6 + 5;
            float acc = 0.0f;
            for (int t = 0; t < 12; ++t) {
                const int idx = sourceIndex - t;
                float x;
                if (idx >= 0) x = input[idx];
                else x = history[static_cast<size_t>(12 + idx)];
                acc += x * fir[static_cast<size_t>(t)];
            }
            const float clipped = std::clamp(acc, -1.0f, 1.0f);
            codecIn[static_cast<size_t>(n)] = static_cast<spx_int16_t>(std::lrint(clipped * 32767.0f));
        }
        // Preserve the final 12 input samples for the next callback.
        for (int i = 0; i < 12; ++i) history[static_cast<size_t>(i)] = input[kCallbackFrames - 12 + i];

        speex_bits_reset(&bits);
        speex_encode_int(encoder, codecIn.data(), &bits);
        const int bytes = speex_bits_nbytes(&bits);
        if (bytes <= 0 || bytes > static_cast<int>(packet.size())) {
            std::fill(output, output + kCallbackFrames, 0.0f);
            return;
        }
        speex_bits_write(&bits, packet.data(), bytes);
        speex_bits_reset(&bits);
        speex_bits_read_from(&bits, packet.data(), bytes);
        const int decoded = speex_decode_int(decoder, &bits, codecOut.data());
        if (decoded < 0) {
            std::fill(output, output + kCallbackFrames, 0.0f);
            return;
        }

        // 8 kHz -> 48 kHz: linear interpolation. Output is deliberately mono.
        for (int n = 0; n < kFrame; ++n) {
            const float a = static_cast<float>(codecOut[static_cast<size_t>(n)]) / 32768.0f;
            const float b = static_cast<float>(codecOut[static_cast<size_t>(std::min(n + 1, kFrame - 1))]) / 32768.0f;
            for (int j = 0; j < 6; ++j) {
                const float frac = static_cast<float>(j) / 6.0f;
                output[n * 6 + j] = a + (b - a) * frac;
            }
        }
    }
};

int paCallback(const void* inputBuffer, void* outputBuffer,
               unsigned long framesPerBuffer,
               const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags,
               void* userData) {
    auto* engine = static_cast<Engine*>(userData);
    auto* out = static_cast<float*>(outputBuffer);
    const auto* in = static_cast<const float*>(inputBuffer);
    if (!out) return paContinue;
    if (!in || framesPerBuffer != kCallbackFrames) {
        std::fill(out, out + framesPerBuffer, 0.0f);
        return paContinue;
    }
    engine->process(in, out);
    return paContinue;
}

void listDevices() {
    const int count = Pa_GetDeviceCount();
    if (count < 0) { std::cerr << "PortAudio error: " << Pa_GetErrorText(count) << "\n"; return; }
    for (int i = 0; i < count; ++i) {
        const PaDeviceInfo* d = Pa_GetDeviceInfo(i);
        const PaHostApiInfo* h = Pa_GetHostApiInfo(d->hostApi);
        std::cout << i << " | " << (h ? h->name : "?") << " | " << d->name
                  << " | in=" << d->maxInputChannels << " out=" << d->maxOutputChannels
                  << " | defaultRate=" << d->defaultSampleRate << "\n";
    }
}

void printUsage() {
    std::cout << "Source Voice - experimental Source 1 / CS:S voice path emulator\n\n"
              << "Usage:\n"
              << "  source-voice.exe --list\n"
              << "  source-voice.exe --input N --output N [--quality 0..10]\n\n"
              << "Choose your physical microphone as --input and 'CABLE Input' (VB-CABLE) as --output.\n"
              << "The receiving application must select 'CABLE Output' as its microphone.\n";
}
}

int main(int argc, char** argv) {
    int inputDevice = -1, outputDevice = -1, quality = 3;
    bool list = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--list") list = true;
        else if (arg == "--input" && i + 1 < argc) inputDevice = std::atoi(argv[++i]);
        else if (arg == "--output" && i + 1 < argc) outputDevice = std::atoi(argv[++i]);
        else if (arg == "--quality" && i + 1 < argc) quality = std::atoi(argv[++i]);
        else if (arg == "--help" || arg == "-h") { printUsage(); return 0; }
    }

    PaError err = Pa_Initialize();
    if (err != paNoError) { std::cerr << "Pa_Initialize: " << Pa_GetErrorText(err) << "\n"; return 1; }
    if (list) { listDevices(); Pa_Terminate(); return 0; }
    if (inputDevice < 0 || outputDevice < 0) { printUsage(); Pa_Terminate(); return 2; }

    const PaDeviceInfo* inInfo = Pa_GetDeviceInfo(inputDevice);
    const PaDeviceInfo* outInfo = Pa_GetDeviceInfo(outputDevice);
    if (!inInfo || !outInfo || inInfo->maxInputChannels < 1 || outInfo->maxOutputChannels < 1) {
        std::cerr << "Invalid input/output device index or channel capability. Run --list.\n";
        Pa_Terminate(); return 2;
    }

    Engine engine(quality);
    if (!engine.active) { std::cerr << "Could not initialize Speex.\n"; Pa_Terminate(); return 1; }

    PaStreamParameters input{};
    input.device = inputDevice; input.channelCount = 1; input.sampleFormat = paFloat32;
    input.suggestedLatency = inInfo->defaultLowInputLatency;
    input.hostApiSpecificStreamInfo = nullptr;
    PaStreamParameters output{};
    output.device = outputDevice; output.channelCount = 1; output.sampleFormat = paFloat32;
    output.suggestedLatency = outInfo->defaultLowOutputLatency;
    output.hostApiSpecificStreamInfo = nullptr;

    PaStream* stream = nullptr;
    err = Pa_OpenStream(&stream, &input, &output, kDeviceRate, kCallbackFrames,
                        paNoFlag, paCallback, &engine);
    if (err != paNoError) {
        std::cerr << "Pa_OpenStream: " << Pa_GetErrorText(err)
                  << "\nCheck that both devices support 48 kHz mono float, or try another device.\n";
        Pa_Terminate(); return 1;
    }
    err = Pa_StartStream(stream);
    if (err != paNoError) {
        std::cerr << "Pa_StartStream: " << Pa_GetErrorText(err) << "\n";
        Pa_CloseStream(stream); Pa_Terminate(); return 1;
    }

    std::cout << "Source Voice is running. Speex NB, quality " << std::clamp(quality, 0, 10)
              << ". Press ENTER to stop.\n";
    std::cin.get();
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();
    return 0;
}

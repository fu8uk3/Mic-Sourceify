#define NOMINMAX
#include <windows.h>
#include <commctrl.h>
#include <portaudio.h>
#include <speex/speex.h>
#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <cwchar>
#include <vector>

#pragma comment(lib, "Comctl32.lib")

namespace {
constexpr int kDeviceRate = 48000;
constexpr int kCodecRate = 8000;
constexpr int kFrame = 160;                 // Speex NB: 20 ms at 8 kHz
constexpr unsigned long kCallbackFrames = 960; // 20 ms at 48 kHz
constexpr double kSuggestedLatencySeconds = 0.020; // Ask WASAPI for 20 ms per direction.

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
        #undef speex_lib_get_mode
        const SpeexMode* mode = speex_lib_get_mode(SPEEX_MODEID_NB);
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


std::atomic<float> g_inputPeak{0.0f};

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
        g_inputPeak.store(0.0f, std::memory_order_relaxed);
        return paContinue;
    }

    float peak = 0.0f;
    for (unsigned long i = 0; i < framesPerBuffer; ++i)
        peak = std::max(peak, std::abs(in[i]));
    g_inputPeak.store(peak, std::memory_order_relaxed);

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

constexpr int ID_INPUT = 1001;
constexpr int ID_OUTPUT = 1002;
constexpr int ID_QUALITY = 1003;
constexpr int ID_START = 1004;
constexpr int ID_LEVEL = 1005;
constexpr int ID_STATUS = 1006;
constexpr int ID_LATENCY = 1007;
constexpr UINT_PTR TIMER_ID = 1;

struct AppState {
    HWND window = nullptr;
    HWND inputCombo = nullptr;
    HWND outputCombo = nullptr;
    HWND qualitySlider = nullptr;
    HWND qualityValue = nullptr;
    HWND startButton = nullptr;
    HWND levelBar = nullptr;
    HWND statusLabel = nullptr;
    HWND latencyLabel = nullptr;
    PaStream* stream = nullptr;
    std::unique_ptr<Engine> engine;
    bool running = false;
    int quality = 3;
} g_app;

HFONT g_font = nullptr;
HFONT g_titleFont = nullptr;

HWND makeControl(const wchar_t* cls, const wchar_t* text, DWORD style,
                 int x, int y, int w, int h, HWND parent, int id = 0) {
    return CreateWindowExW(0, cls, text, WS_CHILD | WS_VISIBLE | style,
                           x, y, w, h, parent,
                           reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
                           GetModuleHandleW(nullptr), nullptr);
}

void setFont(HWND control, HFONT font = nullptr) {
    SendMessageW(control, WM_SETFONT,
                 reinterpret_cast<WPARAM>(font ? font : g_font), TRUE);
}

void addLabel(HWND parent, const wchar_t* text, int x, int y, int w, int h,
              bool title = false) {
    HWND label = makeControl(L"STATIC", text, SS_LEFT, x, y, w, h, parent);
    setFont(label, title ? g_titleFont : nullptr);
}

std::wstring widen(const char* text) {
    if (!text) return L"";
    const int n = MultiByteToWideChar(CP_UTF8, 0, text, -1, nullptr, 0);
    if (n <= 1) {
        const int m = MultiByteToWideChar(CP_ACP, 0, text, -1, nullptr, 0);
        if (m <= 1) return L"";
        std::wstring result(static_cast<size_t>(m), L'\0');
        MultiByteToWideChar(CP_ACP, 0, text, -1, result.data(), m);
        result.resize(static_cast<size_t>(m - 1));
        return result;
    }
    std::wstring result(static_cast<size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, text, -1, result.data(), n);
    result.resize(static_cast<size_t>(n - 1));
    return result;
}

void populateDevices() {
    SendMessageW(g_app.inputCombo, CB_RESETCONTENT, 0, 0);
    SendMessageW(g_app.outputCombo, CB_RESETCONTENT, 0, 0);

    const int count = Pa_GetDeviceCount();
    int savedInput = GetPrivateProfileIntW(L"Audio", L"InputDevice", -1, L".\\source-voice.ini");
    int savedOutput = GetPrivateProfileIntW(L"Audio", L"OutputDevice", -1, L".\\source-voice.ini");
    int inputPreferred = -1, outputPreferred = -1;
    int inputSavedItem = -1, outputSavedItem = -1;

    for (int i = 0; i < count; ++i) {
        const PaDeviceInfo* d = Pa_GetDeviceInfo(i);
        if (!d) continue;
        const PaHostApiInfo* h = Pa_GetHostApiInfo(d->hostApi);
        const std::wstring host = widen(h ? h->name : "");
        const std::wstring name = widen(d->name);
        const std::wstring display = L"[" + host + L"] " + name;

        if (d->maxInputChannels > 0) {
            const LRESULT item = SendMessageW(g_app.inputCombo, CB_ADDSTRING, 0,
                                              reinterpret_cast<LPARAM>(display.c_str()));
            SendMessageW(g_app.inputCombo, CB_SETITEMDATA, item, static_cast<LPARAM>(i));
            if (i == savedInput) inputSavedItem = static_cast<int>(item);
            if (inputPreferred < 0 &&
                (name.find(L"Xonar") != std::wstring::npos ||
                 (h && h->defaultInputDevice == i)))
                inputPreferred = static_cast<int>(item);
        }
        if (d->maxOutputChannels > 0) {
            const LRESULT item = SendMessageW(g_app.outputCombo, CB_ADDSTRING, 0,
                                              reinterpret_cast<LPARAM>(display.c_str()));
            SendMessageW(g_app.outputCombo, CB_SETITEMDATA, item, static_cast<LPARAM>(i));
            if (i == savedOutput) outputSavedItem = static_cast<int>(item);
            if (outputPreferred < 0 &&
                (name.find(L"CABLE Input") != std::wstring::npos ||
                 (h && h->defaultOutputDevice == i)))
                outputPreferred = static_cast<int>(item);
        }
    }

    const LRESULT inCount = SendMessageW(g_app.inputCombo, CB_GETCOUNT, 0, 0);
    const LRESULT outCount = SendMessageW(g_app.outputCombo, CB_GETCOUNT, 0, 0);
    if (inCount > 0)
        SendMessageW(g_app.inputCombo, CB_SETCURSEL,
                     inputSavedItem >= 0 ? inputSavedItem : (inputPreferred >= 0 ? inputPreferred : 0), 0);
    if (outCount > 0)
        SendMessageW(g_app.outputCombo, CB_SETCURSEL,
                     outputSavedItem >= 0 ? outputSavedItem : (outputPreferred >= 0 ? outputPreferred : 0), 0);
}

int selectedDevice(HWND combo) {
    const LRESULT item = SendMessageW(combo, CB_GETCURSEL, 0, 0);
    if (item == CB_ERR) return -1;
    return static_cast<int>(SendMessageW(combo, CB_GETITEMDATA, item, 0));
}

void saveSettings() {
    const int in = selectedDevice(g_app.inputCombo);
    const int out = selectedDevice(g_app.outputCombo);
    wchar_t value[32]{};
    if (in >= 0) {
        swprintf_s(value, L"%d", in);
        WritePrivateProfileStringW(L"Audio", L"InputDevice", value, L".\\source-voice.ini");
    }
    if (out >= 0) {
        swprintf_s(value, L"%d", out);
        WritePrivateProfileStringW(L"Audio", L"OutputDevice", value, L".\\source-voice.ini");
    }
    swprintf_s(value, L"%d", g_app.quality);
    WritePrivateProfileStringW(L"Audio", L"Quality", value, L".\\source-voice.ini");
}

void setStatus(const std::wstring& text) {
    SetWindowTextW(g_app.statusLabel, text.c_str());
}

void stopAudio() {
    if (g_app.stream) {
        Pa_StopStream(g_app.stream);
        Pa_CloseStream(g_app.stream);
        g_app.stream = nullptr;
    }
    g_app.engine.reset();
    g_app.running = false;
    if (g_app.startButton) SetWindowTextW(g_app.startButton, L"Start processing");
    if (g_app.inputCombo) EnableWindow(g_app.inputCombo, TRUE);
    if (g_app.outputCombo) EnableWindow(g_app.outputCombo, TRUE);
    if (g_app.qualitySlider) EnableWindow(g_app.qualitySlider, TRUE);
    if (g_app.levelBar) SendMessageW(g_app.levelBar, PBM_SETPOS, 0, 0);
    if (g_app.latencyLabel) SetWindowTextW(g_app.latencyLabel, L"Input: —    Output: —");
    g_inputPeak.store(0.0f, std::memory_order_relaxed);
}

bool startAudio() {
    const int inputDevice = selectedDevice(g_app.inputCombo);
    const int outputDevice = selectedDevice(g_app.outputCombo);
    if (inputDevice < 0 || outputDevice < 0) {
        MessageBoxW(g_app.window, L"Select both an input and an output device.",
                    L"Source Voice", MB_OK | MB_ICONWARNING);
        return false;
    }

    const PaDeviceInfo* inInfo = Pa_GetDeviceInfo(inputDevice);
    const PaDeviceInfo* outInfo = Pa_GetDeviceInfo(outputDevice);
    if (!inInfo || !outInfo || inInfo->maxInputChannels < 1 || outInfo->maxOutputChannels < 1) {
        MessageBoxW(g_app.window, L"The selected audio device is unavailable.",
                    L"Source Voice", MB_OK | MB_ICONERROR);
        return false;
    }

    g_app.engine = std::make_unique<Engine>(g_app.quality);
    if (!g_app.engine->active) {
        g_app.engine.reset();
        MessageBoxW(g_app.window, L"Could not initialize the Speex codec.",
                    L"Source Voice", MB_OK | MB_ICONERROR);
        return false;
    }

    PaStreamParameters input{};
    input.device = inputDevice;
    input.channelCount = 1;
    input.sampleFormat = paFloat32;
    input.suggestedLatency = kSuggestedLatencySeconds;
    input.hostApiSpecificStreamInfo = nullptr;

    PaStreamParameters output{};
    output.device = outputDevice;
    output.channelCount = 1;
    output.sampleFormat = paFloat32;
    output.suggestedLatency = kSuggestedLatencySeconds;
    output.hostApiSpecificStreamInfo = nullptr;

    PaError err = Pa_OpenStream(&g_app.stream, &input, &output, kDeviceRate,
                                kCallbackFrames, paNoFlag, paCallback, g_app.engine.get());
    if (err != paNoError) {
        const std::wstring message = L"Could not open audio stream:\n" + widen(Pa_GetErrorText(err)) +
            L"\n\nCheck that the selected devices support 48 kHz mono float audio.";
        MessageBoxW(g_app.window, message.c_str(), L"Source Voice", MB_OK | MB_ICONERROR);
        g_app.stream = nullptr;
        g_app.engine.reset();
        return false;
    }

    err = Pa_StartStream(g_app.stream);
    if (err != paNoError) {
        const std::wstring message = L"Could not start audio stream:\n" + widen(Pa_GetErrorText(err));
        MessageBoxW(g_app.window, message.c_str(), L"Source Voice", MB_OK | MB_ICONERROR);
        Pa_CloseStream(g_app.stream);
        g_app.stream = nullptr;
        g_app.engine.reset();
        return false;
    }

    const PaStreamInfo* info = Pa_GetStreamInfo(g_app.stream);
    wchar_t latency[160]{};
    if (info) {
        swprintf_s(latency, L"Input: %.0f ms    Output: %.0f ms",
                   info->inputLatency * 1000.0, info->outputLatency * 1000.0);
    } else {
        wcscpy_s(latency, L"Latency information unavailable");
    }
    SetWindowTextW(g_app.latencyLabel, latency);
    g_app.running = true;
    SetWindowTextW(g_app.startButton, L"Stop processing");
    EnableWindow(g_app.inputCombo, FALSE);
    EnableWindow(g_app.outputCombo, FALSE);
    EnableWindow(g_app.qualitySlider, FALSE);
    setStatus(L"Running · Speex Narrowband");
    saveSettings();
    return true;
}

void updateQualityText() {
    wchar_t text[32]{};
    swprintf_s(text, L"%d / 10", g_app.quality);
    SetWindowTextW(g_app.qualityValue, text);
}

LRESULT CALLBACK windowProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam) {
    switch (message) {
    case WM_CREATE: {
        g_app.window = hwnd;
        g_font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));
        g_titleFont = CreateFontW(22, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE,
                                 DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                                 CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");

        addLabel(hwnd, L"Source Voice", 28, 20, 300, 34, true);
        addLabel(hwnd, L"Source 1 / CS voice processing", 30, 52, 400, 20);

        addLabel(hwnd, L"MICROPHONE INPUT", 30, 100, 220, 20);
        g_app.inputCombo = makeControl(L"COMBOBOX", L"", CBS_DROPDOWNLIST | WS_VSCROLL,
                                       30, 122, 555, 240, hwnd, ID_INPUT);
        setFont(g_app.inputCombo);

        addLabel(hwnd, L"VIRTUAL OUTPUT", 30, 178, 220, 20);
        g_app.outputCombo = makeControl(L"COMBOBOX", L"", CBS_DROPDOWNLIST | WS_VSCROLL,
                                        30, 200, 555, 240, hwnd, ID_OUTPUT);
        setFont(g_app.outputCombo);

        addLabel(hwnd, L"SPEEX QUALITY", 30, 258, 200, 20);
        g_app.qualityValue = makeControl(L"STATIC", L"3 / 10", SS_RIGHT,
                                         510, 258, 75, 20, hwnd);
        setFont(g_app.qualityValue);
        g_app.qualitySlider = makeControl(TRACKBAR_CLASSW, L"",
                                          TBS_AUTOTICKS | TBS_HORZ,
                                          30, 280, 555, 38, hwnd, ID_QUALITY);
        setFont(g_app.qualitySlider);
        SendMessageW(g_app.qualitySlider, TBM_SETRANGE, TRUE, MAKELPARAM(0, 10));
        SendMessageW(g_app.qualitySlider, TBM_SETPOS, TRUE, g_app.quality);

        addLabel(hwnd, L"INPUT LEVEL", 30, 334, 200, 20);
        g_app.levelBar = makeControl(PROGRESS_CLASSW, L"", PBS_SMOOTH,
                                     30, 356, 555, 18, hwnd, ID_LEVEL);
        SendMessageW(g_app.levelBar, PBM_SETRANGE, 0, MAKELPARAM(0, 100));
        SendMessageW(g_app.levelBar, PBM_SETBARCOLOR, 0, RGB(45, 145, 95));

        g_app.latencyLabel = makeControl(L"STATIC", L"Input: —    Output: —", SS_LEFT,
                                         30, 392, 555, 22, hwnd, ID_LATENCY);
        setFont(g_app.latencyLabel);
        g_app.statusLabel = makeControl(L"STATIC", L"Stopped · Select devices and start",
                                        SS_LEFT, 30, 420, 555, 22, hwnd, ID_STATUS);
        setFont(g_app.statusLabel);

        g_app.startButton = makeControl(L"BUTTON", L"Start processing",
                                        BS_DEFPUSHBUTTON, 30, 462, 555, 42, hwnd, ID_START);
        setFont(g_app.startButton);

        g_app.quality = std::clamp<int>(static_cast<int>(GetPrivateProfileIntW(
            L"Audio", L"Quality", 3, L".\\source-voice.ini")), 0, 10);
        SendMessageW(g_app.qualitySlider, TBM_SETPOS, TRUE, g_app.quality);
        updateQualityText();
        populateDevices();
        SetTimer(hwnd, TIMER_ID, 50, nullptr);
        break;
    }
    case WM_HSCROLL:
        if (reinterpret_cast<HWND>(lParam) == g_app.qualitySlider && !g_app.running) {
            g_app.quality = static_cast<int>(SendMessageW(g_app.qualitySlider, TBM_GETPOS, 0, 0));
            updateQualityText();
            saveSettings();
        }
        return 0;
    case WM_COMMAND:
        if (LOWORD(wParam) == ID_START && HIWORD(wParam) == BN_CLICKED) {
            if (g_app.running) {
                stopAudio();
                setStatus(L"Stopped");
            } else {
                startAudio();
            }
            return 0;
        }
        if ((LOWORD(wParam) == ID_INPUT || LOWORD(wParam) == ID_OUTPUT) &&
            HIWORD(wParam) == CBN_SELCHANGE && !g_app.running) {
            saveSettings();
            return 0;
        }
        break;
    case WM_TIMER:
        if (g_app.running) {
            const float peak = g_inputPeak.exchange(0.0f, std::memory_order_relaxed);
            const int level = std::clamp(static_cast<int>(peak * 100.0f), 0, 100);
            SendMessageW(g_app.levelBar, PBM_SETPOS, level, 0);
        }
        return 0;
    case WM_CLOSE:
        stopAudio();
        saveSettings();
        DestroyWindow(hwnd);
        return 0;
    case WM_DESTROY:
        KillTimer(hwnd, TIMER_ID);
        if (g_titleFont) DeleteObject(g_titleFont);
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hwnd, message, wParam, lParam);
}
}

int main(int argc, char** argv) {
    bool list = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--list") list = true;
        if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
            std::cout << "Source Voice\n  source-voice.exe          Open graphical interface\n"
                         "  source-voice.exe --list   List PortAudio devices\n";
            return 0;
        }
    }

    PaError err = Pa_Initialize();
    if (err != paNoError) {
        MessageBoxW(nullptr, widen(Pa_GetErrorText(err)).c_str(),
                    L"Source Voice · PortAudio error", MB_OK | MB_ICONERROR);
        return 1;
    }

    if (list) {
        listDevices();
        Pa_Terminate();
        return 0;
    }

    // Detach from a console when started from Explorer; terminal sessions remain usable.
    FreeConsole();

    INITCOMMONCONTROLSEX controls{ sizeof(controls), ICC_BAR_CLASSES };
    InitCommonControlsEx(&controls);

    WNDCLASSEXW wc{};
    wc.cbSize = sizeof(wc);
    wc.lpfnWndProc = windowProc;
    wc.hInstance = GetModuleHandleW(nullptr);
    wc.hCursor = LoadCursorW(nullptr, MAKEINTRESOURCEW(32512));
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    wc.lpszClassName = L"SourceVoiceMainWindow";
    if (!RegisterClassExW(&wc)) {
        Pa_Terminate();
        MessageBoxW(nullptr, L"Could not register the application window.",
                    L"Source Voice", MB_OK | MB_ICONERROR);
        return 1;
    }

    HWND hwnd = CreateWindowExW(0, wc.lpszClassName, L"Source Voice",
                                WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
                                CW_USEDEFAULT, CW_USEDEFAULT, 640, 560,
                                nullptr, nullptr, wc.hInstance, nullptr);
    if (!hwnd) {
        Pa_Terminate();
        MessageBoxW(nullptr, L"Could not create the application window.",
                    L"Source Voice", MB_OK | MB_ICONERROR);
        return 1;
    }

    ShowWindow(hwnd, SW_SHOW);
    UpdateWindow(hwnd);

    MSG msg{};
    while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }

    Pa_Terminate();
    return static_cast<int>(msg.wParam);
}

#define NOMINMAX
#include <windows.h>
#include <windowsx.h>
#include <shellapi.h>
#include "resource.h"
#include <commctrl.h>
#include <uxtheme.h>
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
#pragma comment(lib, "UxTheme.lib")
#pragma comment(lib, "Shell32.lib")

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
constexpr UINT WM_TRAYICON = WM_APP + 41;
constexpr UINT ID_TRAY_SHOW = 41001;
constexpr UINT ID_TRAY_TOGGLE = 41002;
constexpr UINT ID_TRAY_EXIT = 41003;

NOTIFYICONDATAW g_trayData{};
HICON g_appIconSmall = nullptr;
HICON g_appIconLarge = nullptr;
bool g_trayIconAdded = false;

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
    bool draggingQuality = false;
    int quality = 3;
    int displayLevel = 0;
} g_app;

HFONT g_font = nullptr;
HFONT g_titleFont = nullptr;
HFONT g_sectionFont = nullptr;
HBRUSH g_windowBrush = nullptr;
HBRUSH g_controlBrush = nullptr;
HBRUSH g_comboBrush = nullptr;

// Palette sampled from the reference Steam skin.
constexpr COLORREF kBackground = RGB(76, 88, 68);   // window, buttons, menus
constexpr COLORREF kPanel = kBackground;
constexpr COLORREF kInset = RGB(62, 70, 55);        // fields, lists, tracks
constexpr COLORREF kHover = RGB(90, 106, 80);       // hover / selected
constexpr COLORREF kText = RGB(220, 222, 223);
constexpr COLORREF kMuted = RGB(155, 162, 153);
constexpr COLORREF kGold = RGB(196, 181, 80);
constexpr COLORREF kEdgeLight = RGB(128, 128, 128);
constexpr COLORREF kEdgeDark = RGB(44, 44, 44);

void fillSolid(HDC dc, const RECT& r, COLORREF c) {
    HBRUSH b = CreateSolidBrush(c);
    FillRect(dc, &r, b);
    DeleteObject(b);
}

// 1px flat bevel: light top/left + dark bottom/right (raised), inverted when sunken.
void drawBevel(HDC dc, const RECT& r, bool raised) {
    const COLORREF tl = raised ? kEdgeLight : kEdgeDark;
    const COLORREF br = raised ? kEdgeDark : kEdgeLight;
    fillSolid(dc, RECT{ r.left, r.top, r.right - 1, r.top + 1 }, tl);
    fillSolid(dc, RECT{ r.left, r.top, r.left + 1, r.bottom - 1 }, tl);
    fillSolid(dc, RECT{ r.left, r.bottom - 1, r.right, r.bottom }, br);
    fillSolid(dc, RECT{ r.right - 1, r.top, r.right, r.bottom }, br);
}

void drawQualitySlider(HDC dc) {
    RECT track{ 48, 293, 567, 300 };
    fillSolid(dc, track, kInset);

    for (int i = 0; i <= 10; ++i) {
        const int x = 50 + i * 51;
        fillSolid(dc, RECT{ x - 1, 305, x + 1, 312 }, kMuted);
    }

    const int knobX = 50 + g_app.quality * 51;
    RECT knob{ knobX - 9, 283, knobX + 10, 311 };
    fillSolid(dc, knob, kHover);
    drawBevel(dc, knob, true);
    fillSolid(dc, RECT{ knobX - 3, 288, knobX - 2, 306 }, kInset);
    fillSolid(dc, RECT{ knobX + 3, 288, knobX + 4, 306 }, kInset);
}

void drawLevelMeter(HDC dc) {
    RECT outer{ 30, 356, 585, 376 };
    fillSolid(dc, outer, kInset);

    const int segments = 54;
    const int gap = 2;
    const int segmentWidth = 8;
    const int lit = (g_app.displayLevel * segments) / 100;
    for (int i = 0; i < segments; ++i) {
        const int left = 39 + i * (segmentWidth + gap);
        RECT seg{ left, 363, left + segmentWidth, 369 };
        const COLORREF c = i < 39 ? RGB(68, 136, 44) : (i < 48 ? kGold : RGB(255, 91, 63));
        fillSolid(dc, seg, i < lit ? c : kBackground);
    }
}

void updateQualityText();
void saveSettings();

void updateQualityFromX(int x) {
    const int clamped = std::clamp(x, 50, 560);
    g_app.quality = std::clamp(static_cast<int>(std::lround((clamped - 50) / 51.0)), 0, 10);
    updateQualityText();
    saveSettings();
    InvalidateRect(g_app.window, nullptr, FALSE);
}

void drawSection(HDC dc, const wchar_t* title, int y, int width = 555) {
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, kGold);
    HFONT old = static_cast<HFONT>(SelectObject(dc, g_sectionFont ? g_sectionFont : g_font));
    RECT tr{ 30, y, 30 + width, y + 23 };
    DrawTextW(dc, title, -1, &tr, DT_LEFT | DT_VCENTER | DT_SINGLELINE);
    SelectObject(dc, old);
}

// Items of the dropdown list.
void drawComboItem(const DRAWITEMSTRUCT* dis) {
    if (!dis || dis->CtlType != ODT_COMBOBOX) return;
    if (dis->itemID == static_cast<UINT>(-1)) return; // closed field is painted by comboProc
    RECT r = dis->rcItem;
    const bool selected = (dis->itemState & ODS_SELECTED) != 0;
    fillSolid(dis->hDC, r, selected ? kHover : kBackground);
    wchar_t text[512]{};
    SendMessageW(dis->hwndItem, CB_GETLBTEXT, dis->itemID, reinterpret_cast<LPARAM>(text));
    SetBkMode(dis->hDC, TRANSPARENT);
    SetTextColor(dis->hDC, kText);
    HFONT old = static_cast<HFONT>(SelectObject(dis->hDC, g_font));
    r.left += 7;
    DrawTextW(dis->hDC, text, -1, &r, DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS | DT_NOPREFIX);
    SelectObject(dis->hDC, old);
}

// Paints the closed combo box (inset field + flat arrow button) in the skin colors.
LRESULT CALLBACK comboProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam,
                           UINT_PTR, DWORD_PTR) {
    switch (msg) {
    case WM_ERASEBKGND:
        return 1;
    case WM_PAINT: {
        PAINTSTRUCT ps{};
        HDC dc = BeginPaint(hwnd, &ps);
        RECT rc{};
        GetClientRect(hwnd, &rc);
        const bool enabled = IsWindowEnabled(hwnd) != FALSE;
        const bool dropped = SendMessageW(hwnd, CB_GETDROPPEDSTATE, 0, 0) != 0;

        fillSolid(dc, rc, kInset);

        RECT btn{ rc.right - 22, rc.top, rc.right, rc.bottom };
        fillSolid(dc, btn, dropped ? kHover : kBackground);
        drawBevel(dc, btn, !dropped);

        const COLORREF arrowColor = enabled ? kText : kMuted;
        const int cx = (btn.left + btn.right) / 2;
        const int cy = (btn.top + btn.bottom) / 2;
        POINT tri[3] = { { cx - 4, cy - 2 }, { cx + 4, cy - 2 }, { cx, cy + 2 } };
        HBRUSH arrowBrush = CreateSolidBrush(arrowColor);
        HPEN arrowPen = CreatePen(PS_SOLID, 1, arrowColor);
        HGDIOBJ oldBrush = SelectObject(dc, arrowBrush);
        HGDIOBJ oldPen = SelectObject(dc, arrowPen);
        Polygon(dc, tri, 3);
        SelectObject(dc, oldBrush);
        SelectObject(dc, oldPen);
        DeleteObject(arrowBrush);
        DeleteObject(arrowPen);

        wchar_t text[512]{};
        const LRESULT sel = SendMessageW(hwnd, CB_GETCURSEL, 0, 0);
        if (sel != CB_ERR)
            SendMessageW(hwnd, CB_GETLBTEXT, static_cast<WPARAM>(sel), reinterpret_cast<LPARAM>(text));
        SetBkMode(dc, TRANSPARENT);
        SetTextColor(dc, enabled ? kText : kMuted);
        HFONT old = static_cast<HFONT>(SelectObject(dc, g_font));
        RECT tr{ rc.left + 7, rc.top, btn.left - 4, rc.bottom };
        DrawTextW(dc, text, -1, &tr, DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS | DT_NOPREFIX);
        SelectObject(dc, old);
        EndPaint(hwnd, &ps);
        return 0;
    }
    case WM_NCDESTROY:
        RemoveWindowSubclass(hwnd, comboProc, 0);
        break;
    }
    return DefSubclassProc(hwnd, msg, wParam, lParam);
}

// Hover tracking for the owner-drawn Start button.
bool g_startHot = false;

LRESULT CALLBACK buttonHoverProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam,
                                 UINT_PTR, DWORD_PTR) {
    switch (msg) {
    case WM_MOUSEMOVE:
        if (!g_startHot) {
            g_startHot = true;
            TRACKMOUSEEVENT t{ sizeof(t), TME_LEAVE, hwnd, 0 };
            TrackMouseEvent(&t);
            InvalidateRect(hwnd, nullptr, FALSE);
        }
        break;
    case WM_MOUSELEAVE:
        g_startHot = false;
        InvalidateRect(hwnd, nullptr, FALSE);
        break;
    case WM_NCDESTROY:
        RemoveWindowSubclass(hwnd, buttonHoverProc, 0);
        break;
    }
    return DefSubclassProc(hwnd, msg, wParam, lParam);
}

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
    setFont(label, title ? g_titleFont : g_sectionFont);
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
    int savedInput = GetPrivateProfileIntW(L"Audio", L"InputDevice", -1, L".\\mic-sourceify.ini");
    int savedOutput = GetPrivateProfileIntW(L"Audio", L"OutputDevice", -1, L".\\mic-sourceify.ini");
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
        WritePrivateProfileStringW(L"Audio", L"InputDevice", value, L".\\mic-sourceify.ini");
    }
    if (out >= 0) {
        swprintf_s(value, L"%d", out);
        WritePrivateProfileStringW(L"Audio", L"OutputDevice", value, L".\\mic-sourceify.ini");
    }
    swprintf_s(value, L"%d", g_app.quality);
    WritePrivateProfileStringW(L"Audio", L"Quality", value, L".\\mic-sourceify.ini");
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
    g_app.displayLevel = 0;
    if (g_app.window) InvalidateRect(g_app.window, nullptr, FALSE);
    if (g_app.latencyLabel) SetWindowTextW(g_app.latencyLabel, L"Input: —    Output: —");
    g_inputPeak.store(0.0f, std::memory_order_relaxed);
}

bool startAudio() {
    const int inputDevice = selectedDevice(g_app.inputCombo);
    const int outputDevice = selectedDevice(g_app.outputCombo);
    if (inputDevice < 0 || outputDevice < 0) {
        MessageBoxW(g_app.window, L"Select both an input and an output device.",
                    L"Mic-Sourceify", MB_OK | MB_ICONWARNING);
        return false;
    }

    const PaDeviceInfo* inInfo = Pa_GetDeviceInfo(inputDevice);
    const PaDeviceInfo* outInfo = Pa_GetDeviceInfo(outputDevice);
    if (!inInfo || !outInfo || inInfo->maxInputChannels < 1 || outInfo->maxOutputChannels < 1) {
        MessageBoxW(g_app.window, L"The selected audio device is unavailable.",
                    L"Mic-Sourceify", MB_OK | MB_ICONERROR);
        return false;
    }

    g_app.engine = std::make_unique<Engine>(g_app.quality);
    if (!g_app.engine->active) {
        g_app.engine.reset();
        MessageBoxW(g_app.window, L"Could not initialize the Speex codec.",
                    L"Mic-Sourceify", MB_OK | MB_ICONERROR);
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
        MessageBoxW(g_app.window, message.c_str(), L"Mic-Sourceify", MB_OK | MB_ICONERROR);
        g_app.stream = nullptr;
        g_app.engine.reset();
        return false;
    }

    err = Pa_StartStream(g_app.stream);
    if (err != paNoError) {
        const std::wstring message = L"Could not start audio stream:\n" + widen(Pa_GetErrorText(err));
        MessageBoxW(g_app.window, message.c_str(), L"Mic-Sourceify", MB_OK | MB_ICONERROR);
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
    setStatus(L"Running · Speex Narrowband");
    saveSettings();
    return true;
}

void updateQualityText() {
    wchar_t text[32]{};
    swprintf_s(text, L"%d / 10", g_app.quality);
    SetWindowTextW(g_app.qualityValue, text);
}


void addTrayIcon(HWND hwnd) {
    if (g_trayIconAdded) return;
    g_trayData = {};
    g_trayData.cbSize = sizeof(g_trayData);
    g_trayData.hWnd = hwnd;
    g_trayData.uID = 1;
    g_trayData.uFlags = NIF_MESSAGE | NIF_ICON | NIF_TIP;
    g_trayData.uCallbackMessage = WM_TRAYICON;
    g_trayData.hIcon = g_appIconSmall;
    wcscpy_s(g_trayData.szTip, L"Mic-Sourceify");
    g_trayIconAdded = Shell_NotifyIconW(NIM_ADD, &g_trayData) == TRUE;
}

void removeTrayIcon() {
    if (g_trayIconAdded) {
        Shell_NotifyIconW(NIM_DELETE, &g_trayData);
        g_trayIconAdded = false;
    }
}

void showMainWindow(HWND hwnd) {
    ShowWindow(hwnd, SW_SHOW);
    ShowWindow(hwnd, SW_RESTORE);
    SetForegroundWindow(hwnd);
}

void toggleAudio() {
    if (g_app.running) {
        stopAudio();
        setStatus(L"Stopped");
    } else {
        startAudio();
    }
}

void showTrayMenu(HWND hwnd) {
    HMENU menu = CreatePopupMenu();
    if (!menu) return;
    AppendMenuW(menu, MF_STRING, ID_TRAY_SHOW, L"Show Mic-Sourceify");
    AppendMenuW(menu, MF_STRING, ID_TRAY_TOGGLE,
                g_app.running ? L"Stop processing" : L"Start processing");
    AppendMenuW(menu, MF_SEPARATOR, 0, nullptr);
    AppendMenuW(menu, MF_STRING, ID_TRAY_EXIT, L"Exit");
    POINT pt{};
    GetCursorPos(&pt);
    SetForegroundWindow(hwnd);
    const UINT command = TrackPopupMenu(menu, TPM_RETURNCMD | TPM_RIGHTBUTTON,
                                        pt.x, pt.y, 0, hwnd, nullptr);
    DestroyMenu(menu);
    if (command == ID_TRAY_SHOW) {
        showMainWindow(hwnd);
    } else if (command == ID_TRAY_TOGGLE) {
        toggleAudio();
    } else if (command == ID_TRAY_EXIT) {
        removeTrayIcon();
        DestroyWindow(hwnd);
    }
    PostMessageW(hwnd, WM_NULL, 0, 0);
}

LRESULT CALLBACK windowProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam) {
    switch (message) {
    case WM_ERASEBKGND:
        return 1;
    case WM_PAINT: {
        PAINTSTRUCT ps{};
        HDC dc = BeginPaint(hwnd, &ps);
        RECT client{};
        GetClientRect(hwnd, &client);
        fillSolid(dc, client, kBackground);   // single flat surface, no nested panel

        SetBkMode(dc, TRANSPARENT);
        SetTextColor(dc, kText);
        HFONT old = static_cast<HFONT>(SelectObject(dc, g_titleFont ? g_titleFont : g_font));
        RECT title{ 30, 20, client.right - 30, 52 };
        DrawTextW(dc, L"MIC-SOURCEIFY", -1, &title, DT_LEFT | DT_VCENTER | DT_SINGLELINE);
        SelectObject(dc, old);
        SetTextColor(dc, kMuted);
        HFONT oldSmall = static_cast<HFONT>(SelectObject(dc, g_font));
        RECT subtitle{ 30, 53, client.right - 30, 78 };
        DrawTextW(dc, L"SOURCE 1  /  COUNTER-STRIKE VOICE PROCESSING", -1,
                  &subtitle, DT_LEFT | DT_VCENTER | DT_SINGLELINE);
        SelectObject(dc, oldSmall);

        drawSection(dc, L"MICROPHONE INPUT", 100);
        drawSection(dc, L"VIRTUAL OUTPUT", 178);
        drawSection(dc, L"SPEEX QUALITY", 258);
        drawSection(dc, L"INPUT LEVEL", 334);
        drawQualitySlider(dc);
        drawLevelMeter(dc);
        EndPaint(hwnd, &ps);
        return 0;
    }
    case WM_CTLCOLORSTATIC: {
        HDC dc = reinterpret_cast<HDC>(wParam);
        SetBkMode(dc, TRANSPARENT);
        SetTextColor(dc, kText);
        return reinterpret_cast<LRESULT>(g_windowBrush);
    }
    case WM_CTLCOLOREDIT:
    case WM_CTLCOLORLISTBOX: {
        HDC dc = reinterpret_cast<HDC>(wParam);
        SetTextColor(dc, kText);
        SetBkColor(dc, kBackground);
        return reinterpret_cast<LRESULT>(g_comboBrush);
    }
    case WM_DRAWITEM: {
        const auto* dis = reinterpret_cast<const DRAWITEMSTRUCT*>(lParam);
        if (dis && dis->CtlID == ID_START) {
            RECT r = dis->rcItem;
            const bool pressed = (dis->itemState & ODS_SELECTED) != 0;
            fillSolid(dis->hDC, r, pressed ? kInset : (g_startHot ? kHover : kBackground));
            drawBevel(dis->hDC, r, !pressed);
            SetBkMode(dis->hDC, TRANSPARENT);
            SetTextColor(dis->hDC, kText);
            HFONT old = static_cast<HFONT>(SelectObject(dis->hDC, g_sectionFont));
            wchar_t label[64]{};
            GetWindowTextW(dis->hwndItem, label, 63);
            if (pressed) OffsetRect(&r, 1, 1);
            DrawTextW(dis->hDC, label, -1, &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
            SelectObject(dis->hDC, old);
            return TRUE;
        }
        if (dis && dis->CtlType == ODT_COMBOBOX) {
            drawComboItem(dis);
            return TRUE;
        }
        break;
    }
    case WM_MEASUREITEM: {
        auto* mis = reinterpret_cast<MEASUREITEMSTRUCT*>(lParam);
        if (mis && mis->CtlType == ODT_COMBOBOX) {
            mis->itemHeight = 25;
            return TRUE;
        }
        break;
    }
    case WM_CREATE: {
        g_app.window = hwnd;
        SendMessageW(hwnd, WM_SETICON, ICON_SMALL, reinterpret_cast<LPARAM>(g_appIconSmall));
        SendMessageW(hwnd, WM_SETICON, ICON_BIG, reinterpret_cast<LPARAM>(g_appIconLarge));
        g_font = CreateFontW(16, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
                             DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                             CLEARTYPE_QUALITY, FIXED_PITCH | FF_MODERN, L"Tahoma");
        g_sectionFont = CreateFontW(15, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
                                    DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                                    CLEARTYPE_QUALITY, FIXED_PITCH | FF_MODERN, L"Tahoma");
        g_windowBrush = CreateSolidBrush(kBackground);
        g_controlBrush = CreateSolidBrush(kPanel);
        g_comboBrush = CreateSolidBrush(kBackground);

        g_titleFont = CreateFontW(22, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE,
                                 DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                                 CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, L"Tahoma");


        g_app.inputCombo = makeControl(L"COMBOBOX", L"", CBS_DROPDOWNLIST | CBS_OWNERDRAWFIXED | CBS_HASSTRINGS | WS_VSCROLL,
                                       30, 122, 555, 240, hwnd, ID_INPUT);
        setFont(g_app.inputCombo);
        SetWindowTheme(g_app.inputCombo, L"", L"");
        SetWindowSubclass(g_app.inputCombo, comboProc, 0, 0);

        g_app.outputCombo = makeControl(L"COMBOBOX", L"", CBS_DROPDOWNLIST | CBS_OWNERDRAWFIXED | CBS_HASSTRINGS | WS_VSCROLL,
                                        30, 200, 555, 240, hwnd, ID_OUTPUT);
        setFont(g_app.outputCombo);
        SetWindowTheme(g_app.outputCombo, L"", L"");
        SetWindowSubclass(g_app.outputCombo, comboProc, 0, 0);

        g_app.qualityValue = makeControl(L"STATIC", L"3 / 10", SS_RIGHT,
                                         510, 258, 75, 20, hwnd);
        setFont(g_app.qualityValue);

        g_app.latencyLabel = makeControl(L"STATIC", L"Input: n/a    Output: n/a", SS_LEFT,
                                         30, 392, 555, 22, hwnd, ID_LATENCY);
        setFont(g_app.latencyLabel);
        g_app.statusLabel = makeControl(L"STATIC", L"Stopped · Select devices and start",
                                        SS_LEFT, 30, 420, 555, 22, hwnd, ID_STATUS);
        setFont(g_app.statusLabel);

        g_app.startButton = makeControl(L"BUTTON", L"Start processing",
                                        BS_OWNERDRAW, 30, 462, 555, 42, hwnd, ID_START);
        setFont(g_app.startButton);
        SetWindowTheme(g_app.startButton, L"", L"");
        SetWindowSubclass(g_app.startButton, buttonHoverProc, 0, 0);

        g_app.quality = std::clamp<int>(static_cast<int>(GetPrivateProfileIntW(
            L"Audio", L"Quality", 3, L".\\mic-sourceify.ini")), 0, 10);
        updateQualityText();
        populateDevices();
        SetTimer(hwnd, TIMER_ID, 50, nullptr);
        break;
    }
    case WM_LBUTTONDOWN: {
        const int x = GET_X_LPARAM(lParam);
        const int y = GET_Y_LPARAM(lParam);
        if (!g_app.running && x >= 30 && x <= 585 && y >= 280 && y <= 318) {
            g_app.draggingQuality = true;
            SetCapture(hwnd);
            updateQualityFromX(x);
            return 0;
        }
        break;
    }
    case WM_MOUSEMOVE:
        if (g_app.draggingQuality && !g_app.running) {
            updateQualityFromX(GET_X_LPARAM(lParam));
            return 0;
        }
        break;
    case WM_LBUTTONUP:
        if (g_app.draggingQuality) {
            g_app.draggingQuality = false;
            ReleaseCapture();
            return 0;
        }
        break;
    case WM_COMMAND:
        if (LOWORD(wParam) == ID_START && HIWORD(wParam) == BN_CLICKED) {
            toggleAudio();
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
            g_app.displayLevel = std::clamp(static_cast<int>(peak * 100.0f), 0, 100);
            InvalidateRect(hwnd, nullptr, FALSE);
        }
        return 0;
    case WM_SIZE:
        if (wParam == SIZE_MINIMIZED) {
            addTrayIcon(hwnd);
            if (g_trayIconAdded) {
                ShowWindow(hwnd, SW_HIDE);
            } else {
                ShowWindow(hwnd, SW_RESTORE);
            }
            return 0;
        }
        break;
    case WM_TRAYICON:
        if (lParam == WM_RBUTTONUP || lParam == WM_CONTEXTMENU) {
            showTrayMenu(hwnd);
            return 0;
        }
        if (lParam == WM_LBUTTONUP || lParam == WM_LBUTTONDBLCLK) {
            showMainWindow(hwnd);
            return 0;
        }
        break;
    case WM_CLOSE:
        saveSettings();
        addTrayIcon(hwnd);
        if (g_trayIconAdded) {
            ShowWindow(hwnd, SW_HIDE);
        } else {
            MessageBoxW(hwnd, L"Could not create the notification-area icon. Mic-Sourceify will remain open.",
                        L"Mic-Sourceify", MB_OK | MB_ICONWARNING);
        }
        return 0;
    case WM_DESTROY:
        KillTimer(hwnd, TIMER_ID);
        removeTrayIcon();
        stopAudio();
        if (g_titleFont) DeleteObject(g_titleFont);
        if (g_appIconSmall) DestroyIcon(g_appIconSmall);
        if (g_appIconLarge) DestroyIcon(g_appIconLarge);
        if (g_sectionFont) DeleteObject(g_sectionFont);
        if (g_font) DeleteObject(g_font);
        if (g_windowBrush) DeleteObject(g_windowBrush);
        if (g_controlBrush) DeleteObject(g_controlBrush);
        if (g_comboBrush) DeleteObject(g_comboBrush);
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
            std::cout << "Mic-Sourceify\n  mic-sourceify.exe         Open graphical interface\n"
                         "  mic-sourceify.exe --list  List PortAudio devices\n";
            return 0;
        }
    }

    PaError err = Pa_Initialize();
    if (err != paNoError) {
        MessageBoxW(nullptr, widen(Pa_GetErrorText(err)).c_str(),
                    L"Mic-Sourceify · PortAudio error", MB_OK | MB_ICONERROR);
        return 1;
    }

    if (list) {
        listDevices();
        Pa_Terminate();
        return 0;
    }

    // Detach from a console when started from Explorer; terminal sessions remain usable.
    FreeConsole();

    // Keep saved settings from the old name (no-op if already migrated).
    MoveFileW(L".\\source-voice.ini", L".\\mic-sourceify.ini");

    INITCOMMONCONTROLSEX controls{ sizeof(controls), ICC_BAR_CLASSES };
    InitCommonControlsEx(&controls);

    HINSTANCE instance = GetModuleHandleW(nullptr);
    g_appIconSmall = static_cast<HICON>(LoadImageW(instance, MAKEINTRESOURCEW(IDI_APP_ICON),
        IMAGE_ICON, GetSystemMetrics(SM_CXSMICON), GetSystemMetrics(SM_CYSMICON), LR_DEFAULTCOLOR));
    g_appIconLarge = static_cast<HICON>(LoadImageW(instance, MAKEINTRESOURCEW(IDI_APP_ICON),
        IMAGE_ICON, GetSystemMetrics(SM_CXICON), GetSystemMetrics(SM_CYICON), LR_DEFAULTCOLOR));

    WNDCLASSEXW wc{};
    wc.cbSize = sizeof(wc);
    wc.lpfnWndProc = windowProc;
    wc.hInstance = instance;
    wc.hIcon = g_appIconLarge;
    wc.hIconSm = g_appIconSmall;
    wc.hCursor = LoadCursorW(nullptr, MAKEINTRESOURCEW(32512));
    wc.hbrBackground = g_windowBrush ? g_windowBrush : reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    wc.lpszClassName = L"MicSourceifyMainWindow";
    if (!RegisterClassExW(&wc)) {
        Pa_Terminate();
        MessageBoxW(nullptr, L"Could not register the application window.",
                    L"Mic-Sourceify", MB_OK | MB_ICONERROR);
        return 1;
    }

    const DWORD windowStyle = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_CLIPCHILDREN;
    RECT windowRect{ 0, 0, 615, 534 };   // client area: 30 px margins on every side
    AdjustWindowRectEx(&windowRect, windowStyle, FALSE, 0);
    HWND hwnd = CreateWindowExW(0, wc.lpszClassName, L"Mic-Sourceify",
                                windowStyle,
                                CW_USEDEFAULT, CW_USEDEFAULT,
                                windowRect.right - windowRect.left, windowRect.bottom - windowRect.top,
                                nullptr, nullptr, wc.hInstance, nullptr);
    if (!hwnd) {
        Pa_Terminate();
        MessageBoxW(nullptr, L"Could not create the application window.",
                    L"Mic-Sourceify", MB_OK | MB_ICONERROR);
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

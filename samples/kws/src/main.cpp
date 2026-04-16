/*
 * Keyword Spotting (KWS) Demo for Arduino Giga R1 WiFi
 *
 * Uses TFLite Micro with two models:
 *   1. Audio Preprocessor  – raw PCM → spectrogram features
 *   2. Micro Speech         – spectrogram → keyword class
 *
 * Categories: silence, unknown, yes, no
 */

#include <zephyr/kernel.h>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <iterator>
#include <cstddef>
#include <cctype>

/* TFLite Micro */
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_log.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/micro/system_setup.h>
#include <tensorflow/lite/schema/schema_generated.h>

#include "test_wavs_gen.hpp"

/* ------------------------------------------------------------------ */
/*  Embedded binary blobs (WAV + two .tflite models)                  */
/* ------------------------------------------------------------------ */

/* Audio preprocessor model (int8) */
static const uint8_t audio_preprocessor_model[] = {
#include "audio_preprocessor_int8.tflite.inc"
};

/* User-trained classifier model from ../model.cc (xxd generated) */
extern unsigned char g_model[];
extern unsigned int g_model_len;

/* ------------------------------------------------------------------ */
/*  Model constants (must match micro_model_settings.h)               */
/* ------------------------------------------------------------------ */
static constexpr int kAudioSampleFrequency = 16000;
static constexpr int kFeatureSize     = 40;   /* mel channels per frame   */
static constexpr int kFeatureCount    = 49;   /* number of frames         */
static constexpr int kFeatureElements = kFeatureSize * kFeatureCount; /* 1960 */
static constexpr int kFeatureStrideMs = 20;   /* hop between windows      */
static constexpr int kFeatureDurationMs = 30; /* window length            */
static constexpr int kAudioSampleDurationCount =
    kFeatureDurationMs * kAudioSampleFrequency / 1000;   /* 480 samples */
static constexpr int kAudioSampleStrideCount =
    kFeatureStrideMs * kAudioSampleFrequency / 1000;     /* 320 samples */

static constexpr int kKnownLabelCount = 6;
static const char* kKnownCategoryLabels[kKnownLabelCount] = {
    "silence", "unknown", "up", "down", "left", "right",
};

static const char* label_for_index(int idx)
{
    if (idx >= 0 && idx < kKnownLabelCount) {
        return kKnownCategoryLabels[idx];
    }
    return "(extra)";
}

static void to_lower_ascii(char* s)
{
    while (*s) {
        *s = static_cast<char>(std::tolower(static_cast<unsigned char>(*s)));
        s++;
    }
}

static const char* true_label_from_filename(const char* sample_name,
                                            char* out_label,
                                            size_t out_size)
{
    if (!sample_name || !out_label || out_size == 0) {
        return "unknown";
    }

    size_t i = 0;
    while (sample_name[i] && sample_name[i] != '_' && i + 1 < out_size) {
        out_label[i] = sample_name[i];
        i++;
    }
    out_label[i] = '\0';

    if (i == 0) {
        strncpy(out_label, "unknown", out_size - 1);
        out_label[out_size - 1] = '\0';
        return out_label;
    }

    to_lower_ascii(out_label);

    for (int idx = 0; idx < kKnownLabelCount; idx++) {
        if (strcmp(out_label, kKnownCategoryLabels[idx]) == 0) {
            return out_label;
        }
    }

    /* Non-category filenames are treated as unknown. */
    strncpy(out_label, "unknown", out_size - 1);
    out_label[out_size - 1] = '\0';
    return out_label;
}

/* Shared arena for both models (used sequentially, not simultaneously) */
static constexpr int kArenaSize = 30 * 1024; /* 30 KB */
alignas(16) static uint8_t g_arena[kArenaSize];

/* Feature buffer */
static int8_t g_features[kFeatureCount][kFeatureSize];

/* ------------------------------------------------------------------ */
/*  WAV parser – returns pointer to PCM data and its size             */
/* ------------------------------------------------------------------ */
struct WavInfo {
    const int16_t* pcm;
    uint32_t       pcm_samples;
    uint16_t       sample_rate;
    uint16_t       bits_per_sample;
    uint16_t       num_channels;
    bool           valid;
};

static WavInfo parse_wav(const uint8_t* data, size_t len)
{
    WavInfo info{};
    if (len < 44 || memcmp(data, "RIFF", 4) != 0 ||
        memcmp(data + 8, "WAVE", 4) != 0) {
        return info;
    }
    /* Walk chunks to find "fmt " and "data" */
    uint32_t offset = 12;
    const uint8_t* fmt_chunk = nullptr;
    const uint8_t* data_chunk = nullptr;
    uint32_t data_size = 0;

    while (offset + 8 <= len) {
        const uint8_t* chunk = data + offset;
        uint32_t chunk_size = chunk[4] | (chunk[5] << 8) |
                              (chunk[6] << 16) | (chunk[7] << 24);
        if (memcmp(chunk, "fmt ", 4) == 0) {
            fmt_chunk = chunk + 8;
        } else if (memcmp(chunk, "data", 4) == 0) {
            data_chunk = chunk + 8;
            data_size  = chunk_size;
        }
        offset += 8 + chunk_size;
        if (offset % 2) { offset++; } /* chunks are word-aligned */
    }
    if (!fmt_chunk || !data_chunk) { return info; }

    info.num_channels    = fmt_chunk[2] | (fmt_chunk[3] << 8);
    info.sample_rate     = fmt_chunk[4] | (fmt_chunk[5] << 8) |
                           (fmt_chunk[6] << 16) | (fmt_chunk[7] << 24);
    info.bits_per_sample = fmt_chunk[14] | (fmt_chunk[15] << 8);
    info.pcm             = reinterpret_cast<const int16_t*>(data_chunk);
    info.pcm_samples     = data_size / (info.bits_per_sample / 8);
    info.valid           = (info.num_channels == 1 &&
                            info.bits_per_sample == 16 &&
                            info.sample_rate == kAudioSampleFrequency);
    return info;
}

/* ------------------------------------------------------------------ */
/*  Audio Preprocessor  – generates spectrogram features              */
/* ------------------------------------------------------------------ */
/* Op resolver for the audio preprocessor model (18 ops) */
using AudioPPOpResolver = tflite::MicroMutableOpResolver<18>;

static bool register_audio_pp_ops(AudioPPOpResolver& r)
{
    if (r.AddReshape()       != kTfLiteOk) return false;
    if (r.AddCast()          != kTfLiteOk) return false;
    if (r.AddStridedSlice()  != kTfLiteOk) return false;
    if (r.AddConcatenation() != kTfLiteOk) return false;
    if (r.AddMul()           != kTfLiteOk) return false;
    if (r.AddAdd()           != kTfLiteOk) return false;
    if (r.AddDiv()           != kTfLiteOk) return false;
    if (r.AddMinimum()       != kTfLiteOk) return false;
    if (r.AddMaximum()       != kTfLiteOk) return false;
    if (r.AddWindow()        != kTfLiteOk) return false;
    if (r.AddFftAutoScale()  != kTfLiteOk) return false;
    if (r.AddRfft()          != kTfLiteOk) return false;
    if (r.AddEnergy()        != kTfLiteOk) return false;
    if (r.AddFilterBank()    != kTfLiteOk) return false;
    if (r.AddFilterBankSquareRoot()            != kTfLiteOk) return false;
    if (r.AddFilterBankSpectralSubtraction()   != kTfLiteOk) return false;
    if (r.AddPCAN()          != kTfLiteOk) return false;
    if (r.AddFilterBankLog() != kTfLiteOk) return false;
    return true;
}

static bool generate_features(const int16_t* audio, size_t audio_samples)
{
    const tflite::Model* model = tflite::GetModel(audio_preprocessor_model);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        printk("AudioPP: model version mismatch\n");
        return false;
    }

    AudioPPOpResolver op_resolver;
    if (!register_audio_pp_ops(op_resolver)) {
        printk("AudioPP: failed to register ops\n");
        return false;
    }

    tflite::MicroInterpreter interpreter(model, op_resolver,
                                         g_arena, kArenaSize);
    if (interpreter.AllocateTensors() != kTfLiteOk) {
        printk("AudioPP: AllocateTensors failed\n");
        return false;
    }
    /* Slide a 30 ms window with 20 ms stride across the 1 s audio */
    size_t remaining = audio_samples;
    int frame = 0;
    while (remaining >= (size_t)kAudioSampleDurationCount &&
           frame < kFeatureCount) {
        TfLiteTensor* input  = interpreter.input(0);
        TfLiteTensor* output = interpreter.output(0);
        if (!input || !output) { return false; }

        /* Copy 480 samples into the model input */
        std::copy_n(audio, kAudioSampleDurationCount,
                    tflite::GetTensorData<int16_t>(input));

        if (interpreter.Invoke() != kTfLiteOk) {
            printk("AudioPP: Invoke failed at frame %d\n", frame);
            return false;
        }

        /* Copy 40-channel feature output */
        std::copy_n(tflite::GetTensorData<int8_t>(output), kFeatureSize,
                    g_features[frame]);

        frame++;
        audio     += kAudioSampleStrideCount;
        remaining -= kAudioSampleStrideCount;
    }
    /* Pad remaining frames with zeros when clip is shorter than 1 second. */
    const int generated_frames = frame;
    while (frame < kFeatureCount) {
        memset(g_features[frame], 0, sizeof(g_features[frame]));
        frame++;
    }

    if (generated_frames < kFeatureCount) {
        printk("Feature frames: %d/%d (zero-padded)\n",
               generated_frames, kFeatureCount);
    }
    return (generated_frames > 0);
}

/* ------------------------------------------------------------------ */
/*  Micro Speech Classifier                                           */
/* ------------------------------------------------------------------ */
using MicroSpeechOpResolver = tflite::MicroMutableOpResolver<5>;

static bool register_speech_ops(MicroSpeechOpResolver& r)
{
    if (r.AddReshape()          != kTfLiteOk) return false;
    if (r.AddConv2D()           != kTfLiteOk) return false;
    if (r.AddFullyConnected()   != kTfLiteOk) return false;
    if (r.AddDepthwiseConv2D()  != kTfLiteOk) return false;
    if (r.AddSoftmax()          != kTfLiteOk) return false;
    return true;
}

static bool classify_keyword(int* out_best_idx, float* out_best_val)
{
    const tflite::Model* model = tflite::GetModel(g_model);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        printk("Speech: model version mismatch\n");
        return false;
    }
    MicroSpeechOpResolver op_resolver;
    if (!register_speech_ops(op_resolver)) {
        printk("Speech: failed to register ops\n");
        return false;
    }

    tflite::MicroInterpreter interpreter(model, op_resolver,
                                         g_arena, kArenaSize);
    if (interpreter.AllocateTensors() != kTfLiteOk) {
        printk("Speech: AllocateTensors failed\n");
        return false;
    }
    TfLiteTensor* input  = interpreter.input(0);
    TfLiteTensor* output = interpreter.output(0);
    if (!input || !output) { return false; }

    /* Copy features into the model input (1960 int8 values) */
    std::copy_n(&g_features[0][0], kFeatureElements,
                tflite::GetTensorData<int8_t>(input));

    if (interpreter.Invoke() != kTfLiteOk) {
        printk("Speech: Invoke failed\n");
        return false;
    }

    /* Dequantize and print results */
    const int output_count = output->dims->data[output->dims->size - 1];
    float scale      = output->params.scale;
    int   zero_point = output->params.zero_point;
    if (output_count <= 0 || output_count > 32) {
        printk("Speech: unexpected output size: %d\n", output_count);
        return false;
    }

    float predictions[32];
    int   best_idx = 0;
    float best_val = -1.0f;
    printk("\n");
    for (int i = 0; i < output_count; i++) {
        predictions[i] = (tflite::GetTensorData<int8_t>(output)[i]
                          - zero_point) * scale;
        printk("  %-8s %.4f\n",
               label_for_index(i), static_cast<double>(predictions[i]));
        if (predictions[i] > best_val) {
            best_val = predictions[i];
            best_idx = i;
        }
    }

    if (out_best_idx) {
        *out_best_idx = best_idx;
    }
    if (out_best_val) {
        *out_best_val = best_val;
    }

    return true;
}

static bool run_one_sample(const char* sample_name,
                           const uint8_t* wav_data,
                           size_t wav_size)
{
    printk("\n--- %s ---\n", sample_name);

    WavInfo wav = parse_wav(wav_data, wav_size);
    if (!wav.valid) {
        printk("ERROR: invalid WAV (need 16 kHz, 16-bit, mono): %s\n",
               sample_name);
        return false;
    }

    if (!generate_features(wav.pcm, wav.pcm_samples)) {
        printk("ERROR: feature generation failed: %s\n", sample_name);
        return false;
    }

    int best_idx = -1;
    float best_val = 0.0f;
    if (!classify_keyword(&best_idx, &best_val)) {
        printk("ERROR: classification failed: %s\n", sample_name);
        return false;
    }

    const char* predicted_label = label_for_index(best_idx);
    char true_label_buf[32];
    const char* true_label = true_label_from_filename(sample_name,
                                                      true_label_buf,
                                                      sizeof(true_label_buf));
    const bool pass = (strcmp(predicted_label, true_label) == 0);

    printk("  predicted: %s\n", predicted_label);
    printk("  true:      %s\n", true_label);
    printk("  %s\n", pass ? "PASS" : "FAIL");

    return true;
}

/* ------------------------------------------------------------------ */
/*  main                                                              */
/* ------------------------------------------------------------------ */
int main(void)
{
    k_msleep(3000);

    printk("\n*** Keyword Spotting (KWS) Demo ***\n\n");

    tflite::InitializeTarget();

    if (g_test_wavs_count == 0) {
        printk("ERROR: no files found in src/test_wavs\n");
        printk("Add WAV files like up_1.wav, down_1.wav, left_1.wav, right_1.wav, unknown_1.wav, silence_1.wav\n");
        return 1;
    }

    printk("Found %u test wav(s) in src/test_wavs\n",
           (unsigned int)g_test_wavs_count);
    for (size_t i = 0; i < g_test_wavs_count; i++) {
        if (!run_one_sample(g_test_wavs[i].name,
                            g_test_wavs[i].data,
                            g_test_wavs[i].size)) {
            return 1;
        }
    }

    printk("\nKWS demo complete.\n");
    return 0;
}

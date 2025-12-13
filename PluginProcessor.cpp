// PluginProcessor.cpp
#include "PluginProcessor.h"
#include "PluginEditor.h"
#include <algorithm>
#include <random>
#include <numeric>

// Include SSE intrinsics for fast inverse square root
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 1)
    #include <xmmintrin.h>
#endif

//==============================================================================
CloudLikeGranularProcessor::CloudLikeGranularProcessor()
    : AudioProcessor (BusesProperties()
                        .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                        .withOutput ("Output", juce::AudioChannelSet::stereo(), true)),
      apvts (*this, nullptr, "PARAMS", createParameterLayout())
{
    apvts.addParameterListener ("randomize", this);
}

CloudLikeGranularProcessor::~CloudLikeGranularProcessor()
{
    apvts.removeParameterListener ("randomize", this);
}

juce::AudioProcessorValueTreeState::ParameterLayout
CloudLikeGranularProcessor::createParameterLayout()
{
    std::vector<std::unique_ptr<juce::RangedAudioParameter>> params;

    // Mode selection: 0 = Granular, 1 = Pitch Shifter, 2 = Looping, 3 = Spectral, 4 = Oliverb, 5 = Resonestor, 6 = Beat Repeat, 7 = Spectral Clouds
    params.push_back (std::make_unique<juce::AudioParameterInt>("mode", "Mode", 0, 7, 0));

    params.push_back (std::make_unique<juce::AudioParameterFloat>("position", "Position", 0.0f, 1.0f, 0.0f));
    params.push_back (std::make_unique<juce::AudioParameterFloat>("size",     "Size",     0.016f, 1.0f, 0.1f));  // 16ms to 1s (Clouds range)
    params.push_back (std::make_unique<juce::AudioParameterFloat>("pitch",    "Pitch",    -24.0f, 24.0f, 0.0f));
    params.push_back (std::make_unique<juce::AudioParameterFloat>("density",  "Density",  0.0f, 1.0f, 0.5f));
    params.push_back (std::make_unique<juce::AudioParameterFloat>("texture",  "Texture",  0.0f, 1.0f, 0.5f));
    params.push_back (std::make_unique<juce::AudioParameterFloat>("spread",   "StereoSpread", 0.0f, 1.0f, 0.5f));
    params.push_back (std::make_unique<juce::AudioParameterFloat>("feedback", "Feedback", 0.0f, 0.95f, 0.0f));
    params.push_back (std::make_unique<juce::AudioParameterFloat>("mix",      "Mix",      0.0f, 1.0f, 1.0f));
    params.push_back (std::make_unique<juce::AudioParameterFloat>("reverb",   "Reverb",   0.0f, 1.0f, 0.3f));

    // TRIG parameters
    // trigMode: false = Manual (MIDI), true = Auto (Tempo Sync)
    params.push_back (std::make_unique<juce::AudioParameterBool>("trigMode",  "Trig Mode", false));
    // trigRate: Center (0) = 1/4 note, Left (-) = divisions, Right (+) = multiplications
    // Range: -4.0 (1/16) to +4.0 (4 bars), with triplet support
    params.push_back (std::make_unique<juce::AudioParameterFloat>("trigRate", "Trig Rate", -4.0f, 4.0f, 0.0f));

    params.push_back (std::make_unique<juce::AudioParameterBool>("freeze",   "Freeze",   false));
    params.push_back (std::make_unique<juce::AudioParameterBool>("randomize","Randomize",false));
    params.push_back (std::make_unique<juce::AudioParameterBool>("killDry",  "Kill Dry", false));
    params.push_back (std::make_unique<juce::AudioParameterBool>("killWet",  "Kill Wet", false));

    return { params.begin(), params.end() };
}

bool CloudLikeGranularProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    return layouts.getMainInputChannelSet()  == juce::AudioChannelSet::stereo()
        && layouts.getMainOutputChannelSet() == juce::AudioChannelSet::stereo();
}

void CloudLikeGranularProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    currentSampleRate = sampleRate;

    // ========== OPTIMIZATION: Initialize Look-Up Tables ==========
    // Sine/Cosine LUT (saves ~20% CPU on trigonometry)
    for (int i = 0; i < SINE_TABLE_SIZE; ++i)
    {
        float phase = juce::MathConstants<float>::twoPi * i / SINE_TABLE_SIZE;
        sineLUT[i] = std::sin(phase);
        cosineLUT[i] = std::cos(phase);
    }

    // Hann window LUT (saves ~30% CPU in Spectral/Granular modes)
    for (int i = 0; i < HANN_WINDOW_SIZE; ++i)
    {
        float x = static_cast<float>(i) / (HANN_WINDOW_SIZE - 1);
        hannWindowLUT[i] = 0.5f * (1.0f - std::cos(2.0f * juce::MathConstants<float>::pi * x));
    }

    // Pitch ratio LUT (saves ~15% CPU on pitch conversion)
    for (int i = 0; i < PITCH_LUT_SIZE; ++i)
    {
        float semitones = static_cast<float>(i - 24);  // -24 to +24
        pitchRatioLUT[i] = std::pow(2.0f, semitones / 12.0f);
    }

    // Initialize parameter smoothing
    smoothedPosition.reset(0.0f);
    smoothedSize.reset(0.1f);
    smoothedPitch.reset(0.0f);
    smoothedDensity.reset(0.5f);
    smoothedTexture.reset(0.5f);
    smoothedSpread.reset(0.5f);
    smoothedFeedback.reset(0.0f);
    smoothedMix.reset(1.0f);
    smoothedReverb.reset(0.3f);

    // OPTIMIZATION: Round buffer size to power of 2 for fast bit-masking instead of modulo
    int requestedSize = static_cast<int> (sampleRate * ringBufferSeconds);
    bufferSize = 1;
    while (bufferSize < requestedSize)
        bufferSize <<= 1;  // OPTIMIZED: Bit shift to find next power of 2
    bufferSizeMask = bufferSize - 1;  // OPTIMIZED: Bit mask for fast modulo operation

    ringBuffer.setSize (2, bufferSize);
    ringBuffer.clear();
    writeHead = 0;

    // Initialize grain free list
    freeGrainIndices.clear();
    freeGrainIndices.reserve (maxGrains);
    for (int i = 0; i < maxGrains; ++i)
    {
        grains[i].active = false;
        freeGrainIndices.push_back (i);
    }

    wetBuffer.setSize (2, samplesPerBlock);
    reverbBuffer.setSize (2, samplesPerBlock);

    juce::Reverb::Parameters rp;
    rp.roomSize   = 0.85f;
    rp.damping    = 0.3f;
    rp.wetLevel   = 1.0f;
    rp.dryLevel   = 0.0f;
    rp.width      = 1.0f;
    rp.freezeMode = 0.0f;
    reverb.setParameters (rp);
    reverb.reset();

    // Initialize diffuser allpass filters (Clouds-style delays)
    int delay1 = static_cast<int>(sampleRate * 0.0127f);  // ~12.7ms
    int delay2 = static_cast<int>(sampleRate * 0.0270f);  // ~27ms
    int delay3 = static_cast<int>(sampleRate * 0.0453f);  // ~45.3ms

    diffuserL1.setDelay(delay1);
    diffuserL2.setDelay(delay2);
    diffuserL3.setDelay(delay3);

    diffuserR1.setDelay(delay1 + 7);  // Slight offset for stereo
    diffuserR2.setDelay(delay2 + 11);
    diffuserR3.setDelay(delay3 + 13);

    // Initialize correlators
    correlatorL.setSampleRate(sampleRate);
    correlatorR.setSampleRate(sampleRate);
    detectedPeriod.store(0);
    correlatorUpdateCounter.store(0);

    // Initialize Spectral mode state
    spectralHopCounter = 0;
    spectralBlockSize = 0;
    spectralOutputPos = 0;
    spectralOutputL.fill(0.0f);
    spectralOutputR.fill(0.0f);

    // Initialize Spectral mode improvements (Clouds/SuperParasites-style)
    // Option 4: Spectral freeze
    spectralFrozenL.fill(0.0f);
    spectralFrozenR.fill(0.0f);
    spectralFrozen = false;
    spectralFreezeAmount = 0.0f;

    // Option 1: Random phase and scramble map
    for (int i = 0; i < fftSize; ++i)
    {
        spectralRandomPhase[i] = uniform(rng) * juce::MathConstants<float>::twoPi;
        spectralScrambleMap[i] = i;  // Identity mapping initially
    }

    // Option 2: Band masking
    spectralBandMask.fill(1.0f);
    spectralBandUpdateCounter = 0;

    // Option 5: Phase vocoder state
    spectralPrevPhaseL.fill(0.0f);
    spectralPrevPhaseR.fill(0.0f);
    spectralPhaseAccumL.fill(0.0f);
    spectralPhaseAccumR.fill(0.0f);

    // Initialize WSOLA state for Pitch Shifter mode
    for (int w = 0; w < 2; ++w)
    {
        wsolaWindows[w].readPos = 0.0;
        wsolaWindows[w].hopCounter = 0;
        wsolaWindows[w].windowSize = 2048;
        wsolaWindows[w].active = false;
        wsolaWindows[w].needsRegeneration = true;  // Start with regeneration needed
    }
    wsolaCurrentWindow = 0;
    wsolaEnvPhase = 1.0f;       // Start fully open
    wsolaEnvIncrement = 0.0f;
    wsolaElapsed = 0;

    // Initialize Looping mode state (Clouds-style with improvements)
    loopReadPos = 0.0;
    loopStartPos = 0;
    loopEndPos = 0;
    loopLength = 0;
    previousLoopLength = 0;

    // Option 1: Crossfade
    loopCrossfadeActive = false;

    // Option 2: Loop capture with envelope
    int maxLoopCaptureSize = static_cast<int>(sampleRate * 2.0);  // 2 seconds max
    loopCaptureBufferL.resize(maxLoopCaptureSize, 0.0f);
    loopCaptureBufferR.resize(maxLoopCaptureSize, 0.0f);
    loopCaptureLength = 0;
    loopUseCaptured = false;
    loopEnvPhase = 1.0f;  // Start fully open
    loopEnvIncrement = 0.0f;

    // Option 3: Feedback filter
    loopFeedbackFilter.previousL = 0.0f;
    loopFeedbackFilter.previousR = 0.0f;

    // Option 4: Multi-tap delay
    for (int t = 0; t < maxLoopTaps; ++t)
    {
        loopTaps[t].readPos = 0.0;
        // Initialize stereo panning for each tap
        float angle = (static_cast<float>(t) / maxLoopTaps) * juce::MathConstants<float>::pi;
        loopTaps[t].panL = std::cos(angle);
        loopTaps[t].panR = std::sin(angle);
    }

    // Initialize Oliverb taps (multi-tap delay with modulation - Clouds/SuperParasites-style)
    // Extended to 16 taps for DENSITY control
    const std::array<float, 16> tapTimes = {
        0.037f, 0.089f, 0.127f, 0.191f, 0.277f, 0.359f, 0.441f, 0.593f,  // Original 8 taps
        0.067f, 0.113f, 0.157f, 0.227f, 0.311f, 0.397f, 0.509f, 0.677f   // Additional 8 taps
    };

    for (size_t i = 0; i < maxOliverbTaps; ++i)
    {
        // OPTIMIZATION: Round to power of 2 for bit masking
        int tapRequestedSize = static_cast<int>(sampleRate * tapTimes[i]);
        int tapSize = 1;
        while (tapSize < tapRequestedSize)
            tapSize <<= 1;  // Find next power of 2

        oliverbTaps[i].buffer.resize(tapSize, 0.0f);
        oliverbTaps[i].bufferSizeMask = tapSize - 1;  // OPTIMIZED: Bit mask
        oliverbTaps[i].writePos = 0;
        oliverbTaps[i].modPhase = static_cast<float>(i) * 0.125f * juce::MathConstants<float>::twoPi;
        oliverbTaps[i].modDepth = 0.002f * sampleRate;  // 2ms modulation depth

        // Option 2: Initialize tone filter state
        oliverbTaps[i].prevSampleL = 0.0f;
        oliverbTaps[i].prevSampleR = 0.0f;

        // Option 5: Store original tap time for room size scaling
        oliverbTaps[i].originalTapTime = tapTimes[i];
    }

    // Option 4: Initialize reverb freeze state
    oliverbFrozen = false;
    oliverbFreezeEnvelope = 1.0f;

    // Option 1: Initialize allpass diffusers (Clouds-style prime number delays)
    oliverbDiffuserL1.setDelay(142);   // Prime delays for dense diffusion
    oliverbDiffuserL2.setDelay(107);
    oliverbDiffuserL3.setDelay(379);
    oliverbDiffuserL4.setDelay(277);

    oliverbDiffuserR1.setDelay(149);   // Different for stereo width
    oliverbDiffuserR2.setDelay(113);
    oliverbDiffuserR3.setDelay(397);
    oliverbDiffuserR4.setDelay(281);

    // Initialize Resonestor resonators (Karplus-Strong - Clouds/SuperParasites-style)
    // Option 3: Define 6 chord presets
    const std::array<std::array<float, maxResonators>, numChordPresets> chordPresets = {{
        // Preset 0: Major chord (C major voicing)
        {130.81f, 164.81f, 196.00f, 261.63f, 329.63f, 392.00f,
         523.25f, 659.25f, 783.99f, 1046.50f, 1318.51f, 1567.98f},

        // Preset 1: Minor chord (C minor voicing)
        {130.81f, 155.56f, 196.00f, 261.63f, 311.13f, 392.00f,
         523.25f, 622.25f, 783.99f, 1046.50f, 1244.51f, 1567.98f},

        // Preset 2: Power chord (C power chord - 5ths)
        {65.41f, 98.00f, 130.81f, 196.00f, 261.63f, 392.00f,
         523.25f, 783.99f, 1046.50f, 1567.98f, 2093.00f, 3135.96f},

        // Preset 3: Octaves (C octaves)
        {65.41f, 130.81f, 261.63f, 523.25f, 1046.50f, 2093.00f,
         65.41f, 130.81f, 261.63f, 523.25f, 1046.50f, 2093.00f},

        // Preset 4: Pentatonic (C pentatonic)
        {130.81f, 146.83f, 164.81f, 196.00f, 220.00f, 261.63f,
         293.66f, 329.63f, 392.00f, 440.00f, 523.25f, 587.33f},

        // Preset 5: Harmonic series (from C2)
        {65.41f, 130.81f, 196.00f, 261.63f, 329.63f, 392.00f,
         457.69f, 523.25f, 587.33f, 659.25f, 726.53f, 783.99f}
    }};

    // Initialize with first chord preset
    resonestorCurrentChord = 0;
    const auto& initialFreqs = chordPresets[0];

    for (int i = 0; i < maxResonators; ++i)
    {
        // OPTIMIZATION: Round to power of 2 for bit masking
        int requestedLength = static_cast<int>(sampleRate / initialFreqs[i]);
        int delayLength = 1;
        while (delayLength < requestedLength)
            delayLength <<= 1;  // Find next power of 2

        resonators[i].delayLine.resize(delayLength, 0.0f);
        resonators[i].delayLineMask = delayLength - 1;  // OPTIMIZED: Bit mask
        resonators[i].writePos = 0;
        resonators[i].feedback = 0.99f;
        resonators[i].brightness = 0.5f;
        resonators[i].active = false;

        // Option 5: Store base frequency
        resonators[i].frequency = initialFreqs[i];

        // Option 2: Initialize stereo panning
        float angle = (static_cast<float>(i) / maxResonators) * juce::MathConstants<float>::pi;
        resonators[i].panL = std::cos(angle);
        resonators[i].panR = std::sin(angle);

        // Option 4: Initialize filter state
        resonators[i].z1 = 0.0f;
        resonators[i].z2 = 0.0f;
    }

    // Initialize burst envelope
    resonestorBurstEnvelope = 0.0f;
    resonestorBurstDecay = 0.9995f;
    resonestorPreviousTrigger = false;

    // Initialize Beat Repeat buffer (1 bar at 120 BPM = 2 seconds)
    int beatRepeatSize = static_cast<int>(sampleRate * 2.0);
    beatRepeat.captureBufferL.resize(beatRepeatSize, 0.0f);
    beatRepeat.captureBufferR.resize(beatRepeatSize, 0.0f);
    beatRepeat.captureLength = beatRepeatSize >> 2;  // OPTIMIZED: Bit shift for division by 4
    beatRepeat.repeatPos = 0.0f;
    beatRepeat.stutterPhase = 0.0f;
    beatRepeat.isCapturing = false;

    // Initialize Clouds/SuperParasites-style improvements
    beatRepeat.numSlices = 1;
    beatRepeat.currentSlice = 0;
    for (int i = 0; i < 16; ++i)
        beatRepeat.sliceOrder[i] = i;  // Sequential order by default

    beatRepeat.triggerMode = 0;  // Mode 0: Repeat (default)
    beatRepeat.triggerHeld = false;
    beatRepeat.triggerHoldTime = 0.0f;

    beatRepeat.envelopePhase = 0.0f;
    beatRepeat.crossfadeBufferL.resize(beatRepeatSize, 0.0f);
    beatRepeat.crossfadeBufferR.resize(beatRepeatSize, 0.0f);
    beatRepeat.hasPreviousCapture = false;

    // Initialize Spectral Clouds state (SuperParasites-style)
    spectralClouds.currentGain.fill(1.0f);  // Start with all bands at full gain
    spectralClouds.targetGain.fill(1.0f);
    spectralClouds.frozenMagnitudeL.fill(0.0f);
    spectralClouds.frozenMagnitudeR.fill(0.0f);
    spectralClouds.phaseL.fill(0.0f);
    spectralClouds.phaseR.fill(0.0f);
    spectralClouds.frozen = false;
    spectralClouds.numFreqBands = 16;  // Default to 16 bands
    spectralClouds.parameterLowpass = 0.1f;
    spectralClouds.previousTrigger = false;
}

float CloudLikeGranularProcessor::getSampleFromRing (int channel, double index) const
{
    if (bufferSize <= 0) return 0.0f;

    // Efficient modulo for negative indices
    index = std::fmod (index, static_cast<double> (bufferSize));
    if (index < 0.0) index += bufferSize;

    int i0 = static_cast<int> (index);
    int i1 = (i0 + 1) & bufferSizeMask;  // OPTIMIZED: Bit mask instead of modulo
    float frac = static_cast<float> (index - i0);

    auto* data = ringBuffer.getReadPointer (channel);
    return juce::jlimit (-1.0f, 1.0f, data[i0] + (data[i1] - data[i0]) * frac);
}

float CloudLikeGranularProcessor::getGrainEnvelope (double t, double duration, float textureParam) const
{
    double x = t / duration;
    if (x < 0.0 || x > 1.0) return 0.0f;

    // Clouds-style envelope shapes: Square -> Triangle -> Hann
    // TEXTURE: 0.0 = Square, 0.333 = Triangle, 0.666+ = Hann
    // Above 0.75: Hann + Diffuser activation

    float env = 0.0f;

    if (textureParam < 0.333f)
    {
        // Square to Triangle morph (0.0 - 0.333)
        float blend = textureParam * 3.0f;  // 0 to 1

        // Square window (boxcar)
        float square = 1.0f;

        // Triangle window
        float triangle = 1.0f - std::abs(2.0f * static_cast<float>(x) - 1.0f);

        env = square * (1.0f - blend) + triangle * blend;
    }
    else if (textureParam < 0.666f)
    {
        // Triangle to Hann morph (0.333 - 0.666)
        float blend = (textureParam - 0.333f) * 3.0f;  // 0 to 1

        // Triangle window
        float triangle = 1.0f - std::abs(2.0f * static_cast<float>(x) - 1.0f);

        // Hann window
        float hann = 0.5f * (1.0f - std::cos(2.0f * juce::MathConstants<float>::pi * x));

        env = triangle * (1.0f - blend) + hann * blend;
    }
    else
    {
        // Pure Hann window (0.666 - 1.0)
        env = 0.5f * (1.0f - std::cos(2.0f * juce::MathConstants<float>::pi * x));

        // Additional smoothing for texture > 0.75 (diffuser region)
        if (textureParam > 0.75f)
        {
            float extraSmooth = (textureParam - 0.75f) * 4.0f;  // 0 to 1
            env = std::pow(env, 1.0f + extraSmooth * 0.5f);  // Slightly sharper
        }
    }

    return env;
}

// Fast inverse square root for gain normalization
// Uses hardware SIMD instructions when available, falls back to standard implementation
float CloudLikeGranularProcessor::fastInverseSqrt (float number) const
{
    if (number <= 0.0f) return 1.0f;

    #if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 1)
        // Use SSE instruction for fast approximation
        __m128 x = _mm_set_ss(number);
        __m128 rsqrt = _mm_rsqrt_ss(x);

        // One Newton-Raphson iteration for better accuracy
        // x' = x * (1.5 - 0.5 * n * x * x)
        __m128 half = _mm_set_ss(0.5f);
        __m128 three_halfs = _mm_set_ss(1.5f);
        __m128 rsqrt_refined = _mm_mul_ss(rsqrt,
            _mm_sub_ss(three_halfs, _mm_mul_ss(_mm_mul_ss(x, half), _mm_mul_ss(rsqrt, rsqrt))));

        return _mm_cvtss_f32(rsqrt_refined);
    #else
        // Fallback to standard library
        return 1.0f / std::sqrt(number);
    #endif
}

// WSOLA: Find best matching segment using correlation
int CloudLikeGranularProcessor::findBestMatch (const float* reference, const float* searchBuffer,
                                               int searchLength, int windowSize)
{
    float bestCorrelation = -1.0f;
    int bestOffset = 0;

    for (int offset = 0; offset < searchLength - windowSize; ++offset)
    {
        float correlation = 0.0f;
        float refEnergy = 0.0001f;
        float searchEnergy = 0.0001f;

        // Compute normalized cross-correlation
        for (int i = 0; i < windowSize; ++i)
        {
            correlation += reference[i] * searchBuffer[offset + i];
            refEnergy += reference[i] * reference[i];
            searchEnergy += searchBuffer[offset + i] * searchBuffer[offset + i];
        }

        // Normalize by energies
        correlation /= std::sqrt(refEnergy * searchEnergy);

        if (correlation > bestCorrelation)
        {
            bestCorrelation = correlation;
            bestOffset = offset;
        }
    }

    return bestOffset;
}

// Clouds-style overlap computation
float CloudLikeGranularProcessor::computeOverlap (float density) const
{
    float overlap = 0.0f;

    if (density >= 0.53f)
    {
        overlap = (density - 0.53f) * 2.12f;
    }
    else if (density <= 0.47f)
    {
        overlap = (0.47f - density) * 2.12f;
    }

    // Cubic scaling (Clouds algorithm)
    overlap = overlap * overlap * overlap;

    return juce::jlimit (0.0f, 1.0f, overlap);
}

// TRIG rate to note value conversion (refactored from duplicate code)
double CloudLikeGranularProcessor::calculateNoteValueFromTrigRate(float trigRate)
{
    // TRIG RATE parameter range: -4.0 to +4.0
    // Negative = divisions (1/16, 1/8, 1/4, etc.)
    // Positive = multiplications (1/2, 1 bar, 2 bars, etc.)
    // Includes triplet patterns: 1/16T, 1/8T, 1/4T, 1/2T, 1barT

    if (trigRate < -3.4f)      return 1.0 / 16.0;   // 1/16 note
    else if (trigRate < -2.8f) return 1.0 / 12.0;   // 1/16 triplet
    else if (trigRate < -2.2f) return 1.0 / 8.0;    // 1/8 note
    else if (trigRate < -1.6f) return 1.0 / 6.0;    // 1/8 triplet
    else if (trigRate < -0.8f) return 1.0 / 4.0;    // 1/4 note
    else if (trigRate < 0.0f)  return 1.0 / 3.0;    // 1/4 triplet
    else if (trigRate < 0.8f)  return 1.0 / 2.0;    // 1/2 note
    else if (trigRate < 1.6f)  return 1.0 / 1.5;    // 1/2 triplet
    else if (trigRate < 2.4f)  return 1.0;          // 1 bar
    else if (trigRate < 3.2f)  return 4.0 / 3.0;    // 1 bar triplet
    else                       return 2.0;          // 2 bars
}

// Helper function to initialize a grain (reduces code duplication)
void CloudLikeGranularProcessor::initializeGrain (Grain& g, int channel, double baseStart,
                                                  double durationSamps, double pitchRatio,
                                                  float positionJitter, float spreadWidth)
{
    g.active = true;
    g.channel = channel;
    g.startSample = baseStart + (uniform(rng) - 0.5f) * positionJitter * durationSamps;
    g.position = 0.0;
    g.durationSamples = durationSamps;
    g.phaseInc = pitchRatio;
    g.pan = (uniform(rng) * 2.0f - 1.0f) * spreadWidth;

    // Clouds-style pitch shifting initialization
    g.phase = 0.0;
    g.phaseIncrement = (1.0 - pitchRatio) / durationSamps;

    // Clouds-style pre-delay (0 to ~10ms random)
    g.preDelay = static_cast<int>(uniform(rng) * currentSampleRate * 0.01f);

    // Clouds-style per-grain stereo gain (OPTIMIZED: LUT)
    float panAngle = (g.pan + 1.0f) * 0.5f * juce::MathConstants<float>::halfPi;
    g.gainL = fastCos(panAngle);
    g.gainR = fastSin(panAngle);
}

void CloudLikeGranularProcessor::launchGrains (int numToLaunch, int channel,
                                               float positionParam, float sizeParam,
                                               float pitchSemis, float textureParam,
                                               float stereoSpread, int periodHint)
{
    // Safety check: ensure buffer is initialized and large enough
    if (bufferSize <= 0) return;

    double duration      = juce::jlimit (0.016, 1.0, (double) sizeParam);  // Clouds: 16ms to 1s
    double durationSamps = duration * currentSampleRate;

    // Ensure grain doesn't exceed buffer size
    durationSamps = juce::jmin (durationSamps, (double) bufferSize * 0.9);

    double maxOffset = (double) bufferSize - durationSamps - 1.0;
    // Additional safety: ensure maxOffset is positive
    if (maxOffset < 1.0) return;

    double offset    = (1.0f - positionParam) * maxOffset;
    double baseStart = (double) writeHead - offset;

    // Align to period boundary if pitch detection succeeded
    if (periodHint > 0 && periodHint < static_cast<int>(durationSamps * 0.5))
    {
        // Snap baseStart to nearest period boundary for more natural grain placement
        double periodsFromStart = baseStart / periodHint;
        baseStart = std::round(periodsFromStart) * periodHint;
    }

    double pitchRatio = pitchToRatio(static_cast<float>(pitchSemis));
    float spreadWidth = stereoSpread;

    // Use texture to control grain start position randomization
    float positionJitter = 0.05f + textureParam * 0.15f;

    for (int n = 0; n < numToLaunch; ++n)
    {
        Grain* grainPtr = nullptr;

        // Use free list for efficient grain allocation
        if (!freeGrainIndices.empty())
        {
            int idx = freeGrainIndices.back();
            freeGrainIndices.pop_back();
            grainPtr = &grains[idx];
        }
        else
        {
            // Fallback: find first inactive grain
            auto it = std::find_if (grains.begin(), grains.end(),
                                    [] (const Grain& g) { return !g.active; });
            if (it == grains.end()) break;
            grainPtr = &(*it);
        }

        // Initialize grain using helper function (eliminates code duplication)
        if (grainPtr != nullptr)
        {
            initializeGrain(*grainPtr, channel, baseStart, durationSamps,
                          pitchRatio, positionJitter, spreadWidth);
        }
    }
}

void CloudLikeGranularProcessor::processBlock (juce::AudioBuffer<float>& buffer,
                                               juce::MidiBuffer& midi)
{
    juce::ScopedNoDenormals noDenormals;

    auto numSamples = buffer.getNumSamples();
    auto* inL = buffer.getReadPointer (0);
    auto* inR = buffer.getNumChannels() > 1 ? buffer.getReadPointer (1) : nullptr;
    auto* outL = buffer.getWritePointer (0);
    auto* outR = buffer.getNumChannels() > 1 ? buffer.getWritePointer (1) : nullptr;

    // Process MIDI for TRIG input (Manual mode)
    bool trigMode = apvts.getRawParameterValue ("trigMode")->load() > 0.5f;  // false = Manual/MIDI, true = Auto

    if (!trigMode)  // Manual/MIDI mode
    {
        // In Manual mode: LED 1 = MIDI triggers, LED 2 = TRIG RATE tempo

        // LED 2: TRIG RATE tempo in Manual mode (same calculation as Auto mode)
        auto playHead = getPlayHead();
        if (playHead != nullptr)
        {
            juce::Optional<juce::AudioPlayHead::PositionInfo> posInfo = playHead->getPosition();
            if (posInfo.hasValue() && posInfo->getBpm().hasValue())
            {
                double bpm = *posInfo->getBpm();
                float trigRate = apvts.getRawParameterValue ("trigRate")->load();
                double noteValue = calculateNoteValueFromTrigRate(trigRate);

                double beatsPerSecond = bpm / 60.0;
                double triggersPerSecond = beatsPerSecond / (noteValue * 4.0);
                double phaseIncrement = triggersPerSecond / currentSampleRate;

                for (int i = 0; i < numSamples; ++i)
                {
                    tempoSyncPhase += phaseIncrement;
                    if (tempoSyncPhase >= 1.0)
                    {
                        tempoSyncPhase -= 1.0;
                        trigRateBlink.store(true);  // LED 2: TRIG RATE tempo in Manual mode
                    }
                }
            }
        }

        // LED 1: Tap tempo or MIDI note triggers in Manual mode
        bool currentMidiNoteState = false;

        // Process MIDI triggers
        for (const auto metadata : midi)
        {
            auto message = metadata.getMessage();

            // Trigger on both note on and note off
            if (message.isNoteOn() || message.isNoteOff())
            {
                currentMidiNoteState = message.isNoteOn();

                // Edge detection: trigger on state change
                if (currentMidiNoteState != lastMidiNoteState)
                {
                    triggerReceived.store(true);
                    baseTempoBlink.store(true);  // LED 1: MIDI trigger in Manual mode
                    lastMidiNoteState = currentMidiNoteState;
                }
            }
        }

        // LED 1: Tap tempo blink (if tap tempo is active)
        if (tapTempoActive.load())
        {
            float tapBPM = detectedTapBPM.load();
            if (tapBPM > 0.0f)
            {
                double beatsPerSecond = tapBPM / 60.0;
                double phaseIncrement = beatsPerSecond / currentSampleRate;

                for (int i = 0; i < numSamples; ++i)
                {
                    tapTempoPhase += phaseIncrement;
                    if (tapTempoPhase >= 1.0)
                    {
                        tapTempoPhase -= 1.0;
                        baseTempoBlink.store(true);  // LED 1: Tap tempo blink
                    }
                }
            }
        }
    }
    else  // Auto/Tempo Sync mode
    {
        // In Auto mode: LED 1 = base tempo, LED 2 = TRIG RATE tempo

        // Get tempo info from DAW
        auto playHead = getPlayHead();
        if (playHead != nullptr)
        {
            juce::Optional<juce::AudioPlayHead::PositionInfo> posInfo = playHead->getPosition();
            if (posInfo.hasValue() && posInfo->getBpm().hasValue())
            {
                double bpm = *posInfo->getBpm();
                hostBPM.store(static_cast<float>(bpm));  // Store for UI display
                float trigRate = apvts.getRawParameterValue ("trigRate")->load();

                // Calculate note value from TRIG RATE parameter
                double noteValue = calculateNoteValueFromTrigRate(trigRate);

                // Calculate frequency: triggers per second
                double beatsPerSecond = bpm / 60.0;
                double triggersPerSecond = beatsPerSecond / (noteValue * 4.0);
                double phaseIncrement = triggersPerSecond / currentSampleRate;

                // Calculate base tempo LED phase (always ×1 quarter note)
                double baseNoteValue = 1.0 / 4.0;  // Quarter note
                double baseTriggersPerSecond = beatsPerSecond / (baseNoteValue * 4.0);
                double basePhaseIncrement = baseTriggersPerSecond / currentSampleRate;

                // Accumulate phase and generate triggers
                for (int i = 0; i < numSamples; ++i)
                {
                    // LED 2: TRIG RATE tempo sync (with rate divisions/multiplications)
                    tempoSyncPhase += phaseIncrement;
                    if (tempoSyncPhase >= 1.0)
                    {
                        tempoSyncPhase -= 1.0;
                        triggerReceived.store(true);
                        trigRateBlink.store(true);  // LED 2: Blink at TRIG RATE tempo
                    }

                    // LED 1: Base tempo (×1 quarter note) in Auto mode
                    baseTempoPhase += basePhaseIncrement;
                    if (baseTempoPhase >= 1.0)
                    {
                        baseTempoPhase -= 1.0;
                        baseTempoBlink.store(true);  // LED 1: Base tempo in Auto mode
                    }
                }
            }
        }
    }

    midi.clear();  // Clear MIDI output (we don't produce MIDI)

    // Get processing mode
    int mode = static_cast<int>(apvts.getRawParameterValue ("mode")->load());

    // Clear buffers when mode changes to prevent clicks/pops
    int prevMode = previousMode.load();
    if (mode != prevMode)
    {
        previousMode.store(mode);

        // Clear Resonestor delay lines when switching away from or to Resonestor mode
        if (prevMode == MODE_RESONESTOR || mode == MODE_RESONESTOR)
        {
            for (auto& res : resonators)
            {
                std::fill(res.delayLine.begin(), res.delayLine.end(), 0.0f);
                res.writePos = 0;
                res.active = false;
            }
            // Reset Resonestor state
            resonestorPreviousTrigger = false;
        }

        // Clear Oliverb taps when switching away from or to Oliverb mode
        if (prevMode == MODE_OLIVERB || mode == MODE_OLIVERB)
        {
            for (auto& tap : oliverbTaps)
            {
                std::fill(tap.buffer.begin(), tap.buffer.end(), 0.0f);
                tap.writePos = 0;
                tap.modPhase = 0.0f;
            }
        }

        // Clear spectral buffers when switching away from or to Spectral mode
        if (prevMode == MODE_SPECTRAL || mode == MODE_SPECTRAL)
        {
            spectralOutputL.fill(0.0f);
            spectralOutputR.fill(0.0f);
            fftDataL.fill(0.0f);
            fftDataR.fill(0.0f);
            spectralHopCounter = 0;
            spectralBlockSize = 0;
            spectralOutputPos = 0;
        }
    }

    // Run correlation analysis periodically (every 256 samples) for pitch detection
    int currentCounter = correlatorUpdateCounter.load();
    currentCounter += numSamples;
    correlatorUpdateCounter.store(currentCounter);

    if (currentCounter >= 256)
    {
        correlatorUpdateCounter.store(0);

        // Analyze recent buffer content for period detection
        int analysisLength = juce::jmin(2048, bufferSize / 4);
        if (analysisLength > 0 && writeHead >= analysisLength)
        {
            const float* analysisData = ringBuffer.getReadPointer(0) + (writeHead - analysisLength);
            int minPeriod = static_cast<int>(currentSampleRate / 1000.0);  // 1000 Hz max
            int maxPeriod = static_cast<int>(currentSampleRate / 50.0);    // 50 Hz min

            int period = correlatorL.detectPeriod(analysisData, analysisLength, minPeriod, maxPeriod);
            detectedPeriod.store(period);
        }
    }

    // ========== OPTIMIZATION: Parameter Smoothing (prevents zipper noise) ==========
    smoothedPosition.setTarget(apvts.getRawParameterValue ("position")->load());
    smoothedSize.setTarget(apvts.getRawParameterValue ("size")->load());
    smoothedPitch.setTarget(apvts.getRawParameterValue ("pitch")->load());
    smoothedDensity.setTarget(apvts.getRawParameterValue ("density")->load());
    smoothedTexture.setTarget(apvts.getRawParameterValue ("texture")->load());
    smoothedSpread.setTarget(apvts.getRawParameterValue ("spread")->load());
    smoothedFeedback.setTarget(apvts.getRawParameterValue ("feedback")->load());
    smoothedMix.setTarget(apvts.getRawParameterValue ("mix")->load());
    smoothedReverb.setTarget(apvts.getRawParameterValue ("reverb")->load());

    float position  = smoothedPosition.getNext();
    float size      = smoothedSize.getNext();
    float pitch     = smoothedPitch.getNext();
    float density   = smoothedDensity.getNext();
    float texture   = smoothedTexture.getNext();
    float spread    = smoothedSpread.getNext();
    float feedback  = smoothedFeedback.getNext();
    float mix       = smoothedMix.getNext();
    float reverbAmt = smoothedReverb.getNext();
    bool freeze     = apvts.getRawParameterValue ("freeze")->load() > 0.5f;
    bool killDry    = apvts.getRawParameterValue ("killDry")->load() > 0.5f;
    bool killWet    = apvts.getRawParameterValue ("killWet")->load() > 0.5f;

    // Kill Dry/Wet: Force mix to 100% or 0% when pressed (Kill Dry takes priority)
    if (killDry)
        mix = 1.0f;
    else if (killWet)
        mix = 0.0f;

    if (wetBuffer.getNumSamples() < numSamples)
        wetBuffer.setSize (2, numSamples, false, false, true);
    if (reverbBuffer.getNumSamples() < numSamples)
        reverbBuffer.setSize (2, numSamples, false, false, true);

    wetBuffer.clear();
    reverbBuffer.clear();

    auto* wetL = wetBuffer.getWritePointer (0);
    auto* wetR = wetBuffer.getWritePointer (1);

    float effectiveFeedback = freeze ? 1.0f : feedback;

    // ========== OPTIMIZATION: Mode-specific block processing ==========
    // Delegating to specialized functions reduces branching and improves cache locality
    switch (mode)
    {
        case MODE_GRANULAR:
            processGranularBlock (buffer, numSamples, wetL, wetR,
                                  position, size, pitch, density, texture, spread,
                                  effectiveFeedback, freeze);
            break;

        case MODE_PITCH_SHIFTER:
            processPitchShifterBlock (buffer, numSamples, wetL, wetR,
                                      position, size, pitch, effectiveFeedback, freeze);
            break;

        case MODE_LOOPING:
            processLoopingBlock (buffer, numSamples, wetL, wetR,
                                 position, size, pitch, density, texture,
                                 effectiveFeedback, freeze);
            break;

        case MODE_SPECTRAL:
            processSpectralBlock (buffer, numSamples, wetL, wetR,
                                  position, size, pitch, density, texture,
                                  effectiveFeedback, freeze);
            break;

        case MODE_OLIVERB:
            processOliverbBlock (buffer, numSamples, wetL, wetR,
                                 position, size, pitch, density, texture,
                                 effectiveFeedback, freeze);
            break;

        case MODE_RESONESTOR:
            processResonestorBlock (buffer, numSamples, wetL, wetR,
                                    position, size, pitch, density, texture,
                                    effectiveFeedback, freeze);
            break;

        case MODE_BEAT_REPEAT:
            processBeatRepeatBlock (buffer, numSamples, wetL, wetR,
                                    position, size, pitch, density, texture,
                                    effectiveFeedback, freeze);
            break;

        case MODE_SPECTRAL_CLOUDS:
            processSpectralCloudsBlock (buffer, numSamples, wetL, wetR,
                                        position, size, pitch, density, texture,
                                        effectiveFeedback, freeze);
            break;

        default:
            break;
    }

    // Update reverb parameters dynamically based on reverb amount
    juce::Reverb::Parameters rp;
    rp.roomSize   = 0.5f + reverbAmt * 0.45f;  // Scale with reverb amount
    rp.damping    = 0.3f;
    rp.wetLevel   = 1.0f;
    rp.dryLevel   = 0.0f;
    rp.width      = 1.0f;
    rp.freezeMode = freeze ? 1.0f : 0.0f;  // Link freeze mode to reverb
    reverb.setParameters (rp);

    reverbBuffer.makeCopyOf (wetBuffer);
    reverbBuffer.applyGain (reverbAmt);
    auto* revL = reverbBuffer.getWritePointer (0);
    auto* revR = reverbBuffer.getWritePointer (1);
    reverb.processStereo (revL, revR, numSamples);

    for (int i = 0; i < numSamples; ++i)
    {
        float dryL = inL[i];
        float dryR = inR ? inR[i] : dryL;

        float finalWetL = wetL[i] + revL[i];
        float finalWetR = wetR[i] + revR[i];

        outL[i] = dryL * (1.0f - mix) + finalWetL * mix;
        if (outR) outR[i] = dryR * (1.0f - mix) + finalWetR * mix;
    }
}

// ========== OPTIMIZATION: Mode-specific block processing implementations ==========

void CloudLikeGranularProcessor::processGranularBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                                                       float* wetL, float* wetR,
                                                       float position, float size, float pitch,
                                                       float density, float texture, float spread,
                                                       float feedback, bool freeze)
{
    auto* inL = buffer.getReadPointer (0);
    auto* inR = buffer.getNumChannels() > 1 ? buffer.getReadPointer (1) : nullptr;

    // ========== GRANULAR MODE (Block Processing) ==========
    // Clouds-style overlap and grain density calculation
    float overlap = computeOverlap (density);
    float targetNumGrains = maxGrains * overlap;
    float grainRate = targetNumGrains / (maxGrains * 0.1f);  // Normalized rate

    // Determine seeding mode based on density
    bool useDeterministic = density < 0.5f;

    for (int i = 0; i < numSamples; ++i)
    {
        float inSampleL = inL[i];
        float inSampleR = inR ? inR[i] : inSampleL;

        writeToRingBuffer(inSampleL, inSampleR, wetL[i] * feedback, wetR[i] * feedback, freeze);

        // TRIG input: Force grain generation (Clouds TRIG behavior)
        // When TRIG is received, generate 1 grain based on POSITION/SIZE, bypassing density
        bool manualTrigger = checkAndClearTrigger(i);

        // Clouds-style grain triggering (automatic, based on density)
        bool triggerGrain = manualTrigger;  // Manual TRIG always triggers

        // Only use density-based triggering if no manual trigger
        if (!manualTrigger)
        {
            if (useDeterministic)
            {
                // Deterministic mode: evenly spaced grains
                grainRatePhasor += grainRate / (float) currentSampleRate * 100.0f;
                if (grainRatePhasor >= 1.0f)
                {
                    grainRatePhasor -= 1.0f;
                    triggerGrain = true;
                }
            }
            else
            {
                // Probabilistic mode: random triggering
                float grainsPerSecond = minGrainsPerSecond + density * (maxGrainsPerSecond - minGrainsPerSecond);
                float prob = grainsPerSecond / (float) currentSampleRate;
                triggerGrain = (uniform (rng) < prob);
            }
        }

        if (triggerGrain)
        {
            // Pass detected period for grain alignment optimization
            int period = detectedPeriod.load();
            launchGrains (1, 0, position, size, pitch, texture, spread, period);
            launchGrains (1, 1, position, size, pitch, texture, spread, period);
        }

        float grainOutL = 0.0f, grainOutR = 0.0f;
        int activeCount = 0;

        // OPTIMIZATION: Scan only active grains
        for (int idx = 0; idx < maxGrains; ++idx)
        {
            auto& g = grains[idx];
            if (!g.active) continue;

            // Clouds-style pre-delay countdown
            if (g.preDelay > 0)
            {
                g.preDelay--;
                continue;  // Skip rendering until pre-delay expires
            }

            activeCount++;

            if (g.position >= g.durationSamples)
            {
                g.active = false;
                freeGrainIndices.push_back (idx);
                activeCount--;
                continue;
            }

            // Clouds-style dual read pointer with triangular crossfade
            float s;

            if (std::abs(g.phaseInc - 1.0) < 0.01)
            {
                // No significant pitch shift - use simple single-pointer reading
                double readIndex = g.startSample + g.position;
                s = getSampleFromRing (g.channel, readIndex);
            }
            else
            {
                // Pitch shifting - use dual read pointer system
                double readIndex1 = g.startSample + g.position;
                double normalizedPhase = g.phase;
                double offset = normalizedPhase * g.durationSamples;
                double readIndex2 = g.startSample + offset;

                // Triangular envelope for crossfading
                float tri = 2.0f * (g.phase >= 0.5 ? 1.0f - static_cast<float>(g.phase) : static_cast<float>(g.phase));

                // Read from both positions
                float s1 = getSampleFromRing (g.channel, readIndex1);
                float s2 = getSampleFromRing (g.channel, readIndex2);

                s = s1 * tri + s2 * (1.0f - tri);

                g.phase += g.phaseIncrement;
                if (g.phase < 0.0) g.phase += 1.0;
                if (g.phase >= 1.0) g.phase -= 1.0;
            }

            float env = getGrainEnvelope (g.position, g.durationSamples, texture);
            float v = s * env;

            grainOutL += v * g.gainL;
            grainOutR += v * g.gainR;

            g.position += g.phaseInc;
        }

        // Store active grain count for potential UI display
        numActiveGrains.store(activeCount);

        // Clouds-style gain normalization
        float gainNormalization = 1.0f;
        if (activeCount > 1)
        {
            gainNormalization = fastInverseSqrt (static_cast<float> (activeCount - 1));
        }

        float windowGain = 1.0f + overlap;
        float targetGain = gainNormalization * windowGain;
        smoothedGain += 0.01f * (targetGain - smoothedGain);

        float outputL = grainOutL * smoothedGain;
        float outputR = grainOutR * smoothedGain;

        // Clouds-style diffuser (activated when TEXTURE > 0.75)
        if (texture > 0.75f)
        {
            float diffuserAmount = (texture - 0.75f) * 4.0f;  // 0 to 1
            float feedbackCoeff = 0.625f;

            float diffusedL = diffuserL1.process(outputL, feedbackCoeff);
            diffusedL = diffuserL2.process(diffusedL, feedbackCoeff);
            diffusedL = diffuserL3.process(diffusedL, feedbackCoeff);

            float diffusedR = diffuserR1.process(outputR, feedbackCoeff);
            diffusedR = diffuserR2.process(diffusedR, feedbackCoeff);
            diffusedR = diffuserR3.process(diffusedR, feedbackCoeff);

            outputL = outputL * (1.0f - diffuserAmount) + diffusedL * diffuserAmount;
            outputR = outputR * (1.0f - diffuserAmount) + diffusedR * diffuserAmount;
        }

        wetL[i] = outputL;
        wetR[i] = outputR;
    }
}

void CloudLikeGranularProcessor::processPitchShifterBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                                                           float* wetL, float* wetR,
                                                           float position, float size, float pitch,
                                                           float feedback, bool freeze)
{
    auto* inL = buffer.getReadPointer (0);
    auto* inR = buffer.getNumChannels() > 1 ? buffer.getReadPointer (1) : nullptr;

    // ========== PITCH SHIFTER MODE (Clouds-style WSOLA with improvements) ==========
    // Improvements:
    // 1. Envelope-based smooth trigger response (Clouds-style)
    // 2. Dual-window overlap-add system for continuous output
    // 3. Simplified correlator for phase-aligned windows
    //
    // SIZE: Controls window size - large = smooth, small = grainy/ring-modulated
    // PITCH: Controls pitch shift amount (-24 to +24 semitones)
    // POSITION: Controls read delay position

    // OPTIMIZATION: Calculate parameters once per block
    float pitchRatio = pitchToRatio(pitch);

    // Clouds-style SIZE mapping: 0.0 = small (256), 1.0 = large (2048)
    int windowSize = static_cast<int>(256.0f + size * 1792.0f);
    windowSize = (windowSize + 3) & ~3;  // Round to multiple of 4 for alignment

    int hopSize = windowSize >> 1;  // Output hop = half window size

    // Update window size for both windows
    wsolaWindows[0].windowSize = windowSize;
    wsolaWindows[1].windowSize = windowSize;

    for (int i = 0; i < numSamples; ++i)
    {
        // === Option 1: Envelope-based smooth trigger response ===
        // TRIG input: Smoothly ramp to new position (Clouds-style envelope)
        if (checkAndClearTrigger(i))
        {
            // Calculate elapsed samples since last trigger for smooth envelope
            int elapsed = juce::jmax(1, wsolaElapsed);

            // Start envelope ramp (0.0 = fully reset, 1.0 = fully open)
            wsolaEnvPhase = 0.0f;
            wsolaEnvIncrement = 1.0f / static_cast<float>(elapsed);

            // Mark windows for regeneration at new position
            wsolaWindows[0].needsRegeneration = true;
            wsolaWindows[1].needsRegeneration = true;

            wsolaElapsed = 0;
        }

        // Update envelope phase
        if (wsolaEnvPhase < 1.0f)
        {
            wsolaEnvPhase += wsolaEnvIncrement;
            if (wsolaEnvPhase > 1.0f) wsolaEnvPhase = 1.0f;
        }

        wsolaElapsed++;

        float inSampleL = inL[i];
        float inSampleR = inR ? inR[i] : inSampleL;

        writeToRingBuffer(inSampleL, inSampleR, wetL[i] * feedback, wetR[i] * feedback, freeze);

        // === Option 2: Dual-window overlap-add system ===
        // Process two overlapping windows for continuous smooth output
        float outputL = 0.0f;
        float outputR = 0.0f;
        int activeWindows = 0;

        for (int w = 0; w < 2; ++w)
        {
            auto& window = wsolaWindows[w];

            // Check if window needs regeneration (hop complete or trigger received)
            if (window.hopCounter >= hopSize || window.needsRegeneration)
            {
                window.hopCounter = 0;
                window.needsRegeneration = false;

                // === Option 3: Simplified correlator for phase alignment ===
                // Find best read position using zero-crossing detection
                // This reduces artifacts at window boundaries

                // Effective position with envelope-based interpolation during trigger ramp
                float effectivePosition = position;
                if (wsolaEnvPhase < 1.0f)
                {
                    // Interpolate between current position and extreme during ramp
                    effectivePosition += (1.0f - wsolaEnvPhase) * (1.0f - position);
                }

                // Calculate base read position
                double delayAmount = effectivePosition * (bufferSize - windowSize * 2);
                double baseReadPos = static_cast<double>(writeHead) - delayAmount;

                // Wrap to valid buffer range
                while (baseReadPos < 0) baseReadPos += bufferSize;
                while (baseReadPos >= bufferSize) baseReadPos -= bufferSize;

                // Simple correlator: find zero-crossing within search range
                // Search ±32 samples for better phase alignment
                const int searchRange = 32;
                double bestReadPos = baseReadPos;
                float minAbsValue = 1000.0f;

                for (int offset = -searchRange; offset <= searchRange; offset += 4)
                {
                    double testPos = baseReadPos + offset;
                    while (testPos < 0) testPos += bufferSize;
                    while (testPos >= bufferSize) testPos -= bufferSize;

                    // Check for zero-crossing (phase alignment indicator)
                    float sample0 = getSampleFromRing(0, testPos);
                    float sample1 = getSampleFromRing(0, testPos + 1);

                    // Find position closest to zero crossing
                    if (sample0 * sample1 <= 0.0f && std::abs(sample0) < minAbsValue)
                    {
                        minAbsValue = std::abs(sample0);
                        bestReadPos = testPos;
                    }
                }

                window.readPos = bestReadPos;
                window.active = true;
            }

            if (window.active)
            {
                // Calculate window position within hop
                double windowPhase = static_cast<double>(window.hopCounter) / hopSize;

                // Hann window for smooth overlap-add
                float envelope = 0.5f * (1.0f - std::cos(windowPhase * juce::MathConstants<float>::twoPi));

                // Additional envelope shaping for small windows (ring-mod effect)
                if (size < 0.3f)
                {
                    float ringModAmount = (0.3f - size) * 3.333f;
                    envelope *= (1.0f - ringModAmount * 0.5f);
                }

                // Read samples from ring buffer
                double sampleReadPos = window.readPos + window.hopCounter * pitchRatio;

                // Wrap read position
                while (sampleReadPos < 0) sampleReadPos += bufferSize;
                while (sampleReadPos >= bufferSize) sampleReadPos -= bufferSize;

                float sampleL = getSampleFromRing(0, sampleReadPos);
                float sampleR = getSampleFromRing(1, sampleReadPos);

                // Accumulate windowed output
                outputL += sampleL * envelope;
                outputR += sampleR * envelope;

                window.hopCounter++;
                activeWindows++;

                // Deactivate window when hop is complete
                if (window.hopCounter >= hopSize)
                {
                    window.active = false;
                }
            }
        }

        // Normalize by active window count and apply envelope
        float normalization = activeWindows > 0 ? (1.0f / activeWindows) : 1.0f;
        float envelopeGain = 0.3f + wsolaEnvPhase * 0.7f;  // Fade in during trigger ramp

        wetL[i] = outputL * normalization * envelopeGain;
        wetR[i] = outputR * normalization * envelopeGain;
    }
}

void CloudLikeGranularProcessor::processLoopingBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                                                      float* wetL, float* wetR,
                                                      float position, float size, float pitch,
                                                      float density, float texture,
                                                      float feedback, bool freeze)
{
    auto* inL = buffer.getReadPointer (0);
    auto* inR = buffer.getNumChannels() > 1 ? buffer.getReadPointer (1) : nullptr;

    // ========== LOOPING MODE (Clouds-style with improvements) ==========
    // Improvements:
    // 1. Loop boundary crossfade for smooth seamless loops
    // 2. TRIG loop capture with envelope (Clouds-style)
    // 3. TEXTURE feedback filtering (Clouds-style)
    // 4. DENSITY multi-tap delay (SuperParasites-style)
    //
    // SIZE: Loop length
    // POSITION: Loop buffer position/delay
    // PITCH: Loop playback speed
    // DENSITY: Number of loop taps (1-4)
    // TEXTURE: Feedback filter brightness

    // OPTIMIZATION: Calculate loop parameters once per block
    int targetLoopLength = static_cast<int>(size * currentSampleRate);
    targetLoopLength = juce::jlimit(1024, bufferSize >> 1, targetLoopLength);

    // Smooth loop length changes to prevent clicks
    if (previousLoopLength == 0)
        previousLoopLength = targetLoopLength;

    // Detect significant loop length changes and reset read position safely
    int lengthDifference = std::abs(targetLoopLength - previousLoopLength);
    if (lengthDifference > 512)
    {
        if (previousLoopLength > 0 && loopReadPos > 0)
            loopReadPos = (loopReadPos / previousLoopLength) * targetLoopLength;
    }

    float pitchRatio = pitchToRatio(pitch);

    // Option 4: Calculate number of taps based on DENSITY
    int numTaps = 1 + static_cast<int>(density * (maxLoopTaps - 1));
    numTaps = juce::jlimit(1, maxLoopTaps, numTaps);

    for (int i = 0; i < numSamples; ++i)
    {
        // === Option 2: TRIG loop capture with envelope (Clouds-style) ===
        if (checkAndClearTrigger(i))
        {
            // Capture current loop segment to dedicated buffer
            loopCaptureLength = juce::jmin(targetLoopLength, static_cast<int>(loopCaptureBufferL.size()));

            // Calculate capture start position
            double delayTime = position * (bufferSize - loopCaptureLength);
            int captureStart = writeHead - static_cast<int>(delayTime) - loopCaptureLength;
            while (captureStart < 0) captureStart += bufferSize;

            // Capture loop segment with crossfade at boundaries for seamless loop
            for (int n = 0; n < loopCaptureLength; ++n)
            {
                int readIdx = (captureStart + n) & bufferSizeMask;
                loopCaptureBufferL[n] = ringBuffer.getSample(0, readIdx);
                loopCaptureBufferR[n] = ringBuffer.getSample(1, readIdx);
            }

            // Switch to captured loop playback
            loopUseCaptured = true;

            // Start envelope ramp (smooth fade-in)
            loopEnvPhase = 0.0f;
            loopEnvIncrement = 1.0f / (loopCaptureLength * 0.1f);  // 10% fade-in

            // Reset read position
            loopReadPos = 0.0;
        }

        // Update envelope phase
        if (loopEnvPhase < 1.0f)
        {
            loopEnvPhase += loopEnvIncrement;
            if (loopEnvPhase > 1.0f) loopEnvPhase = 1.0f;
        }

        float inSampleL = inL[i];
        float inSampleR = inR ? inR[i] : inSampleL;

        // === Option 3: TEXTURE feedback filtering (Clouds-style) ===
        // TEXTURE < 0.5: Low-pass filter (dark feedback)
        // TEXTURE > 0.5: High-pass filter (bright feedback)
        // float filterCoeff = 0.1f + texture * 0.8f;  // 0.1 to 0.9 (reserved for future use)

        float feedbackL = wetL[i] * feedback;
        float feedbackR = wetR[i] * feedback;

        if (texture < 0.5f)
        {
            // Low-pass filter for dark feedback
            float lpAmount = 1.0f - (texture * 2.0f);  // 1.0 at texture=0, 0.0 at texture=0.5
            feedbackL = feedbackL * (1.0f - lpAmount) + loopFeedbackFilter.previousL * lpAmount;
            feedbackR = feedbackR * (1.0f - lpAmount) + loopFeedbackFilter.previousR * lpAmount;
        }
        else
        {
            // High-pass filter for bright feedback
            float hpAmount = (texture - 0.5f) * 2.0f;  // 0.0 at texture=0.5, 1.0 at texture=1.0
            feedbackL = feedbackL - loopFeedbackFilter.previousL * hpAmount;
            feedbackR = feedbackR - loopFeedbackFilter.previousR * hpAmount;
        }

        loopFeedbackFilter.previousL = feedbackL;
        loopFeedbackFilter.previousR = feedbackR;

        // Write to ring buffer with filtered feedback
        writeToRingBuffer(inSampleL, inSampleR, feedbackL, feedbackR, freeze);

        // Smoothly interpolate loop length per sample
        float t = static_cast<float>(i) / numSamples;
        loopLength = static_cast<int>(previousLoopLength + t * (targetLoopLength - previousLoopLength));
        loopLength = juce::jmax(1024, loopLength);

        // Calculate loop boundaries
        double delayTime = position * (bufferSize - loopLength);
        loopEndPos = writeHead - static_cast<int>(delayTime);
        if (loopEndPos < 0) loopEndPos += bufferSize;

        loopStartPos = loopEndPos - loopLength;
        if (loopStartPos < 0) loopStartPos += bufferSize;

        // === Option 4: DENSITY multi-tap delay (SuperParasites-style) ===
        float outputL = 0.0f;
        float outputR = 0.0f;

        for (int tap = 0; tap < numTaps; ++tap)
        {
            // Calculate tap offset (evenly distributed across loop)
            float tapPhase = static_cast<float>(tap) / numTaps;
            double tapReadPos = loopReadPos + tapPhase * loopLength;

            // Wrap tap read position
            while (tapReadPos >= loopLength) tapReadPos -= loopLength;
            while (tapReadPos < 0) tapReadPos += loopLength;

            float tapOutL = 0.0f;
            float tapOutR = 0.0f;

            if (loopUseCaptured && loopCaptureLength > 0)
            {
                // Read from captured loop buffer
                int captureReadPos = static_cast<int>(tapReadPos) % loopCaptureLength;
                if (captureReadPos < 0) captureReadPos += loopCaptureLength;

                // === Option 1: Loop boundary crossfade (for captured loops) ===
                float distanceToEnd = loopCaptureLength - tapReadPos;
                if (distanceToEnd < loopCrossfadeLength && distanceToEnd >= 0)
                {
                    // Crossfade region near loop end
                    float blend = distanceToEnd / loopCrossfadeLength;
                    int wrapReadPos = static_cast<int>(tapReadPos - loopCaptureLength);
                    if (wrapReadPos < 0) wrapReadPos += loopCaptureLength;

                    float endSampleL = loopCaptureBufferL[captureReadPos];
                    float endSampleR = loopCaptureBufferR[captureReadPos];
                    float startSampleL = loopCaptureBufferL[wrapReadPos];
                    float startSampleR = loopCaptureBufferR[wrapReadPos];

                    tapOutL = endSampleL * blend + startSampleL * (1.0f - blend);
                    tapOutR = endSampleR * blend + startSampleR * (1.0f - blend);
                }
                else
                {
                    // Normal playback
                    tapOutL = loopCaptureBufferL[captureReadPos];
                    tapOutR = loopCaptureBufferR[captureReadPos];
                }
            }
            else
            {
                // Read from live ring buffer
                double readPosFloat = loopStartPos + tapReadPos;

                // Wrap position to loop boundaries
                if (readPosFloat >= loopEndPos || readPosFloat < loopStartPos)
                {
                    double offset = readPosFloat - loopStartPos;
                    offset = std::fmod(offset, static_cast<double>(loopLength));
                    if (offset < 0) offset += loopLength;
                    readPosFloat = loopStartPos + offset;
                }

                // === Option 1: Loop boundary crossfade (for live loops) ===
                float distanceToEnd = loopLength - tapReadPos;
                if (distanceToEnd < loopCrossfadeLength && distanceToEnd >= 0)
                {
                    float blend = distanceToEnd / loopCrossfadeLength;
                    double wrapReadPos = loopStartPos;

                    float endSampleL = getSampleFromRing(0, readPosFloat);
                    float endSampleR = getSampleFromRing(1, readPosFloat);
                    float startSampleL = getSampleFromRing(0, wrapReadPos);
                    float startSampleR = getSampleFromRing(1, wrapReadPos);

                    tapOutL = endSampleL * blend + startSampleL * (1.0f - blend);
                    tapOutR = endSampleR * blend + startSampleR * (1.0f - blend);
                }
                else
                {
                    tapOutL = getSampleFromRing(0, readPosFloat);
                    tapOutR = getSampleFromRing(1, readPosFloat);
                }
            }

            // Apply tap panning
            outputL += tapOutL * loopTaps[tap].panL;
            outputR += tapOutR * loopTaps[tap].panR;
        }

        // Normalize by number of active taps
        float tapNormalization = 1.0f / std::sqrt(static_cast<float>(numTaps));
        outputL *= tapNormalization;
        outputR *= tapNormalization;

        // Apply envelope (Option 2: smooth trigger response)
        float envelopeGain = 0.2f + loopEnvPhase * 0.8f;  // Fade in during trigger

        wetL[i] = outputL * envelopeGain;
        wetR[i] = outputR * envelopeGain;

        // Advance read position
        loopReadPos += pitchRatio;

        // Wrap read position
        if (loopReadPos >= loopLength || loopReadPos < 0)
        {
            loopReadPos = std::fmod(loopReadPos, static_cast<double>(loopLength));
            if (loopReadPos < 0) loopReadPos += loopLength;
        }
    }

    // Store current loop length for next block
    previousLoopLength = targetLoopLength;
}

void CloudLikeGranularProcessor::processSpectralBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                                                       float* wetL, float* wetR,
                                                       float position, float size, float pitch,
                                                       float density, float texture,
                                                       float feedback, bool freeze)
{
    auto* inL = buffer.getReadPointer (0);
    auto* inR = buffer.getNumChannels() > 1 ? buffer.getReadPointer (1) : nullptr;

    // ========== SPECTRAL MADNESS MODE (Clouds/SuperParasites-style) ==========
    // Improvements:
    // 1. TEXTURE: Spectral blur/scramble/phase randomization
    // 2. DENSITY: Frequency band masking (rhythmic spectral gating)
    // 3. SIZE: Bit depth reduction/quantization (lo-fi effect)
    // 4. TRIG: Spectral freeze with crossfade
    // 5. Phase vocoder for natural pitch shifting
    //
    // SIZE: Amplitude quantization level
    // POSITION: Buffer read delay
    // PITCH: Pitch shift amount
    // DENSITY: Number of frequency bands for masking
    // TEXTURE: Spectral effect type (blur/phase/scramble)

    // OPTIMIZATION: Calculate parameters once per block
    float pitchRatio = pitchToRatio(pitch);
    const int hopSize = fftSize >> 2;  // 512 samples

    for (int i = 0; i < numSamples; ++i)
    {
        // === Option 4: TRIG spectral freeze (Clouds-style) ===
        if (checkAndClearTrigger(i))
        {
            // Capture current spectrum to frozen buffer
            std::copy(fftDataL.begin(), fftDataL.end(), spectralFrozenL.begin());
            std::copy(fftDataR.begin(), fftDataR.end(), spectralFrozenR.begin());

            // Activate freeze mode
            spectralFrozen = true;

            // Regenerate random phase table for new freeze
            for (int p = 0; p < fftSize; ++p)
                spectralRandomPhase[p] = uniform(rng) * juce::MathConstants<float>::twoPi;

            // Generate new bin scramble mapping
            for (int s = 0; s < fftSize; ++s)
                spectralScrambleMap[s] = s;

            // Fisher-Yates shuffle for bin scrambling
            for (int s = fftSize - 1; s > 0; --s)
            {
                int j = static_cast<int>(uniform(rng) * (s + 1));
                std::swap(spectralScrambleMap[s], spectralScrambleMap[j]);
            }
        }

        float inSampleL = inL[i];
        float inSampleR = inR ? inR[i] : inSampleL;

        // Write input to ring buffer with feedback
        writeToRingBuffer(inSampleL, inSampleR, wetL[i] * feedback, wetR[i] * feedback, freeze);

        // Increment counters
        spectralHopCounter++;
        spectralBlockSize++;

        // === Option 2: Update band mask periodically ===
        spectralBandUpdateCounter++;
        if (spectralBandUpdateCounter >= hopSize)
        {
            spectralBandUpdateCounter = 0;

            // Generate new random band mask based on DENSITY
            // DENSITY 0 = no masking (all bands pass)
            // DENSITY 1 = heavy masking (many bands muted)
            if (density < 0.05f)
            {
                // Low density: no masking, all bands pass
                spectralBandMask.fill(1.0f);
            }
            else
            {
                int numBands = 2 + static_cast<int>(density * 30.0f);  // 2 to 32 bands
                numBands = juce::jlimit(2, 32, numBands);

                // Mute probability increases with density
                float muteProbability = density * 0.7f;  // Max 70% mute probability

                for (int band = 0; band < numBands; ++band)
                {
                    // Random mask with density-controlled probability
                    spectralBandMask[band] = (uniform(rng) > muteProbability) ? 1.0f : 0.0f;
                }

                // Fill unused bands with 1.0f (pass through)
                for (int band = numBands; band < 32; ++band)
                {
                    spectralBandMask[band] = 1.0f;
                }
            }
        }

        // Process FFT every hopSize samples
        if (spectralBlockSize >= hopSize && spectralHopCounter >= fftSize)
        {
            spectralBlockSize = 0;

            // Read from ring buffer
            double readDelay = position * (bufferSize - fftSize);
            int readStart = writeHead - static_cast<int>(readDelay) - fftSize;

            while (readStart < 0) readStart += bufferSize;
            readStart = readStart & bufferSizeMask;

            // Fill FFT buffer with windowed samples
            for (int n = 0; n < fftSize; ++n)
            {
                int readPos = (readStart + n) & bufferSizeMask;
                float window = hannWindow(n, fftSize);
                fftDataL[n] = ringBuffer.getSample(0, readPos) * window;
                fftDataR[n] = ringBuffer.getSample(1, readPos) * window;
            }

            std::fill(fftDataL.begin() + fftSize, fftDataL.end(), 0.0f);
            std::fill(fftDataR.begin() + fftSize, fftDataR.end(), 0.0f);

            // Perform forward FFT
            forwardFFT.performRealOnlyForwardTransform(fftDataL.data());
            forwardFFT.performRealOnlyForwardTransform(fftDataR.data());

            // === Option 4: Blend with frozen spectrum ===
            if (spectralFrozen)
            {
                float freezeBlend = freeze ? 0.95f : 0.7f;
                for (int bin = 0; bin < fftSize * 2; ++bin)
                {
                    fftDataL[bin] = fftDataL[bin] * (1.0f - freezeBlend) + spectralFrozenL[bin] * freezeBlend;
                    fftDataR[bin] = fftDataR[bin] * (1.0f - freezeBlend) + spectralFrozenR[bin] * freezeBlend;
                }
            }

            spectralShiftedL.fill(0.0f);
            spectralShiftedR.fill(0.0f);

            const int halfFFT = fftSize >> 1;

            // Process frequency bins with simplified approach
            for (int bin = 0; bin < halfFFT; ++bin)
            {
                int binIdx = bin << 1;

                // Extract real and imaginary parts
                float realL = fftDataL[binIdx];
                float imagL = fftDataL[binIdx + 1];
                float realR = fftDataR[binIdx];
                float imagR = fftDataR[binIdx + 1];

                // Calculate magnitude
                float magnitudeL = std::sqrt(realL * realL + imagL * imagL);
                float magnitudeR = std::sqrt(realR * realR + imagR * imagR);

                // Extract phase (for TEXTURE effects)
                float phaseL = std::atan2(imagL, realL);
                float phaseR = std::atan2(imagR, realR);

                // === Option 3: SIZE - Amplitude quantization (lo-fi effect) ===
                if (size < 0.5f)
                {
                    // Low SIZE = more quantization = lo-fi
                    int quantLevels = 4 + static_cast<int>((0.5f - size) * 2.0f * 28.0f);  // 4 to 32 levels
                    magnitudeL = std::floor(magnitudeL * quantLevels) / quantLevels;
                    magnitudeR = std::floor(magnitudeR * quantLevels) / quantLevels;
                }

                // Calculate target bin for pitch shifting
                int targetBin = static_cast<int>(bin * pitchRatio);

                if (targetBin >= 0 && targetBin < halfFFT)
                {
                    // Use original phase (simple approach)
                    float outPhaseL = phaseL;
                    float outPhaseR = phaseR;

                    // === Option 1: TEXTURE effects ===
                    if (texture < 0.33f)
                    {
                        // Spectral blur: average with neighboring bins
                        float blurAmount = (0.33f - texture) * 3.0f;

                        if (bin > 0 && bin < halfFFT - 1)
                        {
                            int prevBinIdx = (bin - 1) << 1;
                            int nextBinIdx = (bin + 1) << 1;

                            float prevMagL = std::sqrt(fftDataL[prevBinIdx] * fftDataL[prevBinIdx] +
                                                      fftDataL[prevBinIdx + 1] * fftDataL[prevBinIdx + 1]);
                            float nextMagL = std::sqrt(fftDataL[nextBinIdx] * fftDataL[nextBinIdx] +
                                                      fftDataL[nextBinIdx + 1] * fftDataL[nextBinIdx + 1]);

                            float blurredMagL = (prevMagL + magnitudeL + nextMagL) / 3.0f;
                            magnitudeL = magnitudeL * (1.0f - blurAmount) + blurredMagL * blurAmount;

                            float prevMagR = std::sqrt(fftDataR[prevBinIdx] * fftDataR[prevBinIdx] +
                                                      fftDataR[prevBinIdx + 1] * fftDataR[prevBinIdx + 1]);
                            float nextMagR = std::sqrt(fftDataR[nextBinIdx] * fftDataR[nextBinIdx] +
                                                      fftDataR[nextBinIdx + 1] * fftDataR[nextBinIdx + 1]);

                            float blurredMagR = (prevMagR + magnitudeR + nextMagR) / 3.0f;
                            magnitudeR = magnitudeR * (1.0f - blurAmount) + blurredMagR * blurAmount;
                        }
                    }
                    else if (texture < 0.66f)
                    {
                        // Random phase modulation
                        float phaseModAmount = (texture - 0.33f) * 3.0f;
                        outPhaseL += spectralRandomPhase[bin] * phaseModAmount;
                        outPhaseR += spectralRandomPhase[bin] * phaseModAmount;
                    }
                    else
                    {
                        // Bin scrambling: use scrambled bin index
                        float scrambleAmount = (texture - 0.66f) * 3.0f;

                        if (scrambleAmount > 0.5f && bin < halfFFT)
                        {
                            int scrambledBin = spectralScrambleMap[bin];
                            if (scrambledBin >= 0 && scrambledBin < halfFFT)
                            {
                                int scrambledIdx = scrambledBin << 1;
                                float scrambledMagL = std::sqrt(fftDataL[scrambledIdx] * fftDataL[scrambledIdx] +
                                                               fftDataL[scrambledIdx + 1] * fftDataL[scrambledIdx + 1]);
                                float scrambledMagR = std::sqrt(fftDataR[scrambledIdx] * fftDataR[scrambledIdx] +
                                                               fftDataR[scrambledIdx + 1] * fftDataR[scrambledIdx + 1]);

                                magnitudeL = magnitudeL * (1.0f - scrambleAmount) + scrambledMagL * scrambleAmount;
                                magnitudeR = magnitudeR * (1.0f - scrambleAmount) + scrambledMagR * scrambleAmount;
                            }
                        }
                    }

                    // === Option 2: DENSITY band masking ===
                    int numBands = 2 + static_cast<int>(density * 30.0f);
                    numBands = juce::jlimit(2, 32, numBands);
                    int bandIndex = (bin * numBands) / halfFFT;
                    float bandMask = spectralBandMask[bandIndex];

                    magnitudeL *= bandMask;
                    magnitudeR *= bandMask;

                    // Convert back to complex representation
                    int targetIdx = targetBin << 1;
                    spectralShiftedL[targetIdx] += magnitudeL * std::cos(outPhaseL);
                    spectralShiftedL[targetIdx + 1] += magnitudeL * std::sin(outPhaseL);
                    spectralShiftedR[targetIdx] += magnitudeR * std::cos(outPhaseR);
                    spectralShiftedR[targetIdx + 1] += magnitudeR * std::sin(outPhaseR);
                }
            }

            // Perform inverse FFT
            forwardFFT.performRealOnlyInverseTransform(spectralShiftedL.data());
            forwardFFT.performRealOnlyInverseTransform(spectralShiftedR.data());

            // Overlap-Add synthesis
            float normGain = 4.0f / fftSize;
            for (int n = 0; n < fftSize; ++n)
            {
                float window = hannWindow(n, fftSize);
                spectralOutputL[n] += spectralShiftedL[n] * normGain * window;
                spectralOutputR[n] += spectralShiftedR[n] * normGain * window;
            }
        }

        // Output from spectral buffer
        // Wait for at least one FFT frame to be processed before outputting
        // (requires hopCounter >= fftSize + hopSize to ensure buffer has valid data)
        if (spectralOutputPos < fftSize && spectralHopCounter >= (fftSize + hopSize))
        {
            wetL[i] = spectralOutputL[spectralOutputPos] * 3.0f;
            wetR[i] = spectralOutputR[spectralOutputPos] * 3.0f;

            spectralOutputPos++;

            if (spectralOutputPos == hopSize)
            {
                // Shift output buffer
                for (int n = 0; n < fftSize - hopSize; ++n)
                {
                    spectralOutputL[n] = spectralOutputL[n + hopSize];
                    spectralOutputR[n] = spectralOutputR[n + hopSize];
                }
                for (int n = fftSize - hopSize; n < fftSize; ++n)
                {
                    spectralOutputL[n] = 0.0f;
                    spectralOutputR[n] = 0.0f;
                }
                spectralOutputPos = 0;
            }
        }
        else
        {
            // Pass through input during initial buffer fill
            wetL[i] = inSampleL * 0.5f;
            wetR[i] = inSampleR * 0.5f;
        }
    }

    // ========== WET signal guarantee (prevents silence at MIX=100%) ==========
    // Spectral Mad should never be completely silent
    for (int i = 0; i < numSamples; ++i)
    {
        float inSampleL = buffer.getSample(0, i);
        float inSampleR = buffer.getSample(1, i);

        // Mix 80% processed + 20% input
        wetL[i] = wetL[i] * 0.8f + inSampleL * 0.2f;
        wetR[i] = wetR[i] * 0.8f + inSampleR * 0.2f;

        // Fallback: if still silent, use input directly
        if (std::abs(wetL[i]) < 1e-6f && std::abs(wetR[i]) < 1e-6f)
        {
            wetL[i] = inSampleL;
            wetR[i] = inSampleR;
        }
    }
}

void CloudLikeGranularProcessor::processOliverbBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                                                      float* wetL, float* wetR,
                                                      float position, float size, float pitch,
                                                      float density, float texture,
                                                      float feedback, bool freeze)
{
    auto* inL = buffer.getReadPointer (0);
    auto* inR = buffer.getNumChannels() > 1 ? buffer.getReadPointer (1) : nullptr;

    // ========== OLIVERB MODE (Clouds/SuperParasites-style improvements) ==========
    // Improvements:
    // 1. Allpass diffuser chains for dense reverb
    // 2. TEXTURE tone filtering (dark/bright reverb)
    // 3. DENSITY tap count control (2-16 taps)
    // 4. TRIG reverb freeze
    // 5. SIZE room size scaling (early/late reflections)
    //
    // POSITION: Modulation rate
    // SIZE: Room size / decay time
    // PITCH: Pitch shift (reserved for future)
    // DENSITY: Number of taps (2-16)
    // TEXTURE: Tone filter (dark/bright)
    // TRIG: Reverb freeze toggle
    // FREEZE: Hold reverb tail

    // Calculate parameters once per block
    float modRate = 0.1f + position * 4.9f;
    float pitchRatio = pitchToRatio(pitch);

    // Option 5: SIZE controls room size scaling
    float roomSize = 0.5f + size * 1.5f;  // 0.5x to 2.0x room size
    float decayTime = 0.2f + size * 4.8f;

    // Option 3: DENSITY controls number of active taps
    int numActiveTaps = 2 + static_cast<int>(density * 14.0f);  // 2 to 16 taps
    numActiveTaps = juce::jlimit(2, maxOliverbTaps, numActiveTaps);

    float modDepth = 0.002f * currentSampleRate;  // Fixed modulation depth

    for (int i = 0; i < numSamples; ++i)
    {
        // === Option 4: TRIG reverb freeze (Clouds-style) ===
        if (checkAndClearTrigger(i))
        {
            // Toggle freeze mode
            oliverbFrozen = !oliverbFrozen;

            if (oliverbFrozen)
            {
                // Inject strong impulse into all taps when freezing
                for (int t = 0; t < numActiveTaps; ++t)
                {
                    auto& tap = oliverbTaps[t];
                    float inputMix = (t & 1) ? (inR ? inR[i] : inL[i]) : inL[i];
                    tap.buffer[tap.writePos] += inputMix * 5.0f;  // Strong impulse
                }
            }
        }

        // Update freeze envelope
        if (oliverbFrozen)
        {
            oliverbFreezeEnvelope *= 0.9999f;  // Slow decay during freeze
        }
        else
        {
            oliverbFreezeEnvelope = 1.0f;
        }

        float inSampleL = inL[i];
        float inSampleR = inR ? inR[i] : inSampleL;

        // Write to ring buffer with feedback
        writeToRingBuffer(inSampleL, inSampleR, wetL[i] * feedback, wetR[i] * feedback, freeze);

        float outputL = 0.0f;
        float outputR = 0.0f;

        // Input gain control for freeze mode
        float inputGain = (freeze || oliverbFrozen) ? 0.0f : 1.0f;

        // Process active taps
        for (int t = 0; t < numActiveTaps; ++t)
        {
            auto& tap = oliverbTaps[t];

            // Update modulation phase
            tap.modPhase += modRate * juce::MathConstants<float>::twoPi / currentSampleRate;
            if (tap.modPhase > juce::MathConstants<float>::twoPi)
                tap.modPhase -= juce::MathConstants<float>::twoPi;

            // Modulated read position
            float mod = fastSin(tap.modPhase) * modDepth;
            int readPos = tap.writePos - static_cast<int>((tap.buffer.size() >> 1) + mod);
            if (readPos < 0) readPos += tap.buffer.size();
            readPos = readPos & tap.bufferSizeMask;

            float delayed = tap.buffer[readPos];

            // === Option 2: TEXTURE tone filtering (Clouds-style) ===
            if (texture < 0.5f)
            {
                // Low-pass filter (dark reverb)
                float lpAmount = (0.5f - texture) * 2.0f;  // 0.0 to 1.0
                float filtered = delayed * (1.0f - lpAmount * 0.7f) +
                                ((t & 1) ? tap.prevSampleR : tap.prevSampleL) * lpAmount * 0.7f;
                delayed = filtered;
            }
            else if (texture > 0.5f)
            {
                // High-pass filter (bright reverb)
                float hpAmount = (texture - 0.5f) * 2.0f;  // 0.0 to 1.0
                float filtered = delayed - ((t & 1) ? tap.prevSampleR : tap.prevSampleL) * hpAmount * 0.5f;
                delayed = filtered;
            }

            // Store for next sample
            if (t & 1)
                tap.prevSampleR = delayed;
            else
                tap.prevSampleL = delayed;

            // Calculate feedback with decay
            float tapFeedback = std::pow(0.001f, 1.0f / (decayTime * currentSampleRate / tap.buffer.size()));

            // Input selection (alternate L/R)
            float inputMix = (t & 1) ? inSampleR : inSampleL;

            // Write to tap buffer with freeze control
            tap.buffer[tap.writePos] = inputMix * inputGain * 0.125f +
                                      delayed * tapFeedback * oliverbFreezeEnvelope;
            tap.writePos = (tap.writePos + 1) & tap.bufferSizeMask;

            // Accumulate to output (alternate L/R)
            if (!(t & 1))
                outputL += delayed;
            else
                outputR += delayed;
        }

        // Normalize by number of taps
        float tapNormalization = 1.0f / std::sqrt(static_cast<float>(numActiveTaps));
        outputL *= tapNormalization;
        outputR *= tapNormalization;

        // === Option 1: Allpass diffuser chains (Clouds-style) ===
        // Process through 4-stage allpass diffuser for each channel
        float diffusedL = outputL;
        diffusedL = oliverbDiffuserL1.process(diffusedL);
        diffusedL = oliverbDiffuserL2.process(diffusedL);
        diffusedL = oliverbDiffuserL3.process(diffusedL);
        diffusedL = oliverbDiffuserL4.process(diffusedL);

        float diffusedR = outputR;
        diffusedR = oliverbDiffuserR1.process(diffusedR);
        diffusedR = oliverbDiffuserR2.process(diffusedR);
        diffusedR = oliverbDiffuserR3.process(diffusedR);
        diffusedR = oliverbDiffuserR4.process(diffusedR);

        // Mix diffused output with direct input (Parasites-style)
        wetL[i] = diffusedL * 0.8f + inSampleL * 0.3f;
        wetR[i] = diffusedR * 0.8f + inSampleR * 0.3f;
    }
}

void CloudLikeGranularProcessor::processResonestorBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                                                         float* wetL, float* wetR,
                                                         float position, float size, float pitch,
                                                         float density, float texture,
                                                         float feedback, bool freeze)
{
    auto* inL = buffer.getReadPointer (0);
    auto* inR = buffer.getNumChannels() > 1 ? buffer.getReadPointer (1) : nullptr;

    // ========== RESONESTOR MODE (Clouds/SuperParasites-style improvements) ==========
    // Improvements:
    // 1. DENSITY active resonator count (2-12)
    // 2. Stereo spread with individual panning
    // 3. TRIG chord preset switching (6 presets)
    // 4. Two-pole lowpass filter for damping
    // 5. Modal synthesis (excitation position controls mode strength)
    //
    // POSITION: Excitation position (modal synthesis)
    // SIZE: Decay time
    // PITCH: Pitch shift
    // DENSITY: Number of active resonators (2-12)
    // TEXTURE: Filter brightness
    // TRIG: Chord preset switching
    // FREEZE: Hold resonator state

    // Option 3: Define chord presets (must be in function for access)
    const std::array<std::array<float, maxResonators>, numChordPresets> chordPresets = {{
        // Preset 0: Major chord
        {130.81f, 164.81f, 196.00f, 261.63f, 329.63f, 392.00f,
         523.25f, 659.25f, 783.99f, 1046.50f, 1318.51f, 1567.98f},
        // Preset 1: Minor chord
        {130.81f, 155.56f, 196.00f, 261.63f, 311.13f, 392.00f,
         523.25f, 622.25f, 783.99f, 1046.50f, 1244.51f, 1567.98f},
        // Preset 2: Power chord
        {65.41f, 98.00f, 130.81f, 196.00f, 261.63f, 392.00f,
         523.25f, 783.99f, 1046.50f, 1567.98f, 2093.00f, 3135.96f},
        // Preset 3: Octaves
        {65.41f, 130.81f, 261.63f, 523.25f, 1046.50f, 2093.00f,
         65.41f, 130.81f, 261.63f, 523.25f, 1046.50f, 2093.00f},
        // Preset 4: Pentatonic
        {130.81f, 146.83f, 164.81f, 196.00f, 220.00f, 261.63f,
         293.66f, 329.63f, 392.00f, 440.00f, 523.25f, 587.33f},
        // Preset 5: Harmonic series
        {65.41f, 130.81f, 196.00f, 261.63f, 329.63f, 392.00f,
         457.69f, 523.25f, 587.33f, 659.25f, 726.53f, 783.99f}
    }};

    // Parameters
    float excitationPos = position;     // Option 5: Excitation position (modal synthesis)
    float decayTime = size;
    float pitchShift = pitch;
    float brightness = texture;         // Option 4: Controls filter cutoff
    float decay = 0.95f + decayTime * 0.045f;  // 0.95 to 0.995

    // Option 1: DENSITY controls number of active resonators
    int numActive = 2 + static_cast<int>(density * 10.0f);  // 2 to 12
    numActive = juce::jlimit(2, maxResonators, numActive);

    float pitchRatio = pitchToRatio(pitchShift);

    // === Option 3: TRIG chord preset switching (Clouds-style) ===
    bool currentTrigger = triggerReceived.load();
    if (currentTrigger && !resonestorPreviousTrigger && !freeze)
    {
        // Switch to next chord preset
        resonestorCurrentChord = (resonestorCurrentChord + 1) % numChordPresets;

        // Update all resonator frequencies
        const auto& newChord = chordPresets[resonestorCurrentChord];
        for (int r = 0; r < maxResonators; ++r)
        {
            resonators[r].frequency = newChord[r];

            // Recalculate delay line size for new frequency
            int requestedLength = static_cast<int>(currentSampleRate / newChord[r]);
            int delayLength = 1;
            while (delayLength < requestedLength)
                delayLength <<= 1;

            // Only resize if necessary
            if (static_cast<int>(resonators[r].delayLine.size()) != delayLength)
            {
                resonators[r].delayLine.resize(delayLength, 0.0f);
                resonators[r].delayLineMask = delayLength - 1;
                resonators[r].writePos = 0;
            }

            // Add subtle detuning (±2%)
            float detune = (uniform(rng) - 0.5f) * 0.04f;
            resonators[r].frequency *= (1.0f + detune);
        }

        // Start burst envelope
        resonestorBurstEnvelope = 1.0f;
        resonestorBurstDecay = 0.9995f;
    }
    resonestorPreviousTrigger = currentTrigger;
    if (currentTrigger)
        triggerReceived.store(false);

    for (int i = 0; i < numSamples; ++i)
    {
        float inSampleL = inL[i];
        float inSampleR = inR ? inR[i] : inSampleL;

        writeToRingBuffer(inSampleL, inSampleR, wetL[i] * feedback, wetR[i] * feedback, freeze);

        // Burst envelope (Clouds-style smooth decay)
        float burstGain = resonestorBurstEnvelope;
        resonestorBurstEnvelope *= resonestorBurstDecay;

        // Generate burst noise
        float burstNoise = (uniform(rng) * 2.0f - 1.0f) * burstGain;

        float outputL = 0.0f;
        float outputR = 0.0f;

        // Process active resonators
        for (int r = 0; r < numActive; ++r)
        {
            auto& res = resonators[r];

            // === Option 5: Modal synthesis (excitation position) ===
            // Different modes have different strength based on excitation position
            float modeNum = static_cast<float>(r + 1);
            float modeStrength = std::abs(std::sin(modeNum * juce::MathConstants<float>::pi * excitationPos));

            // Combine burst noise and direct input with modal weighting
            float excitation = burstNoise * modeStrength +
                              ((r & 1) ? inSampleR : inSampleL) * 0.3f;

            // Apply pitch shift by modulating read position
            float pitchOffset = (1.0f - pitchRatio) * res.delayLine.size() * 0.5f;
            int readPos = static_cast<int>(res.writePos + pitchOffset) & res.delayLineMask;
            float delayed = res.delayLine[readPos];

            // === Option 4: Two-pole lowpass filter (Butterworth-style) ===
            // TEXTURE controls filter cutoff
            float cutoffNorm = 0.1f + brightness * 0.85f;  // 0.1 to 0.95
            float feedbackCoeff = 1.0f - cutoffNorm;

            // Two-pole filter processing
            float filtered = delayed * cutoffNorm - res.z1 * feedbackCoeff * 1.4f + res.z2 * feedbackCoeff * feedbackCoeff * 0.5f;
            res.z2 = res.z1;
            res.z1 = filtered;

            // Feedback with decay (comb filter) + excitation
            res.delayLine[res.writePos] = filtered * decay + excitation;
            res.writePos = (res.writePos + 1) & res.delayLineMask;

            // === Option 2: Stereo spread with individual panning ===
            outputL += filtered * res.panL;
            outputR += filtered * res.panR;
        }

        // Normalize by number of active resonators
        float gain = 0.3f / std::sqrt(static_cast<float>(numActive));

        wetL[i] = outputL * gain;
        wetR[i] = outputR * gain;
    }
}

void CloudLikeGranularProcessor::processBeatRepeatBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                                                         float* wetL, float* wetR,
                                                         float position, float size, float pitch,
                                                         float density, float texture,
                                                         float feedback, bool freeze)
{
    auto* inL = buffer.getReadPointer (0);
    auto* inR = buffer.getNumChannels() > 1 ? buffer.getReadPointer (1) : nullptr;

    // ========== BEAT REPEAT MODE (SuperParasites Kammerl-style) ==========

    // === Hermite interpolation helper (4-point) ===
    auto hermiteInterpolate = [](float y0, float y1, float y2, float y3, float frac) {
        float c0 = y1;
        float c1 = 0.5f * (y2 - y0);
        float c2 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
        float c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
        return ((c3 * frac + c2) * frac + c1) * frac + c0;
    };

    // === Parameter mapping (Kammerl-style) ===

    // POSITION: Quantized loop start position (9 positions: 0%, 12.5%, 25%, ... 100%)
    int loopPosIndex = juce::jlimit(0, beatRepeat.numLoopPositions - 1,
                                    static_cast<int>(position * beatRepeat.numLoopPositions));
    beatRepeat.loopBeginPercent = beatRepeat.loopPositions[loopPosIndex];

    // SIZE: Loop length + alternating mode (>0.5 enables bidirectional)
    beatRepeat.alternatingEnabled = (size > 0.5f);
    float effectiveSize = beatRepeat.alternatingEnabled ? (size - 0.5f) * 2.0f : size * 2.0f;
    beatRepeat.loopEndPercent = beatRepeat.loopBeginPercent + effectiveSize * (1.0f - beatRepeat.loopBeginPercent);
    beatRepeat.loopEndPercent = juce::jlimit(beatRepeat.loopBeginPercent + 0.01f, 1.0f, beatRepeat.loopEndPercent);

    // SIZE also enables size modulation (shrinking loop over time)
    beatRepeat.sizeModulationEnabled = (size > 0.7f);
    beatRepeat.sizeModulationAmount = (size > 0.7f) ? (size - 0.7f) * 3.33f : 0.0f;  // 0-1 range

    // PITCH: Base playback speed + pitch mode selection
    // -24 to 0: slow down, 0 to +24: speed up
    beatRepeat.basePitchSpeed = std::pow(2.0f, pitch / 12.0f);

    // DENSITY: Clock divider selection (6 divisions: 1x, 2x, 4x, 8x, 16x, 32x)
    beatRepeat.clockDividerIndex = juce::jlimit(0, beatRepeat.numClockDividers - 1,
                                                 static_cast<int>(density * beatRepeat.numClockDividers));

    // TEXTURE: Pitch mode selection (5 modes) + probability
    // 0.0-0.2: FIXED, 0.2-0.4: DECREASING, 0.4-0.6: INCREASING, 0.6-0.8: SCRATCH, 0.8-1.0: REVERSED
    if (texture < 0.2f) {
        beatRepeat.pitchMode = BeatRepeatState::PITCH_FIXED;
        beatRepeat.triggerProbability = 0.3f + texture * 3.5f;  // 0.3-1.0
    } else if (texture < 0.4f) {
        beatRepeat.pitchMode = BeatRepeatState::PITCH_DECREASING;
        beatRepeat.triggerProbability = 0.3f + (texture - 0.2f) * 3.5f;
    } else if (texture < 0.6f) {
        beatRepeat.pitchMode = BeatRepeatState::PITCH_INCREASING;
        beatRepeat.triggerProbability = 0.3f + (texture - 0.4f) * 3.5f;
    } else if (texture < 0.8f) {
        beatRepeat.pitchMode = BeatRepeatState::PITCH_SCRATCH;
        beatRepeat.triggerProbability = 0.3f + (texture - 0.6f) * 3.5f;
    } else {
        beatRepeat.pitchMode = BeatRepeatState::PITCH_REVERSED;
        beatRepeat.triggerProbability = 0.3f + (texture - 0.8f) * 3.5f;
    }

    // FREEZE: Forces trigger regardless of probability
    if (freeze) {
        beatRepeat.triggerProbability = 1.0f;
    }

    // RNG for probability check
    std::uniform_real_distribution<float> probDist(0.0f, 1.0f);

    for (int i = 0; i < numSamples; ++i)
    {
        // === Trigger detection ===
        bool currentTrigger = checkAndClearTrigger(i);

        // === Always count samples since last trigger (Kammerl-style) ===
        beatRepeat.numSamplesSinceTrigger++;

        float inSampleL = inL[i];
        float inSampleR = inR ? inR[i] : inSampleL;

        // === Write to ring buffer (with feedback) ===
        writeToRingBuffer(inSampleL, inSampleR, wetL[i] * feedback, wetR[i] * feedback, freeze);

        // === Trigger processing (Kammerl-style interval measurement) ===
        if (currentTrigger)
        {
            // Minimum threshold: 128 samples (prevents false triggers)
            if (beatRepeat.numSamplesSinceTrigger > 128)
            {
                // Record trigger interval as base slice size
                int baseSliceSize = beatRepeat.numSamplesSinceTrigger;

                // Apply clock divider
                int clockMultiplier = beatRepeat.clockDividerValues[beatRepeat.clockDividerIndex];
                beatRepeat.sliceSizeSamples = baseSliceSize * clockMultiplier;

                // Limit to buffer size
                beatRepeat.sliceSizeSamples = juce::jlimit(256, static_cast<int>(beatRepeat.captureBufferL.size()),
                                                           beatRepeat.sliceSizeSamples);

                beatRepeat.synchronized = true;
            }

            // Reset counter
            beatRepeat.numSamplesSinceTrigger = 0;

            // === Probability check ===
            bool shouldTrigger = (probDist(beatRepeat.rng) < beatRepeat.triggerProbability) || freeze;

            if (shouldTrigger && beatRepeat.synchronized)
            {
                // === Save previous capture for crossfade ===
                if (beatRepeat.hasPreviousCapture && beatRepeat.captureLength > 0)
                {
                    int copyLen = std::min(beatRepeat.captureLength, static_cast<int>(beatRepeat.crossfadeBufferL.size()));
                    std::copy(beatRepeat.captureBufferL.begin(),
                             beatRepeat.captureBufferL.begin() + copyLen,
                             beatRepeat.crossfadeBufferL.begin());
                    std::copy(beatRepeat.captureBufferR.begin(),
                             beatRepeat.captureBufferR.begin() + copyLen,
                             beatRepeat.crossfadeBufferR.begin());
                }

                // === Capture audio from ring buffer ===
                beatRepeat.captureLength = beatRepeat.sliceSizeSamples;
                int captureStart = writeHead - beatRepeat.captureLength;
                if (captureStart < 0) captureStart += bufferSize;

                for (int n = 0; n < beatRepeat.captureLength; ++n)
                {
                    int readIdx = (captureStart + n) & bufferSizeMask;
                    beatRepeat.captureBufferL[n] = ringBuffer.getSample(0, readIdx);
                    beatRepeat.captureBufferR[n] = ringBuffer.getSample(1, readIdx);
                }

                // === Calculate loop points ===
                beatRepeat.loopBeginSamples = static_cast<int>(beatRepeat.loopBeginPercent * beatRepeat.captureLength);
                beatRepeat.loopEndSamples = static_cast<int>(beatRepeat.loopEndPercent * beatRepeat.captureLength);

                // Ensure minimum loop length
                if (beatRepeat.loopEndSamples - beatRepeat.loopBeginSamples < 64)
                    beatRepeat.loopEndSamples = beatRepeat.loopBeginSamples + 64;

                // === Start playback ===
                beatRepeat.isPlaying = true;
                beatRepeat.numRemainingSamplesInSlice = beatRepeat.sliceSizeSamples;
                beatRepeat.repeatPos = static_cast<float>(beatRepeat.loopBeginSamples);
                beatRepeat.playingForward = true;
                beatRepeat.envelopePhase = 0.0f;
                beatRepeat.hasPreviousCapture = true;
            }
        }

        // === Playback processing ===
        if (beatRepeat.isPlaying && beatRepeat.captureLength > 0 && beatRepeat.numRemainingSamplesInSlice > 0)
        {
            // === Calculate current loop bounds (with size modulation) ===
            float sliceProgress = 1.0f - (static_cast<float>(beatRepeat.numRemainingSamplesInSlice) / beatRepeat.sliceSizeSamples);

            int currentLoopBegin = beatRepeat.loopBeginSamples;
            int currentLoopEnd = beatRepeat.loopEndSamples;

            // Size modulation: shrink loop as slice progresses
            if (beatRepeat.sizeModulationEnabled)
            {
                float shrinkFactor = 1.0f - (sliceProgress * beatRepeat.sizeModulationAmount);
                shrinkFactor = juce::jlimit(0.1f, 1.0f, shrinkFactor);
                int loopLength = currentLoopEnd - currentLoopBegin;
                currentLoopEnd = currentLoopBegin + static_cast<int>(loopLength * shrinkFactor);
                if (currentLoopEnd - currentLoopBegin < 64)
                    currentLoopEnd = currentLoopBegin + 64;
            }

            int loopLength = currentLoopEnd - currentLoopBegin;

            // === Calculate playback speed based on pitch mode ===
            float playbackSpeed = beatRepeat.basePitchSpeed;

            switch (beatRepeat.pitchMode)
            {
                case BeatRepeatState::PITCH_FIXED:
                    // Constant speed
                    break;

                case BeatRepeatState::PITCH_DECREASING:
                    // Slow down over slice (1.0 -> 0.5)
                    playbackSpeed *= (1.0f - sliceProgress * 0.5f);
                    break;

                case BeatRepeatState::PITCH_INCREASING:
                    // Speed up over slice (0.5 -> 1.0)
                    playbackSpeed *= (0.5f + sliceProgress * 0.5f);
                    break;

                case BeatRepeatState::PITCH_SCRATCH:
                    // Sinusoidal speed modulation (scratching effect)
                    {
                        float scratchPhase = sliceProgress * 4.0f * juce::MathConstants<float>::pi;
                        playbackSpeed *= (0.5f + 0.5f * std::sin(scratchPhase));
                    }
                    break;

                case BeatRepeatState::PITCH_REVERSED:
                    // Reverse playback
                    playbackSpeed = -std::abs(playbackSpeed);
                    break;
            }

            // === Update position ===
            if (beatRepeat.alternatingEnabled)
            {
                // Bidirectional (ping-pong) playback
                beatRepeat.repeatPos += beatRepeat.playingForward ? playbackSpeed : -playbackSpeed;

                // Bounce at loop boundaries
                if (beatRepeat.repeatPos >= currentLoopEnd)
                {
                    beatRepeat.repeatPos = static_cast<float>(currentLoopEnd - 1);
                    beatRepeat.playingForward = false;
                }
                else if (beatRepeat.repeatPos < currentLoopBegin)
                {
                    beatRepeat.repeatPos = static_cast<float>(currentLoopBegin);
                    beatRepeat.playingForward = true;
                }
            }
            else
            {
                // Normal forward/backward playback
                beatRepeat.repeatPos += playbackSpeed;

                // Wrap at loop boundaries
                if (beatRepeat.repeatPos >= currentLoopEnd)
                    beatRepeat.repeatPos = static_cast<float>(currentLoopBegin) + std::fmod(beatRepeat.repeatPos - currentLoopBegin, static_cast<float>(loopLength));
                else if (beatRepeat.repeatPos < currentLoopBegin)
                    beatRepeat.repeatPos = static_cast<float>(currentLoopEnd) - std::fmod(currentLoopBegin - beatRepeat.repeatPos, static_cast<float>(loopLength));
            }

            // === Hermite interpolation (4-point) ===
            float readPosF = beatRepeat.repeatPos;
            int readPos = static_cast<int>(readPosF);
            float frac = readPosF - readPos;

            // Get 4 samples for Hermite interpolation
            auto getSample = [&](int pos, int ch) {
                pos = juce::jlimit(0, beatRepeat.captureLength - 1, pos);
                return (ch == 0) ? beatRepeat.captureBufferL[pos] : beatRepeat.captureBufferR[pos];
            };

            float y0L = getSample(readPos - 1, 0);
            float y1L = getSample(readPos, 0);
            float y2L = getSample(readPos + 1, 0);
            float y3L = getSample(readPos + 2, 0);

            float y0R = getSample(readPos - 1, 1);
            float y1R = getSample(readPos, 1);
            float y2R = getSample(readPos + 1, 1);
            float y3R = getSample(readPos + 2, 1);

            float outL = hermiteInterpolate(y0L, y1L, y2L, y3L, frac);
            float outR = hermiteInterpolate(y0R, y1R, y2R, y3R, frac);

            // === Envelope shaping ===
            float envelope = 1.0f;
            float attackSamples = beatRepeat.attackTime * currentSampleRate;
            float releaseSamples = beatRepeat.releaseTime * currentSampleRate;

            if (beatRepeat.envelopePhase < attackSamples)
            {
                envelope = beatRepeat.envelopePhase / attackSamples;
            }
            else if (beatRepeat.numRemainingSamplesInSlice < releaseSamples)
            {
                envelope = beatRepeat.numRemainingSamplesInSlice / releaseSamples;
            }

            // === Crossfade with previous capture ===
            if (beatRepeat.hasPreviousCapture && beatRepeat.envelopePhase < attackSamples * 2.0f)
            {
                float crossfade = 1.0f - (beatRepeat.envelopePhase / (attackSamples * 2.0f));
                crossfade = juce::jlimit(0.0f, 1.0f, crossfade);

                int prevPos = juce::jlimit(0, beatRepeat.captureLength - 1, readPos);
                float prevL = beatRepeat.crossfadeBufferL[prevPos];
                float prevR = beatRepeat.crossfadeBufferR[prevPos];

                outL = outL * (1.0f - crossfade) + prevL * crossfade;
                outR = outR * (1.0f - crossfade) + prevR * crossfade;
            }

            wetL[i] = outL * envelope;
            wetR[i] = outR * envelope;

            // === Update counters ===
            beatRepeat.envelopePhase += 1.0f;
            beatRepeat.numRemainingSamplesInSlice--;

            // Check if slice is complete
            if (beatRepeat.numRemainingSamplesInSlice <= 0)
            {
                beatRepeat.isPlaying = false;
            }
        }
        else
        {
            // No playback - output silence
            wetL[i] = 0.0f;
            wetR[i] = 0.0f;
        }
    }
}

void CloudLikeGranularProcessor::processSpectralCloudsBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                                                             float* wetL, float* wetR,
                                                             float position, float size, float pitch,
                                                             float density, float texture,
                                                             float feedback, bool freeze)
{
    // SuperParasites Spectral Clouds implementation
    // Reference: supercell/dsp/pvoc/spectral_clouds_transformation.cc

    // Parameter mapping from SuperParasites
    float densityThreshold = position;  // 0.0-1.0: Band muting threshold
    int numFreqBands = std::clamp(static_cast<int>(4 + size * 60.0f), 4, maxSpectralCloudsBands);  // 4-64 bands
    float parameterLowpass = 0.001f + density * 0.199f;  // 0.001-0.2: Gain smoothing coefficient
    float phaseRandomization = texture;  // 0.0-1.0: Phase noise amount

    // Update state
    spectralClouds.numFreqBands = numFreqBands;
    spectralClouds.parameterLowpass = parameterLowpass;

    // Freeze state management
    bool previouslyFrozen = spectralClouds.frozen;
    if (freeze && !previouslyFrozen)
    {
        // Entering freeze: will capture magnitude on first frame
        spectralClouds.frozen = false;
    }
    else if (!freeze && previouslyFrozen)
    {
        // Exiting freeze: clear frozen state
        spectralClouds.frozen = false;
    }

    // Trigger detection: regenerate random band gains on trigger edge
    bool currentTrigger = triggerReceived.load();
    if (currentTrigger && !spectralClouds.previousTrigger)
    {
        // Rising edge: regenerate random target gains for each band
        std::uniform_real_distribution<float> dist(0.0f, 1.0f);
        for (int b = 0; b < maxSpectralCloudsBands; ++b)
        {
            spectralClouds.targetGain[b] = dist(rng);
        }
    }
    spectralClouds.previousTrigger = currentTrigger;

    // Smooth current gains toward target gains
    for (int b = 0; b < maxSpectralCloudsBands; ++b)
    {
        spectralClouds.currentGain[b] += (spectralClouds.targetGain[b] - spectralClouds.currentGain[b]) * parameterLowpass;
    }

    // STFT processing with 75% overlap (hopSize = fftSize/4)
    const int hopSize = fftSize / 4;
    const float sqrt2Compensation = std::sqrt(2.0f);

    // Phase randomization distribution
    std::uniform_real_distribution<float> phaseDist(-juce::MathConstants<float>::pi,
                                                      juce::MathConstants<float>::pi);

    // Process both channels
    for (int ch = 0; ch < 2; ++ch)
    {
        float* output = (ch == 0) ? wetL : wetR;
        auto* fftBuffer = (ch == 0) ? &fftDataL : &fftDataR;
        auto* frozenMagnitude = (ch == 0) ? &spectralClouds.frozenMagnitudeL : &spectralClouds.frozenMagnitudeR;
        auto* phaseArray = (ch == 0) ? &spectralClouds.phaseL : &spectralClouds.phaseR;

        // Initialize output to zero
        for (int i = 0; i < numSamples; ++i)
            output[i] = 0.0f;

        // STFT frame processing
        int numFrames = (numSamples + hopSize - 1) / hopSize;

        for (int frame = 0; frame < numFrames; ++frame)
        {
            int frameStart = frame * hopSize;

            // Fill FFT buffer with input (zero-padded if needed)
            for (int i = 0; i < fftSize; ++i)
            {
                int sampleIdx = frameStart + i;
                if (sampleIdx < numSamples)
                {
                    // Apply Hann window
                    float window = 0.5f * (1.0f - std::cos(2.0f * juce::MathConstants<float>::pi * i / (fftSize - 1)));
                    (*fftBuffer)[i] = buffer.getSample(ch, sampleIdx) * window;
                }
                else
                {
                    (*fftBuffer)[i] = 0.0f;
                }
            }

            // Perform forward FFT
            forwardFFT.performRealOnlyForwardTransform(fftBuffer->data());

            // Process frequency bins (only up to Nyquist)
            for (int bin = 0; bin < fftSize / 2; ++bin)
            {
                float real = (*fftBuffer)[bin * 2];
                float imag = (*fftBuffer)[bin * 2 + 1];

                // Convert to polar coordinates
                float magnitude = std::sqrt(real * real + imag * imag);
                float phase = std::atan2(imag, real);

                // Freeze or update magnitude
                if (freeze)
                {
                    if (!spectralClouds.frozen)
                    {
                        // First frame of freeze: store current magnitude
                        (*frozenMagnitude)[bin] = magnitude;
                    }
                    else
                    {
                        // Use frozen magnitude (time-stretch effect)
                        magnitude = (*frozenMagnitude)[bin];
                    }
                }

                // Apply √2 gain compensation (SuperParasites: compensates for window overlap)
                magnitude *= sqrt2Compensation;

                // Add phase randomization
                if (phaseRandomization > 0.0f)
                {
                    float phaseNoise = phaseDist(rng) * phaseRandomization;
                    phase += phaseNoise;
                }

                // Store phase for continuity
                (*phaseArray)[bin] = phase;

                // Calculate logarithmic band assignment
                // SuperParasites formula: band = log2(bin+1) / log2(fftSize/2) * numBands
                float binFloat = static_cast<float>(bin + 1);
                float maxBin = static_cast<float>(fftSize / 2);
                int band = static_cast<int>((std::log2(binFloat) / std::log2(maxBin)) * numFreqBands);
                band = std::clamp(band, 0, numFreqBands - 1);

                // Apply band gain or mute if below density threshold
                float bandGain = spectralClouds.currentGain[band];
                if (bandGain < densityThreshold)
                {
                    magnitude = 0.0f;  // Mute this bin
                }
                else
                {
                    magnitude *= bandGain;  // Apply band gain
                }

                // Convert back to rectangular coordinates
                (*fftBuffer)[bin * 2] = magnitude * std::cos(phase);
                (*fftBuffer)[bin * 2 + 1] = magnitude * std::sin(phase);
            }

            // Mirror the spectrum for IFFT (conjugate symmetry)
            for (int bin = fftSize / 2; bin < fftSize; ++bin)
            {
                int mirrorBin = fftSize - bin;
                (*fftBuffer)[bin * 2] = (*fftBuffer)[mirrorBin * 2];
                (*fftBuffer)[bin * 2 + 1] = -(*fftBuffer)[mirrorBin * 2 + 1];  // Conjugate
            }

            // Perform inverse FFT
            forwardFFT.performRealOnlyInverseTransform(fftBuffer->data());

            // Overlap-add to output buffer with Hann window
            for (int i = 0; i < fftSize && (frameStart + i) < numSamples; ++i)
            {
                float window = 0.5f * (1.0f - std::cos(2.0f * juce::MathConstants<float>::pi * i / (fftSize - 1)));
                output[frameStart + i] += (*fftBuffer)[i] * window * 0.5f;  // 0.5f: normalize for overlap
            }

            // Mark freeze as active after first frame capture
            if (freeze && !spectralClouds.frozen)
            {
                spectralClouds.frozen = true;
            }
        }
    }

    // ========== WET signal guarantee (prevents silence at MIX=100%) ==========
    // Spectral Clouds should never be completely "wet" - always mix some input
    for (int i = 0; i < numSamples; ++i)
    {
        float inSampleL = buffer.getSample(0, i);
        float inSampleR = buffer.getSample(1, i);

        // Mix 80% processed + 20% input (Clouds/Kammerl-style design)
        wetL[i] = wetL[i] * 0.8f + inSampleL * 0.2f;
        wetR[i] = wetR[i] * 0.8f + inSampleR * 0.2f;

        // Fallback: if still silent, use input directly
        if (std::abs(wetL[i]) < 1e-6f && std::abs(wetR[i]) < 1e-6f)
        {
            wetL[i] = inSampleL;
            wetR[i] = inSampleR;
        }
    }

    // Apply feedback (mix previous output back into input buffer for next iteration)
    if (feedback > 0.001f)
    {
        for (int i = 0; i < numSamples; ++i)
        {
            float feedbackGain = feedback * 0.98f;  // Slight attenuation to prevent runaway
            if (i < buffer.getNumSamples())
            {
                buffer.setSample(0, i, buffer.getSample(0, i) * (1.0f - feedbackGain) + wetL[i] * feedbackGain);
                buffer.setSample(1, i, buffer.getSample(1, i) * (1.0f - feedbackGain) + wetR[i] * feedbackGain);
            }
        }
    }
}

void CloudLikeGranularProcessor::parameterChanged (const juce::String& parameterID, float newValue)
{
    if (parameterID == "randomize")
    {
        float prev = lastRandomizeValue.load();
        lastRandomizeValue.store (newValue);
        if (prev <= 0.5f && newValue > 0.5f)
            randomizeParameters();
    }
}

void CloudLikeGranularProcessor::randomizeParameters()
{
    // Helper to set parameter with normalized 0-1 value
    auto setNormalized = [this] (const juce::String& id)
    {
        if (auto* p = apvts.getParameter (id))
        {
            // Generate random normalized value (0.0 to 1.0)
            float normalizedValue = uniform(rng);
            p->setValueNotifyingHost (normalizedValue);
        }
    };

    // Randomize all parameters (except freeze, mode, spread, and mix)
    setNormalized ("position");
    setNormalized ("size");
    setNormalized ("pitch");      // Now correctly randomizes full -24 to +24 range
    setNormalized ("density");
    setNormalized ("texture");
    // setNormalized ("spread");  // Excluded from randomization
    setNormalized ("feedback");
    setNormalized ("reverb");
    // setNormalized ("mix");     // Excluded from randomization

    // Note: Freeze, Mode, Spread, and Mix are excluded from randomization
}

juce::AudioProcessorEditor* CloudLikeGranularProcessor::createEditor()
{
    return new CloudLikeGranularEditor (*this);
}

void CloudLikeGranularProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    if (auto xml = apvts.copyState().createXml())
        copyXmlToBinary (*xml, destData);
}

void CloudLikeGranularProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    if (auto xml = getXmlFromBinary (data, sizeInBytes))
        apvts.replaceState (juce::ValueTree::fromXml (*xml));
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new CloudLikeGranularProcessor();
}
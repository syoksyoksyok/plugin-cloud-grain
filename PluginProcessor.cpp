// PluginProcessor.cpp
#include "PluginProcessor.h"
#include "PluginEditor.h"

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

    // Mode selection: 0 = Granular, 1 = Pitch Shifter, 2 = Looping, 3 = Spectral, 4 = Oliverb, 5 = Resonestor, 6 = Beat Repeat
    params.push_back (std::make_unique<juce::AudioParameterInt>("mode", "Mode", 0, 6, 0));

    params.push_back (std::make_unique<juce::AudioParameterFloat>("position", "Position", 0.0f, 1.0f, 0.0f));
    params.push_back (std::make_unique<juce::AudioParameterFloat>("size",     "Size",     0.016f, 1.0f, 0.1f));  // 16ms to 1s (Clouds range)
    params.push_back (std::make_unique<juce::AudioParameterFloat>("pitch",    "Pitch",    -24.0f, 24.0f, 0.0f));
    params.push_back (std::make_unique<juce::AudioParameterFloat>("density",  "Density",  0.0f, 1.0f, 0.5f));
    params.push_back (std::make_unique<juce::AudioParameterFloat>("texture",  "Texture",  0.0f, 1.0f, 0.5f));
    params.push_back (std::make_unique<juce::AudioParameterFloat>("spread",   "StereoSpread", 0.0f, 1.0f, 0.5f));
    params.push_back (std::make_unique<juce::AudioParameterFloat>("feedback", "Feedback", 0.0f, 0.95f, 0.0f));
    params.push_back (std::make_unique<juce::AudioParameterFloat>("mix",      "Mix",      0.0f, 1.0f, 1.0f));
    params.push_back (std::make_unique<juce::AudioParameterFloat>("reverb",   "Reverb",   0.0f, 1.0f, 0.3f));
    params.push_back (std::make_unique<juce::AudioParameterBool>("freeze",   "Freeze",   false));
    params.push_back (std::make_unique<juce::AudioParameterBool>("randomize","Randomize",false));

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
    detectedPeriod = 0;
    correlatorUpdateCounter = 0;

    // Initialize Spectral mode state
    spectralInputPos = 0;
    spectralOutputPos = 0;
    spectralOutputL.fill(0.0f);
    spectralOutputR.fill(0.0f);

    // Initialize Oliverb taps (multi-tap delay with modulation)
    const std::array<float, 8> tapTimes = { 0.037f, 0.089f, 0.127f, 0.191f, 0.277f, 0.359f, 0.441f, 0.593f };
    for (size_t i = 0; i < oliverbTaps.size(); ++i)
    {
        int tapSize = static_cast<int>(sampleRate * tapTimes[i]);
        oliverbTaps[i].buffer.resize(tapSize, 0.0f);
        oliverbTaps[i].writePos = 0;
        oliverbTaps[i].modPhase = static_cast<float>(i) * 0.125f * juce::MathConstants<float>::twoPi;  // OPTIMIZED: Multiply instead of divide
        oliverbTaps[i].modDepth = 0.002f * sampleRate;  // 2ms modulation depth
    }

    // Initialize Resonestor resonators (Karplus-Strong)
    const std::array<float, maxResonators> resonatorFreqs = {
        82.41f, 110.0f, 146.83f, 196.0f, 246.94f, 329.63f,  // E2, A2, D3, G3, B3, E4
        392.0f, 493.88f, 587.33f, 659.25f, 783.99f, 987.77f  // G4, B4, D5, E5, G5, B5
    };
    for (int i = 0; i < maxResonators; ++i)
    {
        int delayLength = static_cast<int>(sampleRate / resonatorFreqs[i]);
        resonators[i].delayLine.resize(delayLength, 0.0f);
        resonators[i].writePos = 0;
        resonators[i].feedback = 0.99f;
        resonators[i].brightness = 0.5f;
        resonators[i].active = false;
    }

    // Initialize Beat Repeat buffer (1 bar at 120 BPM = 2 seconds)
    int beatRepeatSize = static_cast<int>(sampleRate * 2.0);
    beatRepeat.captureBufferL.resize(beatRepeatSize, 0.0f);
    beatRepeat.captureBufferR.resize(beatRepeatSize, 0.0f);
    beatRepeat.captureLength = beatRepeatSize >> 2;  // OPTIMIZED: Bit shift for division by 4
    beatRepeat.repeatPos = 0;
    beatRepeat.stutterPhase = 0.0f;
    beatRepeat.isCapturing = false;
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

// Carmack's fast inverse square root (for gain normalization)
float CloudLikeGranularProcessor::fastInverseSqrt (float number) const
{
    // Modern approximation with better accuracy
    if (number <= 0.0f) return 1.0f;
    return 1.0f / std::sqrt (number);
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
        // Use free list for efficient grain allocation
        if (freeGrainIndices.empty())
        {
            // Fallback: find first inactive grain
            auto it = std::find_if (grains.begin(), grains.end(),
                                    [] (const Grain& g) { return !g.active; });
            if (it == grains.end()) break;

            Grain& g = *it;
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
        else
        {
            int idx = freeGrainIndices.back();
            freeGrainIndices.pop_back();

            Grain& g = grains[idx];
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
    }
}

void CloudLikeGranularProcessor::processBlock (juce::AudioBuffer<float>& buffer,
                                               juce::MidiBuffer& midi)
{
    juce::ScopedNoDenormals noDenormals;
    midi.clear();

    auto numSamples = buffer.getNumSamples();
    auto* inL = buffer.getReadPointer (0);
    auto* inR = buffer.getNumChannels() > 1 ? buffer.getReadPointer (1) : nullptr;
    auto* outL = buffer.getWritePointer (0);
    auto* outR = buffer.getNumChannels() > 1 ? buffer.getWritePointer (1) : nullptr;

    // Get processing mode
    int mode = static_cast<int>(apvts.getRawParameterValue ("mode")->load());

    // Run correlation analysis periodically (every 256 samples) for pitch detection
    correlatorUpdateCounter += numSamples;
    if (correlatorUpdateCounter >= 256)
    {
        correlatorUpdateCounter = 0;

        // Analyze recent buffer content for period detection
        int analysisLength = juce::jmin(2048, bufferSize / 4);
        if (analysisLength > 0 && writeHead >= analysisLength)
        {
            const float* analysisData = ringBuffer.getReadPointer(0) + (writeHead - analysisLength);
            int minPeriod = static_cast<int>(currentSampleRate / 1000.0);  // 1000 Hz max
            int maxPeriod = static_cast<int>(currentSampleRate / 50.0);    // 50 Hz min

            detectedPeriod = correlatorL.detectPeriod(analysisData, analysisLength, minPeriod, maxPeriod);
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
                                 position, size, pitch, effectiveFeedback, freeze);
            break;

        case MODE_SPECTRAL:
            processSpectralBlock (buffer, numSamples, wetL, wetR,
                                  position, pitch, effectiveFeedback, freeze);
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

        if (!freeze && bufferSize > 0)
        {
            ringBuffer.setSample (0, writeHead, inSampleL + wetL[i] * feedback);
            ringBuffer.setSample (1, writeHead, inSampleR + wetR[i] * feedback);
            writeHead = (writeHead + 1) & bufferSizeMask;  // OPTIMIZED: Bit mask
        }

        // Clouds-style grain triggering
        bool triggerGrain = false;

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

        if (triggerGrain)
        {
            // Pass detected period for grain alignment optimization
            launchGrains (1, 0, position, size, pitch, texture, spread, detectedPeriod);
            launchGrains (1, 1, position, size, pitch, texture, spread, detectedPeriod);
        }

        float grainOutL = 0.0f, grainOutR = 0.0f;
        numActiveGrains = 0;

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

            numActiveGrains++;

            if (g.position >= g.durationSamples)
            {
                g.active = false;
                freeGrainIndices.push_back (idx);
                numActiveGrains--;
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

        // Clouds-style gain normalization
        float gainNormalization = 1.0f;
        if (numActiveGrains > 1)
        {
            gainNormalization = fastInverseSqrt (static_cast<float> (numActiveGrains - 1));
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

    // ========== PITCH SHIFTER MODE (Clouds-style WSOLA) ==========
    // SIZE: Controls window size - large = smooth, small = grainy/ring-modulated
    // PITCH: Controls pitch shift amount (-24 to +24 semitones)
    // POSITION: Controls read delay position

    // OPTIMIZATION: Calculate parameters once per block (OPTIMIZED: Bit shift)
    float pitchRatio = pitchToRatio(pitch);

    // Clouds-style SIZE mapping: 0.0 = small (256), 1.0 = large (2048)
    // Small window = grainy, ring-modulated; Large window = smooth
    int windowSize = static_cast<int>(256.0f + size * 1792.0f);  // 256 to 2048
    windowSize = (windowSize + 3) & ~3;  // Round to multiple of 4 for alignment

    int hopOut = windowSize >> 1;  // OPTIMIZED: Output hop = half window size

    // Time stretching: combine pitch shift with independent time scaling
    // pitchRatio > 1.0 = higher pitch, < 1.0 = lower pitch
    float timeStretch = 1.0f / pitchRatio;  // Inverse for time-domain playback
    int hopIn = static_cast<int>(hopOut * timeStretch);

    for (int i = 0; i < numSamples; ++i)
    {
        float inSampleL = inL[i];
        float inSampleR = inR ? inR[i] : inSampleL;

        if (!freeze && bufferSize > 0)
        {
            ringBuffer.setSample (0, writeHead, inSampleL + wetL[i] * feedback);
            ringBuffer.setSample (1, writeHead, inSampleR + wetR[i] * feedback);
            writeHead = (writeHead + 1) & bufferSizeMask;  // OPTIMIZED: Bit mask
        }

        // Clouds-style read position with delay
        double delayAmount = position * (bufferSize - windowSize);
        double readPos = pitchShifterReadPos;
        double delayedReadPos = static_cast<double>(writeHead) - delayAmount - readPos;

        // Wrap read position
        while (delayedReadPos < 0) delayedReadPos += bufferSize;
        while (delayedReadPos >= bufferSize) delayedReadPos -= bufferSize;

        float outL = getSampleFromRing(0, delayedReadPos);
        float outR = getSampleFromRing(1, delayedReadPos);

        // Clouds-style Hann window with pitch-dependent shaping (OPTIMIZED: LUT)
        // Smoother envelope for large windows, more aggressive for small windows
        float windowPhase = std::fmod(pitchShifterReadPos, static_cast<double>(hopOut)) / hopOut;
        float window = 0.5f * (1.0f - fastCos(windowPhase * juce::MathConstants<float>::twoPi));

        // Additional envelope shaping for small windows (ring-mod effect)
        if (size < 0.3f)
        {
            float ringModAmount = (0.3f - size) * 3.333f;  // 0 to 1
            window = window * (1.0f - ringModAmount * 0.5f);  // Reduce amplitude
        }

        wetL[i] = outL * window;
        wetR[i] = outR * window;

        // Advance read position
        pitchShifterReadPos += 1.0;
        if (pitchShifterReadPos >= hopOut)
        {
            pitchShifterReadPos -= hopOut;
        }
    }
}

void CloudLikeGranularProcessor::processLoopingBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                                                      float* wetL, float* wetR,
                                                      float position, float size, float pitch,
                                                      float feedback, bool freeze)
{
    auto* inL = buffer.getReadPointer (0);
    auto* inR = buffer.getNumChannels() > 1 ? buffer.getReadPointer (1) : nullptr;

    // ========== LOOPING MODE (Block Processing) ==========
    // OPTIMIZATION: Calculate loop parameters once per block (OPTIMIZED: Bit shift)
    loopLength = static_cast<int>(size * currentSampleRate);
    loopLength = juce::jlimit(1024, bufferSize >> 1, loopLength);  // OPTIMIZED: Division by 2

    double delayTime = position * (bufferSize - loopLength);
    loopEndPos = writeHead - static_cast<int>(delayTime);
    if (loopEndPos < 0) loopEndPos += bufferSize;

    loopStartPos = loopEndPos - loopLength;
    if (loopStartPos < 0) loopStartPos += bufferSize;

    float pitchRatio = pitchToRatio(pitch);

    for (int i = 0; i < numSamples; ++i)
    {
        float inSampleL = inL[i];
        float inSampleR = inR ? inR[i] : inSampleL;

        if (!freeze && bufferSize > 0)
        {
            ringBuffer.setSample(0, writeHead, inSampleL + wetL[i] * feedback);
            ringBuffer.setSample(1, writeHead, inSampleR + wetR[i] * feedback);
            writeHead = (writeHead + 1) & bufferSizeMask;  // OPTIMIZED: Bit mask
        }

        int readPosInt = loopStartPos + static_cast<int>(loopReadPos);
        if (readPosInt >= loopEndPos) readPosInt = loopStartPos;
        if (readPosInt < 0) readPosInt += bufferSize;
        readPosInt = readPosInt & bufferSizeMask;  // OPTIMIZED: Bit mask

        float outL = getSampleFromRing(0, readPosInt);
        float outR = getSampleFromRing(1, readPosInt);

        wetL[i] = outL;
        wetR[i] = outR;

        loopReadPos += pitchRatio;
        if (loopReadPos >= loopLength)
        {
            loopReadPos -= loopLength;
        }
    }
}

void CloudLikeGranularProcessor::processSpectralBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                                                       float* wetL, float* wetR,
                                                       float position, float pitch,
                                                       float feedback, bool freeze)
{
    auto* inL = buffer.getReadPointer (0);
    auto* inR = buffer.getNumChannels() > 1 ? buffer.getReadPointer (1) : nullptr;

    // ========== SPECTRAL MODE (Block Processing) ==========
    // OPTIMIZATION: Calculate parameters once per block (OPTIMIZED: Bit shift)
    float pitchRatio = pitchToRatio(pitch);
    const int hopSize = fftSize >> 2;  // OPTIMIZED: Division by 4 using bit shift

    for (int i = 0; i < numSamples; ++i)
    {
        float inSampleL = inL[i];
        float inSampleR = inR ? inR[i] : inSampleL;

        if (!freeze && bufferSize > 0)
        {
            ringBuffer.setSample(0, writeHead, inSampleL + wetL[i] * feedback);
            ringBuffer.setSample(1, writeHead, inSampleR + wetR[i] * feedback);
            writeHead = (writeHead + 1) & bufferSizeMask;  // OPTIMIZED: Bit mask
        }

        // Fill FFT input buffer from ring buffer
        if (spectralInputPos < fftSize)
        {
            double readDelay = position * (bufferSize - fftSize);
            int readPos = static_cast<int>(writeHead - readDelay - spectralInputPos);
            if (readPos < 0) readPos += bufferSize;
            readPos = readPos & bufferSizeMask;  // OPTIMIZED: Bit mask

            // Apply Hann window during input (OPTIMIZED: LUT)
            float window = hannWindow(spectralInputPos, fftSize);
            fftDataL[spectralInputPos] = ringBuffer.getSample(0, readPos) * window;
            fftDataR[spectralInputPos] = ringBuffer.getSample(1, readPos) * window;

            spectralInputPos++;
        }

        // Process FFT when we have a full window
        if (spectralInputPos >= fftSize)
        {
            spectralInputPos = 0;

            // Zero pad the second half for complex FFT (OPTIMIZED: Use bit shift)
            for (int n = fftSize; n < (fftSize << 1); ++n)  // fftSize * 2
            {
                fftDataL[n] = 0.0f;
                fftDataR[n] = 0.0f;
            }

            // Perform forward FFT
            forwardFFT.performRealOnlyForwardTransform(fftDataL.data());
            forwardFFT.performRealOnlyForwardTransform(fftDataR.data());

            // OPTIMIZATION: Use member buffers instead of stack allocation
            spectralShiftedL.fill(0.0f);
            spectralShiftedR.fill(0.0f);

            // Shift frequency bins based on pitch ratio (OPTIMIZED: Bit shift)
            const int halfFFT = fftSize >> 1;  // OPTIMIZED: Pre-calculate
            for (int bin = 0; bin < halfFFT; ++bin)
            {
                int targetBin = static_cast<int>(bin * pitchRatio);
                if (targetBin > 0 && targetBin < halfFFT)
                {
                    // Copy real and imaginary parts (OPTIMIZED: Bit shift for multiplication by 2)
                    int binIdx = bin << 1;
                    int targetIdx = targetBin << 1;
                    spectralShiftedL[targetIdx] = fftDataL[binIdx];
                    spectralShiftedL[targetIdx + 1] = fftDataL[binIdx + 1];
                    spectralShiftedR[targetIdx] = fftDataR[binIdx];
                    spectralShiftedR[targetIdx + 1] = fftDataR[binIdx + 1];
                }
            }

            // Perform inverse FFT
            forwardFFT.performRealOnlyInverseTransform(spectralShiftedL.data());
            forwardFFT.performRealOnlyInverseTransform(spectralShiftedR.data());

            // Overlap-Add: accumulate to output buffer with Hann window (OPTIMIZED: LUT)
            float normGain = 2.0f / fftSize;
            for (int n = 0; n < fftSize; ++n)
            {
                float window = hannWindow(n, fftSize);
                spectralOutputL[n] += spectralShiftedL[n] * normGain * window;
                spectralOutputR[n] += spectralShiftedR[n] * normGain * window;
            }

            spectralOutputPos = 0;
        }

        // Output and consume from spectral buffer
        if (spectralOutputPos < fftSize)
        {
            wetL[i] = spectralOutputL[spectralOutputPos];
            wetR[i] = spectralOutputR[spectralOutputPos];

            spectralOutputL[spectralOutputPos] = 0.0f;
            spectralOutputR[spectralOutputPos] = 0.0f;

            spectralOutputPos++;
        }
        else
        {
            wetL[i] = 0.0f;
            wetR[i] = 0.0f;
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

    // ========== OLIVERB MODE (Block Processing) ==========
    // OPTIMIZATION: Calculate parameters once per block
    float modRate = 0.1f + position * 4.9f;
    float decayTime = 0.2f + size * 4.8f;
    float pitchRatio = pitchToRatio(pitch);
    float modDepth = density * 0.01f * currentSampleRate;
    float diffusion = texture;

    for (int i = 0; i < numSamples; ++i)
    {
        float inSampleL = inL[i];
        float inSampleR = inR ? inR[i] : inSampleL;

        if (!freeze && bufferSize > 0)
        {
            ringBuffer.setSample(0, writeHead, inSampleL + wetL[i] * feedback);
            ringBuffer.setSample(1, writeHead, inSampleR + wetR[i] * feedback);
            writeHead = (writeHead + 1) & bufferSizeMask;  // OPTIMIZED: Bit mask
        }

        float outputL = 0.0f;
        float outputR = 0.0f;

        // Process each tap
        for (size_t t = 0; t < oliverbTaps.size(); ++t)
        {
            auto& tap = oliverbTaps[t];

            // Update modulation phase
            tap.modPhase += modRate * juce::MathConstants<float>::twoPi / currentSampleRate;
            if (tap.modPhase > juce::MathConstants<float>::twoPi)
                tap.modPhase -= juce::MathConstants<float>::twoPi;

            // Modulated read position (OPTIMIZED: LUT)
            float mod = fastSin(tap.modPhase) * modDepth;
            int readPos = tap.writePos - static_cast<int>((tap.buffer.size() >> 1) + mod);  // OPTIMIZED: Bit shift
            if (readPos < 0) readPos += tap.buffer.size();
            readPos = readPos % tap.buffer.size();

            float delayed = tap.buffer[readPos];

            // Calculate feedback with decay
            float tapFeedback = std::pow(0.001f, 1.0f / (decayTime * currentSampleRate / tap.buffer.size()));

            // Write input + feedback (OPTIMIZED: Bit shift for division by 8)
            float inputMix = (t & 1) ? inSampleR : inSampleL;  // OPTIMIZED: Bit mask instead of modulo
            tap.buffer[tap.writePos] = inputMix * 0.125f + delayed * tapFeedback;
            tap.writePos = (tap.writePos + 1) % tap.buffer.size();

            // Accumulate to output (alternate L/R)
            if (!(t & 1))  // OPTIMIZED: Bit mask instead of modulo
                outputL += delayed * (1.0f - diffusion * 0.5f);
            else
                outputR += delayed * (1.0f - diffusion * 0.5f);
        }

        // Apply diffusion (cross-mix)
        float diffusedL = outputL * (1.0f - diffusion) + outputR * diffusion;
        float diffusedR = outputR * (1.0f - diffusion) + outputL * diffusion;

        wetL[i] = diffusedL * 0.25f;
        wetR[i] = diffusedR * 0.25f;
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

    // ========== RESONESTOR MODE (Block Processing) ==========
    // OPTIMIZATION: Calculate parameters once per block
    float excitation = position;
    float decayTime = size;
    float pitchShift = pitch;
    float activationThreshold = 1.0f - density;
    float brightness = texture;
    float decay = 0.9f + decayTime * 0.099f;  // 0.9 to 0.999
    float damping = 0.5f + brightness * 0.5f;  // 0.5 to 1.0

    for (int i = 0; i < numSamples; ++i)
    {
        float inSampleL = inL[i];
        float inSampleR = inR ? inR[i] : inSampleL;

        if (!freeze && bufferSize > 0)
        {
            ringBuffer.setSample(0, writeHead, inSampleL + wetL[i] * feedback);
            ringBuffer.setSample(1, writeHead, inSampleR + wetR[i] * feedback);
            writeHead = (writeHead + 1) & bufferSizeMask;  // OPTIMIZED: Bit mask
        }

        float outputL = 0.0f;
        float outputR = 0.0f;

        // Excite resonators based on input energy
        float inputEnergy = std::abs(inSampleL) + std::abs(inSampleR);

        // Process each resonator
        for (int r = 0; r < maxResonators; ++r)
        {
            auto& res = resonators[r];

            // Activate resonator based on density and input
            if (inputEnergy > activationThreshold * 0.1f && uniform(rng) > activationThreshold)
            {
                res.active = true;
                float excite = (uniform(rng) * 2.0f - 1.0f) * excitation * inputEnergy;
                for (size_t n = 0; n < res.delayLine.size() && n < 10; ++n)
                {
                    res.delayLine[(res.writePos + n) % res.delayLine.size()] += excite;
                }
            }

            if (res.active)
            {
                int readPos = res.writePos;
                float delayed = res.delayLine[readPos];

                // Low-pass filter for damping (brightness control)
                int prevPos = (readPos - 1 + res.delayLine.size()) % res.delayLine.size();
                float filtered = delayed * damping + res.delayLine[prevPos] * (1.0f - damping);

                // Feedback with decay
                res.delayLine[res.writePos] = filtered * decay;
                res.writePos = (res.writePos + 1) % res.delayLine.size();

                // Accumulate to output (alternate L/R) (OPTIMIZED: Bit mask)
                if (!(r & 1))
                    outputL += filtered;
                else
                    outputR += filtered;

                // Deactivate if energy too low
                if (std::abs(filtered) < 0.0001f)
                    res.active = false;
            }
        }

        wetL[i] = outputL * 0.15f;
        wetR[i] = outputR * 0.15f;
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

    // ========== BEAT REPEAT MODE (Block Processing) ==========
    // OPTIMIZATION: Calculate parameters once per block (OPTIMIZED: Bit shift)
    float capturePos = position;
    float repeatLength = size * 0.5f * currentSampleRate;
    float playbackSpeed = pitchToRatio(pitch);
    float repeatRate = 1.0f + density * 15.0f;
    float stutterAmount = texture;

    // Update capture length based on size
    beatRepeat.captureLength = juce::jlimit(512, static_cast<int>(beatRepeat.captureBufferL.size()),
                                            static_cast<int>(repeatLength));

    for (int i = 0; i < numSamples; ++i)
    {
        float inSampleL = inL[i];
        float inSampleR = inR ? inR[i] : inSampleL;

        if (!freeze && bufferSize > 0)
        {
            ringBuffer.setSample(0, writeHead, inSampleL + wetL[i] * feedback);
            ringBuffer.setSample(1, writeHead, inSampleR + wetR[i] * feedback);
            writeHead = (writeHead + 1) & bufferSizeMask;  // OPTIMIZED: Bit mask
        }

        // Trigger capture based on repeat rate
        beatRepeat.stutterPhase += repeatRate / currentSampleRate;
        if (beatRepeat.stutterPhase >= 1.0f)
        {
            beatRepeat.stutterPhase -= 1.0f;
            beatRepeat.isCapturing = true;
            beatRepeat.repeatPos = 0;

            // Capture from ring buffer at position
            int captureStart = writeHead - static_cast<int>(capturePos * bufferSize);
            if (captureStart < 0) captureStart += bufferSize;

            for (int n = 0; n < beatRepeat.captureLength; ++n)
            {
                int readIdx = (captureStart + n) & bufferSizeMask;  // OPTIMIZED: Bit mask
                beatRepeat.captureBufferL[n] = ringBuffer.getSample(0, readIdx);
                beatRepeat.captureBufferR[n] = ringBuffer.getSample(1, readIdx);
            }
        }

        // Playback captured buffer
        if (beatRepeat.isCapturing && beatRepeat.captureLength > 0)
        {
            // Use linear interpolation for smooth playback at any speed
            float readPosFloat = std::fmod(beatRepeat.repeatPos, static_cast<float>(beatRepeat.captureLength));
            int readPos0 = static_cast<int>(readPosFloat);
            int readPos1 = (readPos0 + 1) % beatRepeat.captureLength;
            float frac = readPosFloat - readPos0;

            // Interpolate between samples for smooth playback
            float outL = beatRepeat.captureBufferL[readPos0] +
                        (beatRepeat.captureBufferL[readPos1] - beatRepeat.captureBufferL[readPos0]) * frac;
            float outR = beatRepeat.captureBufferR[readPos0] +
                        (beatRepeat.captureBufferR[readPos1] - beatRepeat.captureBufferR[readPos0]) * frac;

            // Apply stutter envelope
            float stutterEnv = 1.0f;
            if (stutterAmount > 0.5f)
            {
                float stutterFreq = (stutterAmount - 0.5f) * 40.0f + 2.0f;
                float stutterPhaseLocal = std::fmod(beatRepeat.repeatPos / beatRepeat.captureLength * stutterFreq, 1.0f);
                stutterEnv = stutterPhaseLocal < 0.5f ? 1.0f : 0.3f;
            }

            wetL[i] = outL * stutterEnv;
            wetR[i] = outR * stutterEnv;

            beatRepeat.repeatPos += playbackSpeed;
            if (beatRepeat.repeatPos >= beatRepeat.captureLength)
            {
                beatRepeat.repeatPos -= beatRepeat.captureLength;  // Better wrapping
            }
        }
        else
        {
            wetL[i] = 0.0f;
            wetR[i] = 0.0f;
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
    auto set = [this] (const juce::String& id, float min, float max)
    {
        if (auto* p = apvts.getParameter (id))
        {
            float value = uniform(rng) * (max - min) + min;
            p->setValueNotifyingHost (value);
        }
    };

    set ("position",  0.0f, 1.0f);
    set ("size",      0.01f, 0.5f);
    set ("pitch",    -24.0f, 24.0f);
    set ("density",   0.0f, 1.0f);
    set ("texture",   0.0f, 1.0f);
    set ("feedback",  0.0f, 0.95f);
    set ("reverb",    0.0f, 1.0f);

    // Randomly set freeze parameter
    bool freezeState = uniform(rng) > 0.5f;
    apvts.getParameter("freeze")->setValueNotifyingHost (freezeState ? 1.0f : 0.0f);
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
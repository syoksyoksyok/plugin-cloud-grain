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

    bufferSize = static_cast<int> (sampleRate * ringBufferSeconds);
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
}

float CloudLikeGranularProcessor::getSampleFromRing (int channel, double index) const
{
    if (bufferSize <= 0) return 0.0f;

    // Efficient modulo for negative indices
    index = std::fmod (index, static_cast<double> (bufferSize));
    if (index < 0.0) index += bufferSize;

    int i0 = static_cast<int> (index);
    int i1 = (i0 + 1) % bufferSize;
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
                                               float stereoSpread)
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

    double pitchRatio = std::pow (2.0, pitchSemis / 12.0);
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

            // Clouds-style per-grain stereo gain
            float panAngle = (g.pan + 1.0f) * 0.5f * juce::MathConstants<float>::halfPi;
            g.gainL = std::cos(panAngle);
            g.gainR = std::sin(panAngle);
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

            // Clouds-style per-grain stereo gain
            float panAngle = (g.pan + 1.0f) * 0.5f * juce::MathConstants<float>::halfPi;
            g.gainL = std::cos(panAngle);
            g.gainR = std::sin(panAngle);
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

    auto position  = apvts.getRawParameterValue ("position")->load();
    auto size      = apvts.getRawParameterValue ("size")->load();
    auto pitch     = apvts.getRawParameterValue ("pitch")->load();
    auto density   = apvts.getRawParameterValue ("density")->load();
    auto texture   = apvts.getRawParameterValue ("texture")->load();
    auto spread    = apvts.getRawParameterValue ("spread")->load();
    auto feedback  = apvts.getRawParameterValue ("feedback")->load();
    auto mix       = apvts.getRawParameterValue ("mix")->load();
    auto reverbAmt = apvts.getRawParameterValue ("reverb")->load();
    auto freeze    = apvts.getRawParameterValue ("freeze")->load() > 0.5f;

    if (wetBuffer.getNumSamples() < numSamples)
        wetBuffer.setSize (2, numSamples, false, false, true);
    if (reverbBuffer.getNumSamples() < numSamples)
        reverbBuffer.setSize (2, numSamples, false, false, true);

    wetBuffer.clear();
    reverbBuffer.clear();

    auto* wetL = wetBuffer.getWritePointer (0);
    auto* wetR = wetBuffer.getWritePointer (1);

    float effectiveFeedback = freeze ? 1.0f : feedback;

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
            ringBuffer.setSample (0, writeHead, inSampleL + wetL[i] * effectiveFeedback);
            ringBuffer.setSample (1, writeHead, inSampleR + wetR[i] * effectiveFeedback);
            writeHead = (writeHead + 1) % bufferSize;
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
            launchGrains (1, 0, position, size, pitch, texture, spread);
            launchGrains (1, 1, position, size, pitch, texture, spread);
        }

        float grainOutL = 0.0f, grainOutR = 0.0f;
        numActiveGrains = 0;

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
            double readIndex1 = g.startSample + g.position;

            // Calculate second read pointer within grain bounds
            double normalizedPhase = g.phase;
            double offset = normalizedPhase * g.durationSamples;
            double readIndex2 = g.startSample + offset;

            // Triangular envelope for crossfading
            float tri = 2.0f * (g.phase >= 0.5 ? 1.0f - static_cast<float>(g.phase) : static_cast<float>(g.phase));

            // Read from both positions
            float s1 = getSampleFromRing (g.channel, readIndex1);
            float s2 = getSampleFromRing (g.channel, readIndex2);

            // Crossfade between two read positions
            float s = s1 * tri + s2 * (1.0f - tri);

            float env = getGrainEnvelope (g.position, g.durationSamples, texture);
            float v = s * env;

            // Update phase for next sample
            g.phase += g.phaseIncrement;
            if (g.phase < 0.0) g.phase += 1.0;
            if (g.phase >= 1.0) g.phase -= 1.0;

            // Clouds-style per-grain stereo gain
            grainOutL += v * g.gainL;
            grainOutR += v * g.gainR;

            g.position += g.phaseInc;
        }

        // Clouds-style gain normalization
        float gainNormalization = 1.0f;
        if (numActiveGrains > 1)
        {
            // Fast inverse square root for normalization
            gainNormalization = fastInverseSqrt (static_cast<float> (numActiveGrains - 1));
        }

        // Window gain scaling (1.0 to 2.0 based on overlap)
        float windowGain = 1.0f + overlap;

        // Target gain with smoothing
        float targetGain = gainNormalization * windowGain;

        // One-pole smoothing filter (0.01 coefficient)
        smoothedGain += 0.01f * (targetGain - smoothedGain);

        // Apply gain
        float outputL = grainOutL * smoothedGain;
        float outputR = grainOutR * smoothedGain;

        // Clouds-style diffuser (activated when TEXTURE > 0.75)
        if (texture > 0.75f)
        {
            float diffuserAmount = (texture - 0.75f) * 4.0f;  // 0 to 1
            float feedback = 0.625f;

            // Apply cascaded allpass filters
            float diffusedL = diffuserL1.process(outputL, feedback);
            diffusedL = diffuserL2.process(diffusedL, feedback);
            diffusedL = diffuserL3.process(diffusedL, feedback);

            float diffusedR = diffuserR1.process(outputR, feedback);
            diffusedR = diffuserR2.process(diffusedR, feedback);
            diffusedR = diffuserR3.process(diffusedR, feedback);

            // Crossfade between dry and diffused
            outputL = outputL * (1.0f - diffuserAmount) + diffusedL * diffuserAmount;
            outputR = outputR * (1.0f - diffuserAmount) + diffusedR * diffuserAmount;
        }

        wetL[i] = outputL;
        wetR[i] = outputR;
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
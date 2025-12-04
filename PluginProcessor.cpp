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

    // Mode selection (for future expansion)
    juce::StringArray modeChoices { "Granular", "WSOLA", "Looping", "Spectral" };
    params.push_back (std::make_unique<juce::AudioParameterChoice>("mode", "Mode", modeChoices, 0));

    params.push_back (std::make_unique<juce::AudioParameterFloat>("position", "Position", 0.0f, 1.0f, 0.0f));
    params.push_back (std::make_unique<juce::AudioParameterFloat>("size",     "Size",     0.01f, 0.5f, 0.1f));
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

    // Initialize diffuser allpass filters with different delays
    auto diffuserCoeffs = juce::dsp::IIR::Coefficients<float>::makeAllPass (sampleRate, 500.0f);
    diffuserL1.coefficients = diffuserCoeffs;
    diffuserR1.coefficients = diffuserCoeffs;

    diffuserCoeffs = juce::dsp::IIR::Coefficients<float>::makeAllPass (sampleRate, 1200.0f);
    diffuserL2.coefficients = diffuserCoeffs;
    diffuserR2.coefficients = diffuserCoeffs;

    diffuserCoeffs = juce::dsp::IIR::Coefficients<float>::makeAllPass (sampleRate, 2000.0f);
    diffuserL3.coefficients = diffuserCoeffs;
    diffuserR3.coefficients = diffuserCoeffs;

    diffuserL1.reset();
    diffuserL2.reset();
    diffuserL3.reset();
    diffuserR1.reset();
    diffuserR2.reset();
    diffuserR3.reset();
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

    // Clouds-style window shape mapping
    float windowShape = textureParam < 0.75f ? textureParam * 1.333f : 1.0f;

    // Parametric window function
    float env = std::sin (juce::MathConstants<float>::pi * x);

    // Apply shape with dynamic exponent
    float shape = 0.5f + windowShape * 3.0f;
    env = std::pow (env, shape);

    return env * env;
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

    double duration      = juce::jlimit (0.01, 0.5, (double) sizeParam);
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
            double halfPhase = g.phase + 0.5;
            if (halfPhase >= 1.0) halfPhase -= 1.0;
            double readIndex2 = g.startSample + g.position * g.phaseInc + halfPhase * g.durationSamples;

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

            float panL = 0.5f * (1.0f - g.pan);
            float panR = 0.5f * (1.0f + g.pan);

            grainOutL += v * panL;
            grainOutR += v * panR;

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

        // Apply stereo diffuser based on spread parameter
        float diffusedL = grainOutL;
        float diffusedR = grainOutR;

        if (spread > 0.01f)
        {
            // Apply cascaded allpass filters for diffusion
            diffusedL = diffuserL1.processSample (diffusedL);
            diffusedL = diffuserL2.processSample (diffusedL);
            diffusedL = diffuserL3.processSample (diffusedL);

            diffusedR = diffuserR1.processSample (diffusedR);
            diffusedR = diffuserR2.processSample (diffusedR);
            diffusedR = diffuserR3.processSample (diffusedR);

            // Crossfade between dry and diffused based on spread
            grainOutL = grainOutL * (1.0f - spread) + diffusedL * spread;
            grainOutR = grainOutR * (1.0f - spread) + diffusedR * spread;
        }

        wetL[i] = grainOutL * smoothedGain;
        wetR[i] = grainOutR * smoothedGain;
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
// PluginProcessor.h
#pragma once

#include <JuceHeader.h>
#include <array>
#include <random>
#include <atomic>

class CloudLikeGranularEditor;

//==============================================================================
class CloudLikeGranularProcessor
    : public juce::AudioProcessor
    , public juce::AudioProcessorValueTreeState::Listener
{
public:
    CloudLikeGranularProcessor();
    ~CloudLikeGranularProcessor() override;

    const juce::String getName() const override { return "GranularTexture"; }

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override {}

    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;

    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    bool acceptsMidi() const override      { return false; }
    bool producesMidi() const override     { return false; }
    bool isMidiEffect() const override     { return false; }

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override        { return true; }

    double getTailLengthSeconds() const override { return 5.0; }

    int getNumPrograms() override          { return 1; }
    int getCurrentProgram() override       { return 0; }
    void setCurrentProgram (int) override  {}
    const juce::String getProgramName (int) override     { return {}; }
    void changeProgramName (int, const juce::String&) override {}

    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

    juce::AudioProcessorValueTreeState apvts;

    void parameterChanged (const juce::String& parameterID, float newValue) override;

    // Grain 構造体を public に移動（これで C2099 エラー完全解消）
    struct Grain
    {
        bool   active          = false;
        int    channel         = 0;
        double startSample     = 0.0;
        double position        = 0.0;
        double durationSamples = 0.0;
        double phaseInc        = 1.0;
        float  pan             = 0.0f;
    };

private:
    static juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();

    static constexpr int maxGrains = 64;
    std::array<Grain, maxGrains> grains;

    juce::AudioBuffer<float> ringBuffer;
    int    bufferSize = 0;
    int    writeHead = 0;
    double currentSampleRate = 44100.0;

    std::mt19937 rng { std::random_device{}() };
    std::uniform_real_distribution<float> uniform { 0.0f, 1.0f };

    juce::Reverb reverb;
    juce::AudioBuffer<float> wetBuffer;
    juce::AudioBuffer<float> reverbBuffer;

    std::atomic<float> lastRandomizeValue { 0.0f };

    void launchGrains (int numToLaunch, int channel,
                       float positionParam, float sizeParam,
                       float pitchSemis, float textureParam,
                       float stereoSpread);

    float getSampleFromRing (int channel, double index) const;
    float getGrainEnvelope (double t, double duration, float textureParam) const;

    void randomizeParameters();

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (CloudLikeGranularProcessor)
};
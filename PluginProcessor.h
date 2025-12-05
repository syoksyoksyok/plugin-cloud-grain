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

    // Grain structure (public for potential visualization)
    struct Grain
    {
        bool   active          = false;
        int    channel         = 0;
        double startSample     = 0.0;
        double position        = 0.0;
        double durationSamples = 0.0;
        double phaseInc        = 1.0;
        float  pan             = 0.0f;

        // Clouds-style pitch shifting
        double phase           = 0.0;  // Phase for triangular crossfade
        double phaseIncrement  = 0.0;  // Phase advancement

        // Clouds-style additions
        int    preDelay        = 0;    // Delay before grain activation (samples)
        float  gainL           = 1.0f; // Left channel gain
        float  gainR           = 1.0f; // Right channel gain
    };

private:
    static juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();

    // Processing modes
    enum ProcessingMode
    {
        MODE_GRANULAR = 0,
        MODE_WSOLA = 1,
        MODE_LOOPING = 2,
        MODE_SPECTRAL = 3
    };

    // Correlator for pitch detection and grain alignment (Clouds-style)
    class Correlator
    {
    public:
        Correlator() : sampleRate_(44100.0), period_(0) {}

        void setSampleRate(double sr) { sampleRate_ = sr; }

        // Detect period/pitch of input signal
        int detectPeriod(const float* samples, int numSamples, int minPeriod, int maxPeriod)
        {
            if (numSamples < maxPeriod * 2) return 0;

            float bestCorrelation = -1.0f;
            int bestPeriod = minPeriod;

            // Autocorrelation to find period
            for (int testPeriod = minPeriod; testPeriod < maxPeriod; ++testPeriod)
            {
                float correlation = 0.0f;
                float energy = 0.0001f;

                for (int i = 0; i < numSamples - testPeriod; ++i)
                {
                    correlation += samples[i] * samples[i + testPeriod];
                    energy += samples[i] * samples[i];
                }

                correlation /= energy;

                if (correlation > bestCorrelation)
                {
                    bestCorrelation = correlation;
                    bestPeriod = testPeriod;
                }
            }

            period_ = (bestCorrelation > 0.5f) ? bestPeriod : 0;
            return period_;
        }

        int getPeriod() const { return period_; }

    private:
        double sampleRate_;
        int period_;
    };

    // Simple allpass filter for diffuser (no DSP module required)
    class SimpleAllpass
    {
    public:
        SimpleAllpass() : buffer(nullptr), bufferSize(0), writePos(0) {}

        void setDelay(int delaySamples)
        {
            if (delaySamples != bufferSize)
            {
                bufferSize = delaySamples;
                buffer.reset(new float[bufferSize]);
                clear();
            }
        }

        void clear()
        {
            if (buffer)
                std::fill(buffer.get(), buffer.get() + bufferSize, 0.0f);
            writePos = 0;
        }

        float process(float input, float feedback = 0.5f)
        {
            if (!buffer || bufferSize == 0) return input;

            int readPos = (writePos - bufferSize + bufferSize) % bufferSize;
            float delayed = buffer[readPos];

            float output = -input + delayed;
            buffer[writePos] = input + delayed * feedback;

            writePos = (writePos + 1) % bufferSize;
            return output;
        }

    private:
        std::unique_ptr<float[]> buffer;
        int bufferSize;
        int writePos;
    };

    // Constants
    static constexpr int maxGrains = 64;
    static constexpr double ringBufferSeconds = 4.0;
    static constexpr float minGrainsPerSecond = 4.0f;
    static constexpr float maxGrainsPerSecond = 100.0f;
    std::array<Grain, maxGrains> grains;
    std::vector<int> freeGrainIndices;

    juce::AudioBuffer<float> ringBuffer;
    int    bufferSize = 0;
    int    writeHead = 0;
    double currentSampleRate = 44100.0;

    std::mt19937 rng { std::random_device{}() };
    std::uniform_real_distribution<float> uniform { 0.0f, 1.0f };

    juce::Reverb reverb;
    juce::AudioBuffer<float> wetBuffer;
    juce::AudioBuffer<float> reverbBuffer;

    // Stereo diffuser (Clouds-style, no DSP module)
    SimpleAllpass diffuserL1, diffuserL2, diffuserL3;
    SimpleAllpass diffuserR1, diffuserR2, diffuserR3;

    // Correlator for pitch detection and grain optimization
    Correlator correlatorL, correlatorR;
    int detectedPeriod = 0;
    int correlatorUpdateCounter = 0;

    // WSOLA state
    double wsolaReadPos = 0.0;
    int wsolaWindowSize = 2048;
    int wsolaSearchWindow = 512;

    // Looping mode state
    double loopReadPos = 0.0;
    int loopStartPos = 0;
    int loopEndPos = 0;
    int loopLength = 0;

    // Spectral mode state (FFT-based using juce::dsp)
    static constexpr int fftOrder = 11;
    static constexpr int fftSize = 1 << fftOrder;  // 2048
    juce::dsp::FFT forwardFFT { fftOrder };
    std::array<float, fftSize * 2> fftDataL {};  // Zero-initialized
    std::array<float, fftSize * 2> fftDataR {};
    int spectralInputPos = 0;
    int spectralOutputPos = 0;
    std::array<float, fftSize> spectralOutputL {};  // Zero-initialized
    std::array<float, fftSize> spectralOutputR {};

    std::atomic<float> lastRandomizeValue { 0.0f };

    // Clouds-style density control
    float grainRatePhasor = 0.0f;
    int numActiveGrains = 0;
    float smoothedGain = 1.0f;

    void launchGrains (int numToLaunch, int channel,
                       float positionParam, float sizeParam,
                       float pitchSemis, float textureParam,
                       float stereoSpread, int periodHint = 0);

    float getSampleFromRing (int channel, double index) const;
    float getGrainEnvelope (double t, double duration, float textureParam) const;

    // Clouds-style helper functions
    float fastInverseSqrt (float number) const;
    float computeOverlap (float density) const;

    // WSOLA helper: find best matching segment
    int findBestMatch (const float* reference, const float* searchBuffer,
                       int searchLength, int windowSize);

    void randomizeParameters();

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (CloudLikeGranularProcessor)
};
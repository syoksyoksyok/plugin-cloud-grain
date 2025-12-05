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
        MODE_SPECTRAL = 3,
        MODE_OLIVERB = 4,      // Parasites: Creative reverb mode
        MODE_RESONESTOR = 5,   // Parasites: Polyphonic resonator (Karplus-Strong)
        MODE_BEAT_REPEAT = 6   // Parasites: Beat repeat/stutter effect
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

    // ========== OPTIMIZATION: Look-Up Tables ==========
    static constexpr int SINE_TABLE_SIZE = 2048;
    static constexpr int HANN_WINDOW_SIZE = 2048;
    static constexpr int PITCH_LUT_SIZE = 49;  // -24 to +24 semitones

    std::array<float, SINE_TABLE_SIZE> sineLUT;
    std::array<float, SINE_TABLE_SIZE> cosineLUT;
    std::array<float, HANN_WINDOW_SIZE> hannWindowLUT;
    std::array<float, PITCH_LUT_SIZE> pitchRatioLUT;

    // Fast LUT access helpers
    inline float fastSin(float phase) const {
        int index = static_cast<int>(phase * SINE_TABLE_SIZE / juce::MathConstants<float>::twoPi) & (SINE_TABLE_SIZE - 1);
        return sineLUT[index];
    }

    inline float fastCos(float phase) const {
        int index = static_cast<int>(phase * SINE_TABLE_SIZE / juce::MathConstants<float>::twoPi) & (SINE_TABLE_SIZE - 1);
        return cosineLUT[index];
    }

    inline float hannWindow(int position, int windowSize) const {
        if (windowSize != HANN_WINDOW_SIZE) {
            // Fallback for non-standard sizes
            float x = static_cast<float>(position) / (windowSize - 1);
            return 0.5f * (1.0f - std::cos(2.0f * juce::MathConstants<float>::pi * x));
        }
        return hannWindowLUT[position];
    }

    inline float pitchToRatio(float semitones) const {
        int index = static_cast<int>(semitones) + 24;  // Offset by 24 (-24 to +24 -> 0 to 48)
        if (index < 0 || index >= PITCH_LUT_SIZE)
            return std::pow(2.0f, semitones / 12.0f);  // Fallback

        // Linear interpolation for sub-semitone accuracy
        float frac = semitones - std::floor(semitones);
        if (index + 1 < PITCH_LUT_SIZE)
            return pitchRatioLUT[index] + frac * (pitchRatioLUT[index + 1] - pitchRatioLUT[index]);
        return pitchRatioLUT[index];
    }

    // ========== OPTIMIZATION: Parameter Smoothing ==========
    struct SmoothedParameter {
        float current = 0.0f;
        float target = 0.0f;
        float coefficient = 0.01f;  // 1ms smoothing at 44.1kHz

        void reset(float value) {
            current = target = value;
        }

        void setTarget(float newTarget) {
            target = newTarget;
        }

        float getNext() {
            current += (target - current) * coefficient;
            return current;
        }

        float getCurrentValue() const {
            return current;
        }
    };

    SmoothedParameter smoothedPosition;
    SmoothedParameter smoothedSize;
    SmoothedParameter smoothedPitch;
    SmoothedParameter smoothedDensity;
    SmoothedParameter smoothedTexture;
    SmoothedParameter smoothedSpread;
    SmoothedParameter smoothedFeedback;
    SmoothedParameter smoothedMix;
    SmoothedParameter smoothedReverb;

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
    std::array<float, fftSize * 2> fftDataL;
    std::array<float, fftSize * 2> fftDataR;
    int spectralInputPos = 0;      // Input accumulation position
    int spectralOutputPos = 0;     // Output read position
    std::array<float, fftSize * 2> spectralOutputL;  // Overlap-add buffer (needs 2x size)
    std::array<float, fftSize * 2> spectralOutputR;

    // Oliverb mode state (Multi-tap reverb with modulation)
    struct OliverbTap
    {
        std::vector<float> buffer;
        int writePos = 0;
        float modPhase = 0.0f;
        float modDepth = 0.0f;
    };
    std::array<OliverbTap, 8> oliverbTaps;

    // Resonestor mode state (Karplus-Strong resonators)
    static constexpr int maxResonators = 12;
    struct Resonator
    {
        std::vector<float> delayLine;
        int writePos = 0;
        float feedback = 0.0f;
        float brightness = 0.5f;
        bool active = false;
    };
    std::array<Resonator, maxResonators> resonators;

    // Beat Repeat mode state
    struct BeatRepeatState
    {
        std::vector<float> captureBufferL;
        std::vector<float> captureBufferR;
        int captureLength = 0;
        int repeatPos = 0;
        float stutterPhase = 0.0f;
        bool isCapturing = false;
    };
    BeatRepeatState beatRepeat;

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

    // ========== OPTIMIZATION: Block Processing Functions ==========
    // Mode-specific block processing for better cache locality and reduced branching
    void processGranularBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                               float* wetL, float* wetR,
                               float position, float size, float pitch, float density,
                               float texture, float spread, float feedback, bool freeze);

    void processWSOLABlock (juce::AudioBuffer<float>& buffer, int numSamples,
                            float* wetL, float* wetR,
                            float position, float size, float pitch,
                            float feedback, bool freeze);

    void processLoopingBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                              float* wetL, float* wetR,
                              float position, float size, float pitch,
                              float feedback, bool freeze);

    void processSpectralBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                               float* wetL, float* wetR,
                               float position, float pitch,
                               float feedback, bool freeze);

    void processOliverbBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                              float* wetL, float* wetR,
                              float position, float size, float pitch,
                              float density, float texture,
                              float feedback, bool freeze);

    void processResonestorBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                                 float* wetL, float* wetR,
                                 float position, float size, float pitch,
                                 float density, float texture,
                                 float feedback, bool freeze);

    void processBeatRepeatBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                                 float* wetL, float* wetR,
                                 float position, float size, float pitch,
                                 float density, float texture,
                                 float feedback, bool freeze);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (CloudLikeGranularProcessor)
};
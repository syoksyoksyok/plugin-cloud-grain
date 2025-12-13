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

    bool acceptsMidi() const override      { return true; }  // Accept MIDI for TRIG input
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

    // LED indicators for tempo visualization (public for UI access)
    std::atomic<bool> baseTempoBlink { false };  // LED 1: Blinks at base tempo (Auto mode only)
    std::atomic<bool> trigRateBlink { false };   // LED 2: Blinks at TRIG RATE tempo or MIDI
    std::atomic<bool> midiNoteHeld { false };    // True while MIDI note is held (Manual mode)

    // TRIG system state (public for UI access)
    std::atomic<bool> triggerReceived { false };  // Set when MIDI note or tempo trigger occurs

    // Host BPM (public for UI access in Auto mode)
    std::atomic<float> hostBPM { 0.0f };  // BPM from DAW (Auto mode)

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
        MODE_PITCH_SHIFTER = 1,  // Clouds: WSOLA-based pitch shifter/time stretcher
        MODE_LOOPING = 2,
        MODE_SPECTRAL = 3,
        MODE_OLIVERB = 4,        // Parasites: Creative reverb mode
        MODE_RESONESTOR = 5,     // Parasites: Polyphonic resonator (Karplus-Strong)
        MODE_BEAT_REPEAT = 6,    // Parasites: Beat repeat/stutter effect
        MODE_SPECTRAL_CLOUDS = 7 // SuperParasites: Spectral clouds with random band filtering
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

    // OPTIMIZATION: Memory alignment for SIMD (32-byte for AVX)
    alignas(32) std::array<float, SINE_TABLE_SIZE> sineLUT;
    alignas(32) std::array<float, SINE_TABLE_SIZE> cosineLUT;
    alignas(32) std::array<float, HANN_WINDOW_SIZE> hannWindowLUT;
    alignas(32) std::array<float, PITCH_LUT_SIZE> pitchRatioLUT;

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
    int    bufferSizeMask = 0;  // OPTIMIZATION: Bit mask for fast modulo (bufferSize - 1)
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
    std::atomic<int> detectedPeriod { 0 };
    std::atomic<int> correlatorUpdateCounter { 0 };

    // Pitch Shifter state (WSOLA-based with dual windows and correlator)
    struct WSOLAWindow {
        double readPos = 0.0;           // Current read position
        int hopCounter = 0;             // Samples until next window
        int windowSize = 2048;          // Current window size
        bool active = false;            // Window is playing
        bool needsRegeneration = false; // Needs new position calculation
    };
    WSOLAWindow wsolaWindows[2];        // Two overlapping windows
    int wsolaCurrentWindow = 0;         // Currently active window index

    // WSOLA envelope for smooth trigger response (Clouds-style)
    float wsolaEnvPhase = 1.0f;         // Envelope phase (0.0 = start, 1.0 = fully open)
    float wsolaEnvIncrement = 0.0f;     // Envelope increment per sample
    int wsolaElapsed = 0;               // Samples since last trigger

    // WSOLA parameters
    double pitchShifterReadPos = 0.0;   // Legacy read position (for compatibility)
    int pitchShifterWindowSize = 2048;
    int pitchShifterSearchWindow = 512;

    // Looping mode state (Clouds-style with improvements)
    double loopReadPos = 0.0;
    int loopStartPos = 0;
    int loopEndPos = 0;
    int loopLength = 0;
    int previousLoopLength = 0;  // Track changes to prevent clicks

    // Option 1: Loop boundary crossfade
    static constexpr int loopCrossfadeLength = 128;  // Samples for crossfade
    bool loopCrossfadeActive = false;

    // Option 2: TRIG loop capture with envelope (Clouds-style)
    std::vector<float> loopCaptureBufferL;
    std::vector<float> loopCaptureBufferR;
    int loopCaptureLength = 0;
    bool loopUseCaptured = false;  // Use captured loop or live buffer
    float loopEnvPhase = 1.0f;     // Envelope phase (0.0 = start, 1.0 = fully open)
    float loopEnvIncrement = 0.0f; // Envelope increment per sample

    // Option 3: TEXTURE feedback filtering (Clouds-style)
    struct LoopFilter {
        float previousL = 0.0f;
        float previousR = 0.0f;
    };
    LoopFilter loopFeedbackFilter;

    // Option 4: DENSITY multi-tap delay (SuperParasites-style)
    struct LoopTap {
        double readPos = 0.0;
        float panL = 0.0f;
        float panR = 0.0f;
    };
    static constexpr int maxLoopTaps = 4;
    std::array<LoopTap, maxLoopTaps> loopTaps;

    // Spectral mode state (FFT-based using juce::dsp)
    static constexpr int fftOrder = 11;
    static constexpr int fftSize = 1 << fftOrder;  // 2048
    juce::dsp::FFT forwardFFT { fftOrder };
    alignas(32) std::array<float, fftSize * 2> fftDataL;
    alignas(32) std::array<float, fftSize * 2> fftDataR;
    int spectralHopCounter = 0;    // Counter for hop size (triggers FFT every hopSize samples)
    int spectralBlockSize = 0;     // Accumulated samples in current block
    int spectralOutputPos = 0;     // Output read position
    alignas(32) std::array<float, fftSize * 2> spectralOutputL;  // Overlap-add buffer (needs 2x size)
    alignas(32) std::array<float, fftSize * 2> spectralOutputR;

    // OPTIMIZATION: Reusable buffers for spectral processing (avoid stack allocation)
    alignas(32) std::array<float, fftSize * 2> spectralShiftedL;
    alignas(32) std::array<float, fftSize * 2> spectralShiftedR;

    // Spectral mode improvements (Clouds/SuperParasites-style)
    // Option 4: TRIG spectral freeze
    alignas(32) std::array<float, fftSize * 2> spectralFrozenL;
    alignas(32) std::array<float, fftSize * 2> spectralFrozenR;
    bool spectralFrozen = false;
    float spectralFreezeAmount = 0.0f;

    // Option 1: TEXTURE effects (blur/scramble/phase)
    alignas(32) std::array<float, fftSize> spectralRandomPhase;  // Random phase table
    alignas(32) std::array<int, fftSize> spectralScrambleMap;    // Bin scramble mapping

    // Option 2: DENSITY band masking
    alignas(32) std::array<float, 32> spectralBandMask;  // Max 32 bands
    int spectralBandUpdateCounter = 0;  // Update band mask periodically

    // Option 5: Phase vocoder (previous phase for interpolation)
    alignas(32) std::array<float, fftSize> spectralPrevPhaseL;
    alignas(32) std::array<float, fftSize> spectralPrevPhaseR;
    alignas(32) std::array<float, fftSize> spectralPhaseAccumL;  // Accumulated phase
    alignas(32) std::array<float, fftSize> spectralPhaseAccumR;

    // Oliverb mode state (Multi-tap reverb with modulation - Clouds/SuperParasites-style)
    struct OliverbTap
    {
        std::vector<float> buffer;
        int writePos = 0;
        int bufferSizeMask = 0;  // OPTIMIZATION: Bit mask for power-of-2 buffer
        float modPhase = 0.0f;
        float modDepth = 0.0f;

        // Option 2: Tone filtering state
        float prevSampleL = 0.0f;
        float prevSampleR = 0.0f;

        // Option 5: Original tap time for room size scaling
        float originalTapTime = 0.0f;
    };
    static constexpr int maxOliverbTaps = 16;  // Extended from 8 to 16 for density control
    std::array<OliverbTap, maxOliverbTaps> oliverbTaps;

    // Option 4: TRIG reverb freeze
    bool oliverbFrozen = false;
    float oliverbFreezeEnvelope = 1.0f;

    // Option 1: Allpass diffusers (Clouds-style, 4 stages per channel)
    SimpleAllpass oliverbDiffuserL1, oliverbDiffuserL2, oliverbDiffuserL3, oliverbDiffuserL4;
    SimpleAllpass oliverbDiffuserR1, oliverbDiffuserR2, oliverbDiffuserR3, oliverbDiffuserR4;

    // Resonestor mode state (Karplus-Strong resonators - Clouds/SuperParasites-style)
    static constexpr int maxResonators = 12;
    struct Resonator
    {
        std::vector<float> delayLine;
        int writePos = 0;
        int delayLineMask = 0;  // OPTIMIZATION: Bit mask for power-of-2 delay line
        float feedback = 0.0f;
        float brightness = 0.5f;
        bool active = false;

        // Option 2: Stereo spread (panning)
        float panL = 0.0f;
        float panR = 0.0f;

        // Option 4: Two-pole lowpass filter state
        float z1 = 0.0f;
        float z2 = 0.0f;

        // Option 5: Modal synthesis (base frequency)
        float frequency = 0.0f;
    };
    std::array<Resonator, maxResonators> resonators;

    // Option 3: Chord presets for TRIG switching
    static constexpr int numChordPresets = 6;
    int resonestorCurrentChord = 0;

    // Resonestor: Burst envelope (Clouds-style smooth envelope)
    float resonestorBurstEnvelope = 0.0f;
    float resonestorBurstDecay = 0.9995f;
    bool resonestorPreviousTrigger = false;  // For trigger edge detection

    // Beat Repeat mode state (SuperParasites Kammerl-style)
    struct BeatRepeatState
    {
        std::vector<float> captureBufferL;
        std::vector<float> captureBufferR;
        int captureLength = 0;
        float repeatPos = 0.0f;  // Sub-sample accurate playback position
        bool isPlaying = false;

        // === Trigger interval measurement (Kammerl-style) ===
        int numSamplesSinceTrigger = 0;      // Samples since last trigger
        int sliceSizeSamples = 0;            // Current slice size based on trigger interval
        int numRemainingSamplesInSlice = 0;  // Countdown for current slice
        bool synchronized = false;            // True when trigger interval is valid

        // === Clock divider (1x to 8x) ===
        static constexpr int numClockDividers = 6;
        std::array<int, numClockDividers> clockDividerValues = {1, 2, 4, 8, 16, 32};
        int clockDividerIndex = 0;

        // === Pitch modes (Kammerl-style) ===
        enum PitchMode {
            PITCH_FIXED = 0,      // Constant speed
            PITCH_DECREASING,     // Slows down over slice
            PITCH_INCREASING,     // Speeds up over slice
            PITCH_SCRATCH,        // Sinusoidal speed modulation
            PITCH_REVERSED        // Reverse playback
        };
        PitchMode pitchMode = PITCH_FIXED;
        float basePitchSpeed = 1.0f;  // Base playback speed from PITCH knob

        // === Loop points (Kammerl-style quantized positions) ===
        static constexpr int numLoopPositions = 9;
        std::array<float, numLoopPositions> loopPositions = {0.0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f, 1.0f};
        float loopBeginPercent = 0.0f;
        float loopEndPercent = 1.0f;
        int loopBeginSamples = 0;
        int loopEndSamples = 0;

        // === Size modulation (shrinking loop) ===
        bool sizeModulationEnabled = false;
        float sizeModulationAmount = 0.0f;  // How much to shrink

        // === Slice modulation patterns ===
        enum SlicePattern {
            SLICE_SEQUENTIAL = 0,  // 1, 2, 3, 4...
            SLICE_ALTERNATING,     // 1, 2, 1, 2...
            SLICE_RANDOM,          // Random selection
            SLICE_PINGPONG         // 1, 2, 3, 2, 1...
        };
        SlicePattern slicePattern = SLICE_SEQUENTIAL;
        int numSlices = 1;
        int currentSlice = 0;
        int sliceDirection = 1;  // For ping-pong

        // === Probability-based trigger ===
        float triggerProbability = 1.0f;  // 0.0 to 1.0

        // === Envelope shaping ===
        float envelopePhase = 0.0f;
        float attackTime = 0.002f;   // 2ms attack
        float releaseTime = 0.005f;  // 5ms release
        std::vector<float> crossfadeBufferL;
        std::vector<float> crossfadeBufferR;
        bool hasPreviousCapture = false;

        // === Stutter/capture state ===
        float stutterPhase = 0.0f;
        bool isCapturing = false;

        // === Slice order for pattern playback ===
        std::array<int, 16> sliceOrder = {};

        // === Trigger mode and hold state ===
        int triggerMode = 0;       // 0: Repeat, 1: Gate, etc.
        bool triggerHeld = false;
        float triggerHoldTime = 0.0f;

        // === Alternating direction (bidirectional loop) ===
        bool alternatingEnabled = false;
        bool playingForward = true;

        // RNG for probability and slice randomization
        std::mt19937 rng{std::random_device{}()};
    };
    BeatRepeatState beatRepeat;

    // Spectral Clouds mode state (SuperParasites-style)
    static constexpr int maxSpectralCloudsBands = 64;  // Maximum frequency bands
    struct SpectralCloudsState
    {
        // Frequency band gain control
        alignas(32) std::array<float, maxSpectralCloudsBands> currentGain;  // Current band gains
        alignas(32) std::array<float, maxSpectralCloudsBands> targetGain;   // Target band gains

        // Frozen spectral data (magnitude only, phase is randomized)
        alignas(32) std::array<float, fftSize> frozenMagnitudeL;
        alignas(32) std::array<float, fftSize> frozenMagnitudeR;

        // Phase arrays for randomization
        alignas(32) std::array<float, fftSize> phaseL;
        alignas(32) std::array<float, fftSize> phaseR;

        // Processing state
        bool frozen = false;
        int numFreqBands = 16;  // Current number of frequency bands (4-64)
        float parameterLowpass = 0.1f;  // Smoothing coefficient for gain changes
        bool previousTrigger = false;  // For trigger edge detection
    };
    SpectralCloudsState spectralClouds;

    std::atomic<float> lastRandomizeValue { 0.0f };
    std::atomic<int> previousMode { 0 };  // Track mode changes for cleanup

    // TRIG system internal state (private)
    double tempoSyncPhase = 0.0;  // Phase accumulator for tempo sync triggers
    double baseTempoPhase = 0.0;  // Phase accumulator for base tempo LED

    // Clouds-style density control
    float grainRatePhasor = 0.0f;
    std::atomic<int> numActiveGrains { 0 };
    float smoothedGain = 1.0f;

    void launchGrains (int numToLaunch, int channel,
                       float positionParam, float sizeParam,
                       float pitchSemis, float textureParam,
                       float stereoSpread, int periodHint = 0);

    void initializeGrain (Grain& g, int channel, double baseStart, double durationSamps,
                         double pitchRatio, float positionJitter, float spreadWidth);

    float getSampleFromRing (int channel, double index) const;
    float getGrainEnvelope (double t, double duration, float textureParam) const;

    // Clouds-style helper functions
    float fastInverseSqrt (float number) const;
    float computeOverlap (float density) const;

    // Pitch Shifter (WSOLA) helper: find best matching segment
    int findBestMatch (const float* reference, const float* searchBuffer,
                       int searchLength, int windowSize);

    void randomizeParameters();

    // ========== REFACTORED: Common Helper Functions ==========
    // TRIG rate to note value conversion (eliminates code duplication)
    static double calculateNoteValueFromTrigRate(float trigRate);

    // Ring buffer write with feedback (inline for performance)
    inline void writeToRingBuffer(float inL, float inR, float fbL, float fbR, bool freeze)
    {
        if (!freeze && bufferSize > 0)
        {
            ringBuffer.setSample(0, writeHead, inL + fbL);
            ringBuffer.setSample(1, writeHead, inR + fbR);
            writeHead = (writeHead + 1) & bufferSizeMask;
        }
    }

    // Trigger detection and clear (inline for performance)
    inline bool checkAndClearTrigger(int sampleIndex)
    {
        if (sampleIndex == 0 && triggerReceived.load())
        {
            triggerReceived.store(false);
            return true;
        }
        return false;
    }

    // ========== OPTIMIZATION: Block Processing Functions ==========
    // Mode-specific block processing for better cache locality and reduced branching
    void processGranularBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                               float* wetL, float* wetR,
                               float position, float size, float pitch, float density,
                               float texture, float spread, float feedback, bool freeze);

    void processPitchShifterBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                                   float* wetL, float* wetR,
                                   float position, float size, float pitch,
                                   float feedback, bool freeze);

    void processLoopingBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                              float* wetL, float* wetR,
                              float position, float size, float pitch,
                              float density, float texture,
                              float feedback, bool freeze);

    void processSpectralBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                               float* wetL, float* wetR,
                               float position, float size, float pitch,
                               float density, float texture,
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

    void processSpectralCloudsBlock (juce::AudioBuffer<float>& buffer, int numSamples,
                                     float* wetL, float* wetR,
                                     float position, float size, float pitch,
                                     float density, float texture,
                                     float feedback, bool freeze);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (CloudLikeGranularProcessor)
};
// PluginEditor.h
#pragma once

#include <JuceHeader.h>
#include "PluginProcessor.h"

//==============================================================================
// Centralized Color Configuration
// すべてのGUI要素の色をここで一括管理
struct UIColors
{
    // Knob colors by position (row, column)
    struct KnobColors
    {
        juce::Colour outline;
        juce::Colour indicator;
        juce::Colour centerDot;
        juce::Colour tickMarks;
    };

    // Row 1 knob colors
    KnobColors position      { juce::Colour (0xFFC94C63), juce::Colour (0xFFC94C63), juce::Colour (0xFFC94C63), juce::Colour (224, 224, 224) };  // Position
    KnobColors density       { juce::Colour (0xFFC94C63), juce::Colour (0xFFC94C63), juce::Colour (0xFFC94C63), juce::Colour (224, 224, 224) };  // Density
    KnobColors size          { juce::Colour (0xFF479FA5), juce::Colour (0xFF479FA5), juce::Colour (0xFF479FA5), juce::Colour (224, 224, 224) };  // Size
    KnobColors texture       { juce::Colour (0xFF479FA5), juce::Colour (0xFF479FA5), juce::Colour (0xFF479FA5), juce::Colour (224, 224, 224) };  // Texture
    KnobColors pitch         { juce::Colour (102, 102, 102), juce::Colour (102, 102, 102), juce::Colour (102, 102, 102), juce::Colour (224, 224, 224) };  // Pitch

    // Row 2 knob colors
    KnobColors spread        { juce::Colour (102, 102, 102), juce::Colour (102, 102, 102), juce::Colour (102, 102, 102), juce::Colour (224, 224, 224) };  // Spread
    KnobColors feedback      { juce::Colour (102, 102, 102), juce::Colour (102, 102, 102), juce::Colour (102, 102, 102), juce::Colour (224, 224, 224) };  // Feedback
    KnobColors reverb        { juce::Colour (102, 102, 102), juce::Colour (102, 102, 102), juce::Colour (102, 102, 102), juce::Colour (224, 224, 224) };  // Reverb
    KnobColors mix           { juce::Colour (102, 102, 102), juce::Colour (102, 102, 102), juce::Colour (102, 102, 102), juce::Colour (224, 224, 224) };  // Mix
    KnobColors mode          { juce::Colour (0xFFDAC1AF), juce::Colour (0xFFDAC1AF), juce::Colour (0xFFDAC1AF), juce::Colour (224, 224, 224) };  // Mode

    // Row 3 knob colors
    KnobColors trigRate      { juce::Colour (102, 102, 102), juce::Colour (102, 102, 102), juce::Colour (102, 102, 102), juce::Colour (224, 224, 224) };  // TRIG Rate

    // Button colors
    juce::Colour buttonText              { 26, 26, 26 };
    juce::Colour buttonBackground        { 250, 250, 250 };
    juce::Colour buttonBackgroundPressed { 224, 224, 224 };
    juce::Colour freezeTextOn            { 0xFFBAD64F };  // Freeze ON color
    juce::Colour freezeTextOff           { 26, 26, 26 };

    // Label colors
    juce::Colour modeLabel { 52, 73, 94 };  // Ink blue
    juce::Colour knobLabel { 102, 102, 102 };

    // Background
    juce::Colour background { 255, 255, 255 };
};

// Global color configuration instance
static UIColors uiColors;

//==============================================================================
// E-Paper LookAndFeel for rotary knobs with tick marks
class EPaperLookAndFeel : public juce::LookAndFeel_V4
{
public:
    UIColors::KnobColors* knobColors = nullptr;  // Pointer to current knob's colors
    bool forceMaxPosition = false;  // When true, indicator shows 100% position (for Kill Dry)

    EPaperLookAndFeel()
    {
        setColour (juce::Slider::thumbColourId, juce::Colour::fromRGB (26, 26, 26));
        setColour (juce::Slider::rotarySliderFillColourId, juce::Colour::fromRGB (26, 26, 26));
        setColour (juce::Slider::rotarySliderOutlineColourId, juce::Colour::fromRGB (224, 224, 224));
    }

    void drawRotarySlider (juce::Graphics& g, int x, int y, int width, int height,
                          float sliderPos, float rotaryStartAngle, float rotaryEndAngle,
                          juce::Slider& slider) override
    {
        // Override slider position if forceMaxPosition is set (Kill Dry active)
        float effectiveSliderPos = forceMaxPosition ? 1.0f : sliderPos;
        auto radius = juce::jmin (width / 2, height / 2) - 4.0f;
        auto centreX = x + width * 0.5f;
        auto centreY = y + height * 0.5f;
        auto rx = centreX - radius;
        auto ry = centreY - radius;
        auto rw = radius * 2.0f;
        // Standard angle calculation with Y-axis negated in rendering
        auto angle = rotaryStartAngle + effectiveSliderPos * (rotaryEndAngle - rotaryStartAngle);

        // Get colors for this knob (use default if not set)
        juce::Colour outlineColor = knobColors ? knobColors->outline : juce::Colour (102, 102, 102);
        juce::Colour indicatorColor = knobColors ? knobColors->indicator : juce::Colour (102, 102, 102);
        juce::Colour centerDotColor = knobColors ? knobColors->centerDot : juce::Colour (102, 102, 102);
        juce::Colour tickMarkColor = knobColors ? knobColors->tickMarks : juce::Colour (224, 224, 224);

        // Draw outer circle (outline) - use knob-specific color
        g.setColour (outlineColor);
        g.drawEllipse (rx, ry, rw, rw, 2.0f);

        // Draw tick marks
        g.setColour (tickMarkColor);
        for (int i = 0; i < 5; ++i)
        {
            auto tickAngle = rotaryStartAngle + i * (rotaryEndAngle - rotaryStartAngle) / 4.0f;
            auto tickRadius = radius - 2.0f;
            auto tickLength = 6.0f;

            auto tickX1 = centreX + tickRadius * std::cos (tickAngle - juce::MathConstants<float>::halfPi);
            auto tickY1 = centreY + tickRadius * std::sin (tickAngle - juce::MathConstants<float>::halfPi);
            auto tickX2 = centreX + (tickRadius - tickLength) * std::cos (tickAngle - juce::MathConstants<float>::halfPi);
            auto tickY2 = centreY + (tickRadius - tickLength) * std::sin (tickAngle - juce::MathConstants<float>::halfPi);

            g.drawLine (tickX1, tickY1, tickX2, tickY2, 2.0f);
        }

        // Draw center dot - use knob-specific color
        g.setColour (centerDotColor);
        g.fillEllipse (centreX - 3.0f, centreY - 3.0f, 6.0f, 6.0f);

        // Draw indicator line - use knob-specific color
        g.setColour (indicatorColor);
        auto pointerLength = radius - 8.0f;
        auto pointerThickness = 3.0f;
        juce::Path p;
        auto pointerX = centreX + pointerLength * std::cos (angle - juce::MathConstants<float>::halfPi);
        auto pointerY = centreY + pointerLength * std::sin (angle - juce::MathConstants<float>::halfPi);

        p.startNewSubPath (centreX, centreY);
        p.lineTo (pointerX, pointerY);
        g.strokePath (p, juce::PathStrokeType (pointerThickness, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
    }

    void drawButtonBackground (juce::Graphics& g, juce::Button& button, const juce::Colour& /*backgroundColour*/,
                              bool shouldDrawButtonAsHighlighted, bool shouldDrawButtonAsDown) override
    {
        auto bounds = button.getLocalBounds().toFloat();

        // E-Paper button background
        g.setColour (shouldDrawButtonAsDown ? uiColors.buttonBackgroundPressed : uiColors.buttonBackground);
        g.fillRect (bounds);

        // E-Paper button border
        g.setColour (uiColors.buttonText);
        g.drawRect (bounds, 2.0f);
    }

    void drawToggleButton (juce::Graphics& g, juce::ToggleButton& button,
                          bool shouldDrawButtonAsHighlighted, bool shouldDrawButtonAsDown) override
    {
        auto bounds = button.getLocalBounds().toFloat();
        auto fontSize = juce::jmin (15.0f, button.getHeight() * 0.4f);
        auto tickWidth = fontSize * 1.2f;

        // Draw checkbox
        auto checkboxBounds = bounds.removeFromLeft (tickWidth).reduced (2.0f);
        g.setColour (uiColors.buttonBackground);
        g.fillRect (checkboxBounds);

        g.setColour (uiColors.buttonText);
        g.drawRect (checkboxBounds, 2.0f);

        if (button.getToggleState())
        {
            auto tick = checkboxBounds.reduced (4.0f);
            g.setColour (uiColors.buttonText);
            g.fillRect (tick);
        }

        // Draw text (color is set by component's textColourId, not here)
        g.setColour (button.findColour (juce::ToggleButton::textColourId));
        g.setFont (juce::Font ("Courier New", fontSize, juce::Font::bold));
        g.drawText (button.getButtonText(), bounds.withTrimmedLeft (2.0f),
                   juce::Justification::centredLeft, true);
    }
};

//==============================================================================
class CloudLikeGranularEditor  : public juce::AudioProcessorEditor
                               , public juce::Timer
{
public:
    CloudLikeGranularEditor (CloudLikeGranularProcessor&);
    ~CloudLikeGranularEditor() override;

    void paint (juce::Graphics&) override;
    void resized() override;
    void timerCallback() override;
    void mouseDown (const juce::MouseEvent& event) override;
    void mouseMove (const juce::MouseEvent& event) override;

private:
    // ========== REFACTORED: timerCallback helper functions ==========
    void updateButtonStates(bool trigMode);
    void updateModeLabels(int mode);
    void updateTrigRateLabel(float trigRate);
    void updateKnobValueLabels(int mode, float position, float size, float pitch,
                                float density, float texture, float spread,
                                float feedback, float reverb, float mix);
    void updateLedIndicators();
    void updateBpmDisplay(bool trigMode);

    CloudLikeGranularProcessor& processor;

    std::unique_ptr<EPaperLookAndFeel> ePaperLookAndFeel;  // Custom LookAndFeel for buttons (managed lifetime)

    // Individual LookAndFeel instances for each knob (to support per-knob colors)
    std::unique_ptr<EPaperLookAndFeel> positionLookAndFeel;
    std::unique_ptr<EPaperLookAndFeel> densityLookAndFeel;
    std::unique_ptr<EPaperLookAndFeel> sizeLookAndFeel;
    std::unique_ptr<EPaperLookAndFeel> textureLookAndFeel;
    std::unique_ptr<EPaperLookAndFeel> pitchLookAndFeel;
    std::unique_ptr<EPaperLookAndFeel> spreadLookAndFeel;
    std::unique_ptr<EPaperLookAndFeel> feedbackLookAndFeel;
    std::unique_ptr<EPaperLookAndFeel> reverbLookAndFeel;
    std::unique_ptr<EPaperLookAndFeel> mixLookAndFeel;
    std::unique_ptr<EPaperLookAndFeel> modeLookAndFeel;
    std::unique_ptr<EPaperLookAndFeel> trigRateLookAndFeel;

    struct Knob
    {
        juce::Slider slider;
        juce::Label  label;
    };

    Knob modeKnob, positionKnob, sizeKnob, pitchKnob, densityKnob;
    Knob textureKnob, spreadKnob, feedbackKnob, reverbKnob, mixKnob;
    Knob trigRateKnob;  // 3rd row: TRIG rate/division control

    juce::ToggleButton trigModeButton { "Trig Mode" };  // Manual/Auto toggle
    juce::ToggleButton freezeButton  { "Freeze" };
    juce::TextButton randomButton { "Randomize" };
    juce::TextButton killDryButton { "Kill Dry" };  // Momentary: forces MIX to 100% while pressed
    juce::Label modeValueLabel;  // Displays current MODE name (Granular, PitchShft, etc.)
    juce::Label trigRateValueLabel;  // Displays current TRIG RATE division (1/16, 1/8T, etc.)
    juce::Label tapBpmLabel;  // Displays detected BPM from tap tempo (clickable for tap tempo)

    // Custom value labels for all knobs (displays meaningful text instead of raw numbers)
    juce::Label positionValueLabel;
    juce::Label sizeValueLabel;
    juce::Label pitchValueLabel;
    juce::Label densityValueLabel;
    juce::Label textureValueLabel;
    juce::Label spreadValueLabel;
    juce::Label feedbackValueLabel;
    juce::Label reverbValueLabel;
    juce::Label mixValueLabel;

    // LED indicators for tempo visualization
    bool baseTempoLedOn = false;   // LED 1: Base tempo (×1) indicator
    bool trigRateLedOn = false;    // LED 2: TRIG RATE tempo indicator
    int ledBlinkDuration = 0;      // Counter for LED blink duration (in timer ticks)
    int ledBlinkDuration2 = 0;     // Counter for second LED

    using SliderAttachment = juce::AudioProcessorValueTreeState::SliderAttachment;
    using ButtonAttachment = juce::AudioProcessorValueTreeState::ButtonAttachment;

    std::unique_ptr<SliderAttachment> modeAttachment;
    std::unique_ptr<SliderAttachment> positionAttachment;
    std::unique_ptr<SliderAttachment> sizeAttachment;
    std::unique_ptr<SliderAttachment> pitchAttachment;
    std::unique_ptr<SliderAttachment> densityAttachment;
    std::unique_ptr<SliderAttachment> textureAttachment;
    std::unique_ptr<SliderAttachment> spreadAttachment;
    std::unique_ptr<SliderAttachment> feedbackAttachment;
    std::unique_ptr<SliderAttachment> reverbAttachment;
    std::unique_ptr<SliderAttachment> mixAttachment;
    std::unique_ptr<SliderAttachment> trigRateAttachment;
    std::unique_ptr<ButtonAttachment> trigModeAttachment;
    std::unique_ptr<ButtonAttachment> freezeAttachment;

    void setupKnob (Knob& k, const juce::String& name, EPaperLookAndFeel* lookAndFeel, bool showTextBox = true);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (CloudLikeGranularEditor)
};
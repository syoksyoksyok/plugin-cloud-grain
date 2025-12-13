// PluginEditor.h
#pragma once

#include <JuceHeader.h>
#include "PluginProcessor.h"
#include <array>
#include <unordered_map>

//==============================================================================
// Knob identifiers for array-based management
enum class KnobId
{
    Position = 0,
    Density,
    Size,
    Texture,
    Pitch,
    Spread,
    Feedback,
    Reverb,
    Mix,
    Mode,
    TrigRate,
    NumKnobs
};
constexpr int kNumKnobs = static_cast<int>(KnobId::NumKnobs);

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

    // Default knob color (gray)
    static inline const juce::Colour defaultGray { 102, 102, 102 };
    static inline const juce::Colour tickGray { 224, 224, 224 };

    // Knob colors indexed by KnobId
    std::array<KnobColors, kNumKnobs> knobColors {{
        // Row 1
        { juce::Colour (0xFFC94C63), juce::Colour (0xFFC94C63), juce::Colour (0xFFC94C63), tickGray },  // Position
        { juce::Colour (0xFFC94C63), juce::Colour (0xFFC94C63), juce::Colour (0xFFC94C63), tickGray },  // Density
        { juce::Colour (0xFF479FA5), juce::Colour (0xFF479FA5), juce::Colour (0xFF479FA5), tickGray },  // Size
        { juce::Colour (0xFF479FA5), juce::Colour (0xFF479FA5), juce::Colour (0xFF479FA5), tickGray },  // Texture
        { defaultGray, defaultGray, defaultGray, tickGray },  // Pitch
        // Row 2
        { defaultGray, defaultGray, defaultGray, tickGray },  // Spread
        { defaultGray, defaultGray, defaultGray, tickGray },  // Feedback
        { defaultGray, defaultGray, defaultGray, tickGray },  // Reverb
        { defaultGray, defaultGray, defaultGray, tickGray },  // Mix
        { juce::Colour (0xFFDAC1AF), juce::Colour (0xFFDAC1AF), juce::Colour (0xFFDAC1AF), tickGray },  // Mode
        // Row 3
        { defaultGray, defaultGray, defaultGray, tickGray },  // TrigRate
    }};

    // Button colors
    juce::Colour buttonText              { 26, 26, 26 };
    juce::Colour buttonBackground        { 250, 250, 250 };
    juce::Colour buttonBackgroundPressed { 224, 224, 224 };
    juce::Colour freezeTextOn            { 0xFFBAD64F };
    juce::Colour freezeTextOff           { 26, 26, 26 };
    juce::Colour momentaryOnColor        { 0xFFC94C63 };  // Position knob color for Kill Dry/Wet

    // Label colors
    juce::Colour knobLabel { 102, 102, 102 };

    // Background
    juce::Colour background { 255, 255, 255 };

    // Get knob colors by ID
    const KnobColors& getKnobColors(KnobId id) const { return knobColors[static_cast<int>(id)]; }
};

// Global color configuration instance
inline UIColors uiColors;

//==============================================================================
// Fixed UI Size Configuration (100% scale base values)
namespace UISize
{
    constexpr int baseWidth  = 640;
    constexpr int baseHeight = 440;

    constexpr int windowMargin = 10;
    constexpr int knobPadding  = 8;

    constexpr int numColumns = 5;
    constexpr int numRows = 3;
    constexpr int knobCellWidth = 124;
    constexpr int knobRowHeight = 110;
    constexpr int row3Height = 100;

    constexpr int knobDiameter = 60;

    constexpr int labelHeight = 18;
    constexpr int labelFontSize = 11;
    constexpr int knobLabelOffsetY = 25;

    constexpr int buttonWidth = 83;
    constexpr int buttonHeight = 36;
    constexpr int buttonRowHeight = 50;
    constexpr int buttonPadding = 3;

    constexpr int bpmDisplaySize = 40;
    constexpr int bpmFontSize = 10;

    constexpr int ledDiameter = 40;
    constexpr int ledSpacing = 24;
    constexpr int ledLabelFontSize = 9;

    constexpr float scales[] = { 1.0f, 1.25f, 1.5f, 2.0f };
    constexpr int numScales = 4;
}

//==============================================================================
// E-Paper LookAndFeel for rotary knobs with tick marks
class EPaperLookAndFeel : public juce::LookAndFeel_V4
{
public:
    // Map slider pointer to KnobId for color lookup
    std::unordered_map<juce::Slider*, KnobId> sliderToKnobId;

    // Mix knob special states
    bool forceMaxPosition = false;
    bool forceMinPosition = false;

    EPaperLookAndFeel()
    {
        setColour (juce::Slider::thumbColourId, juce::Colour::fromRGB (26, 26, 26));
        setColour (juce::Slider::rotarySliderFillColourId, juce::Colour::fromRGB (26, 26, 26));
        setColour (juce::Slider::rotarySliderOutlineColourId, juce::Colour::fromRGB (224, 224, 224));
    }

    void registerSlider(juce::Slider* slider, KnobId id)
    {
        sliderToKnobId[slider] = id;
    }

    void drawRotarySlider (juce::Graphics& g, int x, int y, int width, int height,
                          float sliderPos, float rotaryStartAngle, float rotaryEndAngle,
                          juce::Slider& slider) override
    {
        float effectiveSliderPos = forceMaxPosition ? 1.0f : (forceMinPosition ? 0.0f : sliderPos);
        auto radius = juce::jmin (width / 2, height / 2) - 4.0f;
        auto centreX = x + width * 0.5f;
        auto centreY = y + height * 0.5f;
        auto rx = centreX - radius;
        auto ry = centreY - radius;
        auto rw = radius * 2.0f;
        auto angle = rotaryStartAngle + effectiveSliderPos * (rotaryEndAngle - rotaryStartAngle);

        // Get colors for this knob from map
        auto it = sliderToKnobId.find(&slider);
        const auto& colors = (it != sliderToKnobId.end())
            ? uiColors.getKnobColors(it->second)
            : uiColors.getKnobColors(KnobId::Mix);  // Default fallback

        // Draw outer circle
        g.setColour (colors.outline);
        g.drawEllipse (rx, ry, rw, rw, 2.0f);

        // Draw tick marks
        g.setColour (colors.tickMarks);
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

        // Draw center dot
        g.setColour (colors.centerDot);
        g.fillEllipse (centreX - 3.0f, centreY - 3.0f, 6.0f, 6.0f);

        // Draw indicator line
        g.setColour (colors.indicator);
        auto pointerLength = radius - 8.0f;
        juce::Path p;
        auto pointerX = centreX + pointerLength * std::cos (angle - juce::MathConstants<float>::halfPi);
        auto pointerY = centreY + pointerLength * std::sin (angle - juce::MathConstants<float>::halfPi);
        p.startNewSubPath (centreX, centreY);
        p.lineTo (pointerX, pointerY);
        g.strokePath (p, juce::PathStrokeType (3.0f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
    }

    void drawButtonBackground (juce::Graphics& g, juce::Button& button, const juce::Colour&,
                              bool, bool shouldDrawButtonAsDown) override
    {
        auto bounds = button.getLocalBounds().toFloat();
        g.setColour (shouldDrawButtonAsDown ? uiColors.buttonBackgroundPressed : uiColors.buttonBackground);
        g.fillRect (bounds);
        g.setColour (uiColors.getKnobColors(KnobId::Mix).outline);
        g.drawRect (bounds, 1.0f);
    }

    void drawToggleButton (juce::Graphics& g, juce::ToggleButton& button,
                          bool, bool shouldDrawButtonAsDown) override
    {
        auto bounds = button.getLocalBounds().toFloat();
        auto fontSize = juce::jmin (15.0f, button.getHeight() * 0.4f);

        g.setColour (shouldDrawButtonAsDown ? uiColors.buttonBackgroundPressed : uiColors.buttonBackground);
        g.fillRect (bounds);
        g.setColour (uiColors.getKnobColors(KnobId::Mix).outline);
        g.drawRect (bounds, 1.0f);

        g.setColour (button.findColour (juce::ToggleButton::textColourId));
        g.setFont (juce::Font ("Courier New", fontSize, juce::Font::bold));
        g.drawText (button.getButtonText(), bounds, juce::Justification::centred, true);
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
    void mouseUp (const juce::MouseEvent& event) override;
    void mouseMove (const juce::MouseEvent& event) override;

    void setScale (float newScale);
    float getScale() const { return uiScale; }

private:
    // ========== timerCallback helper functions ==========
    void updateButtonStates(bool trigMode);
    void updateModeLabels(int mode);
    void updateTrigRateLabel(float trigRate);
    void updateKnobValueLabels(int mode, float position, float size, float pitch,
                                float density, float texture, float spread,
                                float feedback, float reverb, float mix);
    void updateLedIndicators();
    void updateBpmDisplay(bool trigMode);

    // ========== Setup helper functions ==========
    void setupKnob(KnobId id, const juce::String& name);
    void setupMomentaryButton(juce::TextButton& button, const juce::String& paramId);
    void setupValueLabel(juce::Label& label);

    CloudLikeGranularProcessor& processor;

    // Single LookAndFeel instance (uses slider map for colors)
    std::unique_ptr<EPaperLookAndFeel> lookAndFeel;

    // Knob structure
    struct Knob
    {
        juce::Slider slider;
        juce::Label  label;
    };

    // Array-based knob management
    std::array<Knob, kNumKnobs> knobs;
    std::array<juce::Label, kNumKnobs> valueLabels;

    // Knob names and parameter IDs
    static constexpr std::array<const char*, kNumKnobs> knobNames = {{
        "Position", "Density", "Size", "Texture", "Pitch",
        "Spread", "Feedback", "Reverb", "Mix", "Mode", "Trig Rate"
    }};
    static constexpr std::array<const char*, kNumKnobs> paramIds = {{
        "position", "density", "size", "texture", "pitch",
        "spread", "feedback", "reverb", "mix", "mode", "trigRate"
    }};

    // Buttons
    juce::ToggleButton trigModeButton { "Trig Mode" };
    juce::ToggleButton freezeButton { "Freeze" };
    juce::TextButton randomButton { "Rnd" };
    juce::TextButton killDryButton { "KillDry" };
    juce::TextButton killWetButton { "KillWet" };
    juce::TextButton tapButton;

    // Labels
    juce::Label tapBpmLabel;

    // LED state
    bool baseTempoLedOn = false;
    bool trigRateLedOn = false;
    int ledBlinkDuration = 0;
    int ledBlinkDuration2 = 0;

    // Attachments
    using SliderAttachment = juce::AudioProcessorValueTreeState::SliderAttachment;
    using ButtonAttachment = juce::AudioProcessorValueTreeState::ButtonAttachment;

    std::array<std::unique_ptr<SliderAttachment>, kNumKnobs> sliderAttachments;
    std::unique_ptr<ButtonAttachment> trigModeAttachment;
    std::unique_ptr<ButtonAttachment> freezeAttachment;

    // UI Scale
    float uiScale = 1.0f;
    int scaled(int baseValue) const { return static_cast<int>(baseValue * uiScale); }
    float scaledF(float baseValue) const { return baseValue * uiScale; }

    // Knob accessors for convenience
    Knob& knob(KnobId id) { return knobs[static_cast<int>(id)]; }
    juce::Label& valueLabel(KnobId id) { return valueLabels[static_cast<int>(id)]; }

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (CloudLikeGranularEditor)
};
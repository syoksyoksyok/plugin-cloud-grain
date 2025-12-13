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

    // Toggle switch colors
    juce::Colour toggleTrackOff          { 230, 230, 230 };
    juce::Colour toggleTrackOn           { 102, 102, 102 };
    juce::Colour toggleThumb             { 255, 255, 255 };
    juce::Colour toggleThumbBorder       { 180, 180, 180 };
    juce::Colour toggleLabelOff          { 160, 160, 160 };
    juce::Colour toggleLabelOn           { 26, 26, 26 };

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
// Direct specification approach - knob size and gaps defined separately
namespace UISize
{
    // Window
    constexpr int windowMargin = 20;

    // Knob layout (direct specification)
    constexpr int knobDiameter = 50;
    constexpr int knobGap = 32;           // Horizontal gap between knobs
    constexpr int knobRowGap = 20;        // Vertical gap between rows
    constexpr int knobLabelHeight = 20;   // Space for label below knob

    // Calculated row height: knob + label space + some padding
    constexpr int knobRowHeight = knobDiameter + knobLabelHeight + 20;

    // Buttons
    constexpr int buttonWidth = 70;
    constexpr int buttonHeight = 30;
    constexpr int buttonPadding = 4;

    // Toggle switch dimensions
    constexpr int toggleWidth = 100;
    constexpr int toggleHeight = 26;
    constexpr int toggleThumbSize = 20;
    constexpr int toggleTrackHeight = 18;

    // Window size (calculated from layout)
    // Knob row width: 5 knobs + 4 gaps = 378px
    constexpr int knobRowWidth = (knobDiameter * 5) + (knobGap * 4);
    // Button row width: toggle + 4 buttons + padding = 412px
    constexpr int buttonRowTotalWidth = toggleWidth + (buttonWidth * 4) + (buttonPadding * 8);
    // Use larger of the two + margins
    constexpr int baseWidth = windowMargin * 2 + (buttonRowTotalWidth > knobRowWidth ? buttonRowTotalWidth : knobRowWidth);  // = 432

    // Height: margin + 2 rows + row3 + button row + margin
    constexpr int row3Height = 80;
    constexpr int buttonRowHeight = 50;
    constexpr int baseHeight = windowMargin + (knobRowHeight * 2) + row3Height + buttonRowHeight + windowMargin;  // = 330

    // Labels
    constexpr int labelHeight = 18;
    constexpr int labelFontSize = 11;

    // BPM/TAP display
    constexpr int bpmDisplaySize = 36;
    constexpr int bpmFontSize = 10;

    // LED
    constexpr int ledDiameter = 36;
    constexpr int ledSpacing = 20;
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

        // Check if this is a toggle switch style button (by component ID)
        if (button.getComponentID() == "toggleSwitch")
        {
            drawToggleSwitch (g, button, bounds);
            return;
        }

        // Standard toggle button style
        auto fontSize = juce::jmin (15.0f, button.getHeight() * 0.4f);

        g.setColour (shouldDrawButtonAsDown ? uiColors.buttonBackgroundPressed : uiColors.buttonBackground);
        g.fillRect (bounds);
        g.setColour (uiColors.getKnobColors(KnobId::Mix).outline);
        g.drawRect (bounds, 1.0f);

        g.setColour (button.findColour (juce::ToggleButton::textColourId));
        g.setFont (juce::Font ("Courier New", fontSize, juce::Font::bold));
        g.drawText (button.getButtonText(), bounds, juce::Justification::centred, true);
    }

    void drawToggleSwitch (juce::Graphics& g, juce::ToggleButton& button, juce::Rectangle<float> bounds)
    {
        bool isOn = button.getToggleState();
        float height = bounds.getHeight();
        float trackHeight = height * 0.7f;
        float thumbSize = height * 0.85f;
        float trackY = bounds.getCentreY() - trackHeight / 2.0f;
        float cornerRadius = trackHeight / 2.0f;

        // Calculate track bounds (center portion for the actual track)
        float labelWidth = bounds.getWidth() * 0.32f;
        float trackWidth = bounds.getWidth() - labelWidth * 2.0f;
        float trackX = bounds.getX() + labelWidth;
        auto trackBounds = juce::Rectangle<float> (trackX, trackY, trackWidth, trackHeight);

        // Draw track background
        g.setColour (isOn ? uiColors.toggleTrackOn : uiColors.toggleTrackOff);
        g.fillRoundedRectangle (trackBounds, cornerRadius);

        // Draw track border
        g.setColour (uiColors.toggleThumbBorder);
        g.drawRoundedRectangle (trackBounds, cornerRadius, 1.0f);

        // Calculate thumb position
        float thumbPadding = (trackHeight - thumbSize) / 2.0f + 1.0f;
        float thumbX = isOn
            ? trackBounds.getRight() - thumbSize - thumbPadding
            : trackBounds.getX() + thumbPadding;
        float thumbY = bounds.getCentreY() - thumbSize / 2.0f;

        // Draw thumb shadow
        g.setColour (juce::Colours::black.withAlpha (0.1f));
        g.fillEllipse (thumbX + 1.0f, thumbY + 1.0f, thumbSize, thumbSize);

        // Draw thumb
        g.setColour (uiColors.toggleThumb);
        g.fillEllipse (thumbX, thumbY, thumbSize, thumbSize);
        g.setColour (uiColors.toggleThumbBorder);
        g.drawEllipse (thumbX, thumbY, thumbSize, thumbSize, 1.0f);

        // Draw labels
        float fontSize = height * 0.42f;
        g.setFont (juce::Font ("Courier New", fontSize, juce::Font::bold));

        // "Manual" label on left
        auto leftLabelBounds = juce::Rectangle<float> (bounds.getX(), bounds.getY(), labelWidth - 4.0f, height);
        g.setColour (isOn ? uiColors.toggleLabelOff : uiColors.toggleLabelOn);
        g.drawText ("Manual", leftLabelBounds, juce::Justification::centredRight, false);

        // "Auto" label on right
        auto rightLabelBounds = juce::Rectangle<float> (trackBounds.getRight() + 4.0f, bounds.getY(), labelWidth - 4.0f, height);
        g.setColour (isOn ? uiColors.toggleLabelOn : uiColors.toggleLabelOff);
        g.drawText ("Auto", rightLabelBounds, juce::Justification::centredLeft, false);
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
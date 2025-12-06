// PluginEditor.h
#pragma once

#include <JuceHeader.h>
#include "PluginProcessor.h"

//==============================================================================
// E-Paper LookAndFeel for rotary knobs with tick marks
class EPaperLookAndFeel : public juce::LookAndFeel_V4
{
public:
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
        auto radius = juce::jmin (width / 2, height / 2) - 4.0f;
        auto centreX = x + width * 0.5f;
        auto centreY = y + height * 0.5f;
        auto rx = centreX - radius;
        auto ry = centreY - radius;
        auto rw = radius * 2.0f;
        auto angle = rotaryStartAngle + sliderPos * (rotaryEndAngle - rotaryStartAngle);

        // Draw outer circle (outline) - lighter color to match text
        g.setColour (juce::Colour::fromRGB (102, 102, 102));
        g.drawEllipse (rx, ry, rw, rw, 2.0f);

        // Draw tick marks
        g.setColour (juce::Colour::fromRGB (224, 224, 224));
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

        // Draw center dot - lighter color to match text
        g.setColour (juce::Colour::fromRGB (102, 102, 102));
        g.fillEllipse (centreX - 3.0f, centreY - 3.0f, 6.0f, 6.0f);

        // Draw indicator line - lighter color to match text
        auto pointerLength = radius - 8.0f;
        auto pointerThickness = 3.0f;
        juce::Path p;
        auto pointerX = centreX + pointerLength * std::cos (angle - juce::MathConstants<float>::halfPi);
        auto pointerY = centreY + pointerLength * std::sin (angle - juce::MathConstants<float>::halfPi);

        p.startNewSubPath (centreX, centreY);
        p.lineTo (pointerX, pointerY);
        g.strokePath (p, juce::PathStrokeType (pointerThickness, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
    }

    void drawButtonBackground (juce::Graphics& g, juce::Button& button, const juce::Colour& backgroundColour,
                              bool shouldDrawButtonAsHighlighted, bool shouldDrawButtonAsDown) override
    {
        auto bounds = button.getLocalBounds().toFloat();

        // E-Paper button background
        g.setColour (juce::Colour::fromRGB (250, 250, 250));
        g.fillRect (bounds);

        // E-Paper button border
        g.setColour (juce::Colour::fromRGB (26, 26, 26));
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
        g.setColour (juce::Colour::fromRGB (250, 250, 250));
        g.fillRect (checkboxBounds);

        g.setColour (juce::Colour::fromRGB (26, 26, 26));
        g.drawRect (checkboxBounds, 2.0f);

        if (button.getToggleState())
        {
            auto tick = checkboxBounds.reduced (4.0f);
            g.setColour (juce::Colour::fromRGB (26, 26, 26));
            g.fillRect (tick);
        }

        // Draw text
        g.setColour (juce::Colour::fromRGB (26, 26, 26));
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

private:
    CloudLikeGranularProcessor& processor;

    std::unique_ptr<EPaperLookAndFeel> ePaperLookAndFeel;  // Custom LookAndFeel for e-paper aesthetic (managed lifetime)

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
    juce::Label modeLabel;  // Displays current mode (Granular/Pitch Shifter/etc.)

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

    void setupKnob (Knob& k, const juce::String& name);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (CloudLikeGranularEditor)
};
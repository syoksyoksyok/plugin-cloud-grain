// PluginEditor.cpp
#include "PluginEditor.h"

//==============================================================================
CloudLikeGranularEditor::CloudLikeGranularEditor (CloudLikeGranularProcessor& p)
    : AudioProcessorEditor (&p)
    , processor (p)
    , ePaperLookAndFeel (std::make_unique<EPaperLookAndFeel>())
{
    setSize (600, 400);  // E-Paper UI: 600x400
    startTimerHz (30);  // Update button states at 30Hz

    // Initialize per-knob LookAndFeel instances with their respective colors
    positionLookAndFeel = std::make_unique<EPaperLookAndFeel>();
    positionLookAndFeel->knobColors = &uiColors.position;

    densityLookAndFeel = std::make_unique<EPaperLookAndFeel>();
    densityLookAndFeel->knobColors = &uiColors.density;

    sizeLookAndFeel = std::make_unique<EPaperLookAndFeel>();
    sizeLookAndFeel->knobColors = &uiColors.size;

    textureLookAndFeel = std::make_unique<EPaperLookAndFeel>();
    textureLookAndFeel->knobColors = &uiColors.texture;

    pitchLookAndFeel = std::make_unique<EPaperLookAndFeel>();
    pitchLookAndFeel->knobColors = &uiColors.pitch;

    spreadLookAndFeel = std::make_unique<EPaperLookAndFeel>();
    spreadLookAndFeel->knobColors = &uiColors.spread;

    feedbackLookAndFeel = std::make_unique<EPaperLookAndFeel>();
    feedbackLookAndFeel->knobColors = &uiColors.feedback;

    reverbLookAndFeel = std::make_unique<EPaperLookAndFeel>();
    reverbLookAndFeel->knobColors = &uiColors.reverb;

    mixLookAndFeel = std::make_unique<EPaperLookAndFeel>();
    mixLookAndFeel->knobColors = &uiColors.mix;

    modeLookAndFeel = std::make_unique<EPaperLookAndFeel>();
    modeLookAndFeel->knobColors = &uiColors.mode;

    trigRateLookAndFeel = std::make_unique<EPaperLookAndFeel>();
    trigRateLookAndFeel->knobColors = &uiColors.trigRate;

    // Setup knobs with their individual LookAndFeel instances (all use custom labels)
    setupKnob (modeKnob, "Mode", modeLookAndFeel.get(), false);
    setupKnob (positionKnob, "Position", positionLookAndFeel.get(), false);
    setupKnob (sizeKnob,     "Size",     sizeLookAndFeel.get(), false);
    setupKnob (pitchKnob,    "Pitch",    pitchLookAndFeel.get(), false);
    setupKnob (densityKnob,  "Density",  densityLookAndFeel.get(), false);
    setupKnob (textureKnob,  "Texture",  textureLookAndFeel.get(), false);
    setupKnob (spreadKnob,   "Spread",   spreadLookAndFeel.get(), false);
    setupKnob (feedbackKnob, "Feedback", feedbackLookAndFeel.get(), false);
    setupKnob (reverbKnob,   "Reverb",   reverbLookAndFeel.get(), false);
    setupKnob (mixKnob,      "Mix",      mixLookAndFeel.get(), false);
    setupKnob (trigRateKnob, "Trig Rate", trigRateLookAndFeel.get(), false);

    addAndMakeVisible (trigModeButton);
    addAndMakeVisible (freezeButton);
    addAndMakeVisible (randomButton);
    addAndMakeVisible (modeValueLabel);
    addAndMakeVisible (trigRateValueLabel);
    addAndMakeVisible (positionValueLabel);
    addAndMakeVisible (sizeValueLabel);
    addAndMakeVisible (pitchValueLabel);
    addAndMakeVisible (densityValueLabel);
    addAndMakeVisible (textureValueLabel);
    addAndMakeVisible (spreadValueLabel);
    addAndMakeVisible (feedbackValueLabel);
    addAndMakeVisible (reverbValueLabel);
    addAndMakeVisible (mixValueLabel);

    // Style MODE value label (E-Paper: shows current mode name)
    modeValueLabel.setFont (juce::Font ("Courier New", 11.0f, juce::Font::plain));
    modeValueLabel.setColour (juce::Label::textColourId, uiColors.knobLabel);
    modeValueLabel.setJustificationType (juce::Justification::centred);

    // Style TRIG RATE value label (E-Paper: shows current division)
    trigRateValueLabel.setFont (juce::Font ("Courier New", 11.0f, juce::Font::plain));
    trigRateValueLabel.setColour (juce::Label::textColourId, uiColors.knobLabel);
    trigRateValueLabel.setJustificationType (juce::Justification::centred);

    // Style all knob value labels (E-Paper: shows meaningful values)
    auto setupValueLabel = [this] (juce::Label& label)
    {
        label.setFont (juce::Font ("Courier New", 11.0f, juce::Font::plain));
        label.setColour (juce::Label::textColourId, uiColors.knobLabel);
        label.setJustificationType (juce::Justification::centred);
    };

    setupValueLabel (positionValueLabel);
    setupValueLabel (sizeValueLabel);
    setupValueLabel (pitchValueLabel);
    setupValueLabel (densityValueLabel);
    setupValueLabel (textureValueLabel);
    setupValueLabel (spreadValueLabel);
    setupValueLabel (feedbackValueLabel);
    setupValueLabel (reverbValueLabel);
    setupValueLabel (mixValueLabel);

    // Style TRIG Mode toggle button (E-Paper: matte black text)
    trigModeButton.setColour (juce::ToggleButton::textColourId, uiColors.buttonText);
    trigModeButton.setColour (juce::ToggleButton::tickColourId, uiColors.buttonText);
    trigModeButton.setColour (juce::ToggleButton::tickDisabledColourId, juce::Colour::fromRGB (160, 160, 160));
    trigModeButton.setLookAndFeel (ePaperLookAndFeel.get());

    // Style Freeze button (E-Paper: matte black text, color changes when ON)
    freezeButton.setColour (juce::ToggleButton::textColourId, uiColors.freezeTextOff);
    freezeButton.setColour (juce::ToggleButton::tickColourId, uiColors.buttonText);
    freezeButton.setColour (juce::ToggleButton::tickDisabledColourId, juce::Colour::fromRGB (160, 160, 160));
    freezeButton.setLookAndFeel (ePaperLookAndFeel.get());

    // Style Randomize button (E-Paper: matte black text, off-white background)
    randomButton.setColour (juce::TextButton::buttonColourId, uiColors.buttonBackground);
    randomButton.setColour (juce::TextButton::buttonOnColourId, uiColors.buttonBackgroundPressed);
    randomButton.setColour (juce::TextButton::textColourOffId, uiColors.buttonText);
    randomButton.setColour (juce::TextButton::textColourOnId, uiColors.buttonText);
    randomButton.setColour (juce::ComboBox::outlineColourId, uiColors.buttonText);
    randomButton.setLookAndFeel (ePaperLookAndFeel.get());

    // Trigger randomize parameter when button is clicked (MIDI/Ableton mappable)
    randomButton.onClick = [this]
    {
        if (auto* param = processor.apvts.getParameter ("randomize"))
        {
            param->beginChangeGesture();
            param->setValueNotifyingHost (1.0f);
            param->endChangeGesture();
            // Reset parameter back to 0 after a short delay
            param->setValueNotifyingHost (0.0f);
        }
    };

    auto& apvts = processor.apvts;

    modeAttachment     = std::make_unique<SliderAttachment> (apvts, "mode",     modeKnob.slider);
    positionAttachment = std::make_unique<SliderAttachment> (apvts, "position", positionKnob.slider);
    sizeAttachment     = std::make_unique<SliderAttachment> (apvts, "size",     sizeKnob.slider);
    pitchAttachment    = std::make_unique<SliderAttachment> (apvts, "pitch",    pitchKnob.slider);
    densityAttachment  = std::make_unique<SliderAttachment> (apvts, "density",  densityKnob.slider);
    textureAttachment  = std::make_unique<SliderAttachment> (apvts, "texture",  textureKnob.slider);
    spreadAttachment   = std::make_unique<SliderAttachment> (apvts, "spread",   spreadKnob.slider);
    feedbackAttachment = std::make_unique<SliderAttachment> (apvts, "feedback", feedbackKnob.slider);
    reverbAttachment   = std::make_unique<SliderAttachment> (apvts, "reverb",   reverbKnob.slider);
    mixAttachment      = std::make_unique<SliderAttachment> (apvts, "mix",      mixKnob.slider);
    trigRateAttachment = std::make_unique<SliderAttachment> (apvts, "trigRate", trigRateKnob.slider);
    trigModeAttachment = std::make_unique<ButtonAttachment> (apvts, "trigMode", trigModeButton);
    freezeAttachment   = std::make_unique<ButtonAttachment> (apvts, "freeze",   freezeButton);
}

CloudLikeGranularEditor::~CloudLikeGranularEditor()
{
    stopTimer();

    // Clear LookAndFeel references before unique_ptr destruction
    // This ensures proper cleanup order to prevent dangling pointers
    modeKnob.slider.setLookAndFeel (nullptr);
    positionKnob.slider.setLookAndFeel (nullptr);
    sizeKnob.slider.setLookAndFeel (nullptr);
    pitchKnob.slider.setLookAndFeel (nullptr);
    densityKnob.slider.setLookAndFeel (nullptr);
    textureKnob.slider.setLookAndFeel (nullptr);
    spreadKnob.slider.setLookAndFeel (nullptr);
    feedbackKnob.slider.setLookAndFeel (nullptr);
    reverbKnob.slider.setLookAndFeel (nullptr);
    mixKnob.slider.setLookAndFeel (nullptr);
    trigRateKnob.slider.setLookAndFeel (nullptr);
    trigModeButton.setLookAndFeel (nullptr);
    freezeButton.setLookAndFeel (nullptr);
    randomButton.setLookAndFeel (nullptr);

    // unique_ptr will automatically clean up ePaperLookAndFeel
}

void CloudLikeGranularEditor::timerCallback()
{
    // Update TRIG mode button text based on parameter
    bool trigMode = processor.apvts.getRawParameterValue ("trigMode")->load() > 0.5f;
    juce::String trigModeText = trigMode ? "Auto" : "Manual";
    if (trigModeButton.getButtonText() != trigModeText)
    {
        trigModeButton.setButtonText (trigModeText);
    }
    if (trigModeButton.getToggleState() != trigMode)
    {
        trigModeButton.setToggleState (trigMode, juce::dontSendNotification);
    }

    // Update freeze button visual state and text color based on parameter
    bool freezeState = processor.apvts.getRawParameterValue ("freeze")->load() > 0.5f;
    if (freezeButton.getToggleState() != freezeState)
    {
        freezeButton.setToggleState (freezeState, juce::dontSendNotification);
        repaint();  // Repaint entire editor to update button highlights
    }

    // Change Freeze button text color when ON
    juce::Colour freezeTextColor = freezeState ? uiColors.freezeTextOn : uiColors.freezeTextOff;
    if (freezeButton.findColour (juce::ToggleButton::textColourId) != freezeTextColor)
    {
        freezeButton.setColour (juce::ToggleButton::textColourId, freezeTextColor);
    }

    // Update knob labels and MODE value display based on mode parameter
    int mode = static_cast<int>(processor.apvts.getRawParameterValue ("mode")->load());

    // Update MODE value label
    juce::String modeValueText;
    switch (mode)
    {
        case 0: modeValueText = "Granular"; break;
        case 1: modeValueText = "PitchShft"; break;
        case 2: modeValueText = "Looping"; break;
        case 3: modeValueText = "Spectral"; break;
        case 4: modeValueText = "Oliverb"; break;
        case 5: modeValueText = "Resonstr"; break;
        case 6: modeValueText = "BeatRpt"; break;
        default: modeValueText = "Unknown"; break;
    }
    if (modeValueLabel.getText() != modeValueText)
    {
        modeValueLabel.setText (modeValueText, juce::dontSendNotification);
    }

    // Define knob labels for each mode
    juce::String posLabel, sizeLabel, pitchLabel, densityLabel, textureLabel;

    switch (mode)
    {
        case 0: // Granular
            posLabel = "Position";
            sizeLabel = "Size";
            pitchLabel = "Pitch";
            densityLabel = "Density";
            textureLabel = "Texture";
            break;

        case 1: // Pitch Shifter (Clouds-style)
            posLabel = "Position";
            sizeLabel = "Window";  // Small=Grainy, Large=Smooth
            pitchLabel = "Pitch";
            densityLabel = "Density";
            textureLabel = "Texture";
            break;

        case 2: // Looping
            posLabel = "LoopPos";
            sizeLabel = "LoopLen";
            pitchLabel = "Speed";
            densityLabel = "Density";
            textureLabel = "Texture";
            break;

        case 3: // Spectral
            posLabel = "Delay";
            sizeLabel = "Window";
            pitchLabel = "FreqShft";
            densityLabel = "Density";
            textureLabel = "Texture";
            break;

        case 4: // Oliverb
            posLabel = "ModRate";
            sizeLabel = "Decay";
            pitchLabel = "Pitch";
            densityLabel = "ModDepth";
            textureLabel = "Diffuse";
            break;

        case 5: // Resonestor
            posLabel = "Excite";
            sizeLabel = "Decay";
            pitchLabel = "Pitch";
            densityLabel = "Pattern";
            textureLabel = "Bright";
            break;

        case 6: // Beat Repeat
            posLabel = "Capture";
            sizeLabel = "Length";
            pitchLabel = "Speed";
            densityLabel = "Rate";
            textureLabel = "Stutter";
            break;

        default:
            posLabel = "Position";
            sizeLabel = "Size";
            pitchLabel = "Pitch";
            densityLabel = "Density";
            textureLabel = "Texture";
            break;
    }

    // Update knob labels
    if (positionKnob.label.getText() != posLabel)
        positionKnob.label.setText (posLabel, juce::dontSendNotification);

    if (sizeKnob.label.getText() != sizeLabel)
        sizeKnob.label.setText (sizeLabel, juce::dontSendNotification);

    if (pitchKnob.label.getText() != pitchLabel)
        pitchKnob.label.setText (pitchLabel, juce::dontSendNotification);

    if (densityKnob.label.getText() != densityLabel)
        densityKnob.label.setText (densityLabel, juce::dontSendNotification);

    if (textureKnob.label.getText() != textureLabel)
        textureKnob.label.setText (textureLabel, juce::dontSendNotification);

    // Update TRIG RATE value label to show current division
    float trigRate = processor.apvts.getRawParameterValue ("trigRate")->load();
    juce::String trigRateValueText;

    if (trigRate < -3.4f)      trigRateValueText = "1/16";
    else if (trigRate < -2.8f) trigRateValueText = "1/16T";
    else if (trigRate < -2.2f) trigRateValueText = "1/8";
    else if (trigRate < -1.6f) trigRateValueText = "1/8T";
    else if (trigRate < -0.8f) trigRateValueText = "1/4";
    else if (trigRate < 0.0f)  trigRateValueText = "1/4T";
    else if (trigRate < 0.8f)  trigRateValueText = "1/2";
    else if (trigRate < 1.6f)  trigRateValueText = "1/2T";
    else if (trigRate < 2.4f)  trigRateValueText = "1bar";
    else if (trigRate < 3.2f)  trigRateValueText = "1barT";
    else                       trigRateValueText = "2bars";

    if (trigRateValueLabel.getText() != trigRateValueText)
    {
        trigRateValueLabel.setText (trigRateValueText, juce::dontSendNotification);
    }

    // Update all knob value labels with mode-specific formatting
    float position = processor.apvts.getRawParameterValue ("position")->load();
    float size = processor.apvts.getRawParameterValue ("size")->load();
    float pitch = processor.apvts.getRawParameterValue ("pitch")->load();
    float density = processor.apvts.getRawParameterValue ("density")->load();
    float texture = processor.apvts.getRawParameterValue ("texture")->load();
    float spread = processor.apvts.getRawParameterValue ("spread")->load();
    float feedback = processor.apvts.getRawParameterValue ("feedback")->load();
    float reverb = processor.apvts.getRawParameterValue ("reverb")->load();
    float mix = processor.apvts.getRawParameterValue ("mix")->load();

    juce::String posValueText, sizeValueText, pitchValueText, densityValueText, textureValueText;

    // Mode-specific formatting
    switch (mode)
    {
        case 0: // Granular
        case 1: // Pitch Shifter
        case 3: // Spectral
            posValueText = juce::String (static_cast<int>(position * 100.0f)) + "%";
            sizeValueText = juce::String (static_cast<int>(size * 1000.0f)) + "ms";
            pitchValueText = (pitch >= 0 ? "+" : "") + juce::String (static_cast<int>(pitch)) + "st";
            densityValueText = juce::String (static_cast<int>(density * 100.0f)) + "%";
            textureValueText = juce::String (static_cast<int>(texture * 100.0f)) + "%";
            break;

        case 2: // Looping
        case 6: // Beat Repeat
            posValueText = juce::String (static_cast<int>(position * 100.0f)) + "%";
            sizeValueText = juce::String (static_cast<int>(size * 1000.0f)) + "ms";
            // Speed: pitch -24 to +24 semitones -> 0.25x to 4x speed
            pitchValueText = juce::String (std::pow (2.0f, pitch / 12.0f), 2) + "x";
            densityValueText = juce::String (static_cast<int>(density * 100.0f)) + "%";
            textureValueText = juce::String (static_cast<int>(texture * 100.0f)) + "%";
            break;

        case 4: // Oliverb
            // ModRate: position 0-1 -> 0-10 Hz
            posValueText = juce::String (position * 10.0f, 1) + "Hz";
            // Decay: size 0.016-1.0 -> 2%-100%
            sizeValueText = juce::String (static_cast<int>((size - 0.016f) / (1.0f - 0.016f) * 98.0f + 2.0f)) + "%";
            pitchValueText = (pitch >= 0 ? "+" : "") + juce::String (static_cast<int>(pitch)) + "st";
            densityValueText = juce::String (static_cast<int>(density * 100.0f)) + "%";
            textureValueText = juce::String (static_cast<int>(texture * 100.0f)) + "%";
            break;

        case 5: // Resonestor
            posValueText = juce::String (static_cast<int>(position * 100.0f)) + "%";
            // Decay: size 0.016-1.0 -> 2%-100%
            sizeValueText = juce::String (static_cast<int>((size - 0.016f) / (1.0f - 0.016f) * 98.0f + 2.0f)) + "%";
            pitchValueText = (pitch >= 0 ? "+" : "") + juce::String (static_cast<int>(pitch)) + "st";
            densityValueText = juce::String (static_cast<int>(density * 100.0f)) + "%";
            textureValueText = juce::String (static_cast<int>(texture * 100.0f)) + "%";
            break;

        default:
            posValueText = juce::String (static_cast<int>(position * 100.0f)) + "%";
            sizeValueText = juce::String (static_cast<int>(size * 1000.0f)) + "ms";
            pitchValueText = (pitch >= 0 ? "+" : "") + juce::String (static_cast<int>(pitch)) + "st";
            densityValueText = juce::String (static_cast<int>(density * 100.0f)) + "%";
            textureValueText = juce::String (static_cast<int>(texture * 100.0f)) + "%";
            break;
    }

    // Common formatting (same for all modes)
    juce::String spreadValueText = juce::String (static_cast<int>(spread * 100.0f)) + "%";
    juce::String feedbackValueText = juce::String (static_cast<int>(feedback * 100.0f)) + "%";
    juce::String reverbValueText = juce::String (static_cast<int>(reverb * 100.0f)) + "%";
    juce::String mixValueText = juce::String (static_cast<int>(mix * 100.0f)) + "%";

    // Update labels if changed
    if (positionValueLabel.getText() != posValueText)
        positionValueLabel.setText (posValueText, juce::dontSendNotification);
    if (sizeValueLabel.getText() != sizeValueText)
        sizeValueLabel.setText (sizeValueText, juce::dontSendNotification);
    if (pitchValueLabel.getText() != pitchValueText)
        pitchValueLabel.setText (pitchValueText, juce::dontSendNotification);
    if (densityValueLabel.getText() != densityValueText)
        densityValueLabel.setText (densityValueText, juce::dontSendNotification);
    if (textureValueLabel.getText() != textureValueText)
        textureValueLabel.setText (textureValueText, juce::dontSendNotification);
    if (spreadValueLabel.getText() != spreadValueText)
        spreadValueLabel.setText (spreadValueText, juce::dontSendNotification);
    if (feedbackValueLabel.getText() != feedbackValueText)
        feedbackValueLabel.setText (feedbackValueText, juce::dontSendNotification);
    if (reverbValueLabel.getText() != reverbValueText)
        reverbValueLabel.setText (reverbValueText, juce::dontSendNotification);
    if (mixValueLabel.getText() != mixValueText)
        mixValueLabel.setText (mixValueText, juce::dontSendNotification);

    // Update LED indicators for tempo visualization
    // LED 1: Base tempo (×1 quarter note)
    if (processor.baseTempoBlink.load())
    {
        baseTempoLedOn = true;
        ledBlinkDuration = 3;  // Blink for 3 timer ticks (~100ms at 30Hz)
        processor.baseTempoBlink.store(false);
        repaint();
    }
    else if (ledBlinkDuration > 0)
    {
        ledBlinkDuration--;
        if (ledBlinkDuration == 0)
        {
            baseTempoLedOn = false;
            repaint();
        }
    }

    // LED 2: TRIG RATE tempo
    if (processor.trigRateBlink.load())
    {
        trigRateLedOn = true;
        ledBlinkDuration2 = 3;  // Blink for 3 timer ticks (~100ms at 30Hz)
        processor.trigRateBlink.store(false);
        repaint();
    }
    else if (ledBlinkDuration2 > 0)
    {
        ledBlinkDuration2--;
        if (ledBlinkDuration2 == 0)
        {
            trigRateLedOn = false;
            repaint();
        }
    }
}

void CloudLikeGranularEditor::setupKnob (Knob& k, const juce::String& name, EPaperLookAndFeel* lookAndFeel, bool showTextBox)
{
    addAndMakeVisible (k.slider);
    addAndMakeVisible (k.label);

    k.slider.setSliderStyle (juce::Slider::RotaryHorizontalVerticalDrag);
    if (showTextBox)
        k.slider.setTextBoxStyle (juce::Slider::TextBoxBelow, false, 60, 22);
    else
        k.slider.setTextBoxStyle (juce::Slider::NoTextBox, false, 0, 0);
    k.slider.setPopupDisplayEnabled (false, false, this);

    // Use JUCE default rotary parameters (no explicit setRotaryParameters call)
    // This should give standard knob behavior: left-bottom = min, right-bottom = max

    // Apply E-Paper LookAndFeel (with per-knob colors)
    k.slider.setLookAndFeel (lookAndFeel);

    // E-Paper styling for text box and label
    k.slider.setColour (juce::Slider::textBoxTextColourId, uiColors.knobLabel);
    k.slider.setColour (juce::Slider::textBoxBackgroundColourId, juce::Colours::transparentBlack);
    k.slider.setColour (juce::Slider::textBoxOutlineColourId, juce::Colours::transparentBlack);

    k.label.setText (name, juce::dontSendNotification);
    k.label.attachToComponent (&k.slider, false);
    k.label.setJustificationType (juce::Justification::centred);
    k.label.setColour (juce::Label::textColourId, uiColors.knobLabel);
    k.label.setFont (juce::Font ("Courier New", 12.0f, juce::Font::plain));
}

void CloudLikeGranularEditor::paint (juce::Graphics& g)
{
    // E-Paper UI: Background
    g.fillAll (uiColors.background);

    // Draw border
    g.setColour (uiColors.buttonText);
    g.drawRect (getLocalBounds(), 2);

    // Draw separator line above buttons
    g.setColour (juce::Colour::fromRGB (224, 224, 224));
    g.fillRect (10, getHeight() - 60, getWidth() - 20, 1);

    // Draw subtle border around Freeze button when active (E-Paper style)
    if (freezeButton.getToggleState())
    {
        auto freezeBounds = freezeButton.getBounds().toFloat().reduced (1.0f);
        g.setColour (uiColors.buttonText);
        g.drawRoundedRectangle (freezeBounds, 0.0f, 2.0f);
    }

    // Draw LED indicators (tempo visualization)
    // Position LEDs in the row 3 area, left of TRIG Rate knob
    auto ledSize = 12.0f;
    auto ledY = getHeight() - 200.0f;  // Align with row 3
    auto ledX1 = 80.0f;   // LED 1: Base tempo (×1)
    auto ledX2 = 110.0f;  // LED 2: TRIG RATE tempo

    // LED 1: Base tempo (×1 quarter note)
    g.setColour (juce::Colour::fromRGB (200, 200, 200));  // Off color (light gray)
    g.fillEllipse (ledX1, ledY, ledSize, ledSize);
    if (baseTempoLedOn)
    {
        g.setColour (juce::Colour::fromRGB (100, 180, 100));  // On color (green)
        g.fillEllipse (ledX1 + 2, ledY + 2, ledSize - 4, ledSize - 4);
    }
    g.setColour (uiColors.buttonText);
    g.drawEllipse (ledX1, ledY, ledSize, ledSize, 0.75f);

    // LED 2: TRIG RATE tempo
    g.setColour (juce::Colour::fromRGB (200, 200, 200));  // Off color (light gray)
    g.fillEllipse (ledX2, ledY, ledSize, ledSize);
    if (trigRateLedOn)
    {
        g.setColour (juce::Colour::fromRGB (180, 100, 100));  // On color (red)
        g.fillEllipse (ledX2 + 2, ledY + 2, ledSize - 4, ledSize - 4);
    }
    g.setColour (uiColors.buttonText);
    g.drawEllipse (ledX2, ledY, ledSize, ledSize, 0.75f);

    // LED labels
    g.setColour (uiColors.knobLabel);
    g.setFont (juce::Font ("Courier New", 9.0f, juce::Font::plain));
    g.drawText ("×1", ledX1 - 5, ledY + ledSize + 2, ledSize + 10, 12, juce::Justification::centred);
    g.drawText ("TRIG", ledX2 - 8, ledY + ledSize + 2, ledSize + 16, 12, juce::Justification::centred);
}

void CloudLikeGranularEditor::resized()
{
    auto area = getLocalBounds().reduced (10);

    // E-Paper UI: 3 rows × 5 columns grid (added 3rd row for TRIG rate)
    auto buttonRow = area.removeFromBottom (50);
    auto knobArea = area.reduced (0, 5);

    auto rowHeight = knobArea.getHeight() / 3;
    auto row1 = knobArea.removeFromTop (rowHeight);
    auto row2 = knobArea.removeFromTop (rowHeight);
    auto row3 = knobArea;

    auto colWidth = row1.getWidth() / 5;

    auto placeKnob = [] (Knob& k, juce::Rectangle<int> r)
    {
        // Double the knob size by reducing margin from 30 to 15
        k.slider.setBounds (r.reduced (15));
    };

    // Row 1: Position, Density, Size, Texture, Pitch
    auto positionKnobArea = row1.removeFromLeft (colWidth);
    auto densityKnobArea = row1.removeFromLeft (colWidth);
    auto sizeKnobArea = row1.removeFromLeft (colWidth);
    auto textureKnobArea = row1.removeFromLeft (colWidth);
    auto pitchKnobArea = row1.removeFromLeft (colWidth);

    placeKnob (positionKnob, positionKnobArea);
    placeKnob (densityKnob,  densityKnobArea);
    placeKnob (sizeKnob,     sizeKnobArea);
    placeKnob (textureKnob,  textureKnobArea);
    placeKnob (pitchKnob,    pitchKnobArea);

    // Row 1 value labels (below each knob)
    positionValueLabel.setBounds (positionKnobArea.withHeight (20).withY (positionKnobArea.getBottom() - 25));
    densityValueLabel.setBounds (densityKnobArea.withHeight (20).withY (densityKnobArea.getBottom() - 25));
    sizeValueLabel.setBounds (sizeKnobArea.withHeight (20).withY (sizeKnobArea.getBottom() - 25));
    textureValueLabel.setBounds (textureKnobArea.withHeight (20).withY (textureKnobArea.getBottom() - 25));
    pitchValueLabel.setBounds (pitchKnobArea.withHeight (20).withY (pitchKnobArea.getBottom() - 25));

    // Row 2: Spread, Feedback, Reverb, Mix, Mode
    auto spreadKnobArea = row2.removeFromLeft (colWidth);
    auto feedbackKnobArea = row2.removeFromLeft (colWidth);
    auto reverbKnobArea = row2.removeFromLeft (colWidth);
    auto mixKnobArea = row2.removeFromLeft (colWidth);
    auto modeKnobArea = row2.removeFromLeft (colWidth);

    placeKnob (spreadKnob,   spreadKnobArea);
    placeKnob (feedbackKnob, feedbackKnobArea);
    placeKnob (reverbKnob,   reverbKnobArea);
    placeKnob (mixKnob,      mixKnobArea);
    placeKnob (modeKnob,     modeKnobArea);

    // Row 2 value labels (below each knob)
    spreadValueLabel.setBounds (spreadKnobArea.withHeight (20).withY (spreadKnobArea.getBottom() - 25));
    feedbackValueLabel.setBounds (feedbackKnobArea.withHeight (20).withY (feedbackKnobArea.getBottom() - 25));
    reverbValueLabel.setBounds (reverbKnobArea.withHeight (20).withY (reverbKnobArea.getBottom() - 25));
    mixValueLabel.setBounds (mixKnobArea.withHeight (20).withY (mixKnobArea.getBottom() - 25));
    modeValueLabel.setBounds (modeKnobArea.withHeight (20).withY (modeKnobArea.getBottom() - 25));

    // Row 3: TRIG Rate (centered, single knob)
    auto trigRateArea = row3.withSizeKeepingCentre (colWidth, rowHeight);
    placeKnob (trigRateKnob, trigRateArea);

    // TRIG RATE value label (shows current division below the knob: 1/16, 1/8T, etc.)
    auto trigRateValueLabelArea = trigRateArea.withHeight (20).withY (trigRateArea.getBottom() - 25);
    trigRateValueLabel.setBounds (trigRateValueLabelArea);

    // E-Paper UI: Buttons at bottom (3 buttons: TrigMode, Freeze, Randomize)
    // All buttons have equal size and spacing
    auto buttonArea = buttonRow.reduced (5);
    auto buttonWidth = buttonArea.getWidth() / 3;
    auto buttonPadding = 3;

    auto trigModeArea = buttonArea.removeFromLeft (buttonWidth).reduced (buttonPadding);
    auto freezeArea = buttonArea.removeFromLeft (buttonWidth).reduced (buttonPadding);
    auto randomArea = buttonArea.removeFromLeft (buttonWidth).reduced (buttonPadding);

    trigModeButton.setBounds (trigModeArea);
    freezeButton.setBounds (freezeArea);
    randomButton.setBounds (randomArea);
}
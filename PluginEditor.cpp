// PluginEditor.cpp
#include "PluginEditor.h"

//==============================================================================
CloudLikeGranularEditor::CloudLikeGranularEditor (CloudLikeGranularProcessor& p)
    : AudioProcessorEditor (&p)
    , processor (p)
    , ePaperLookAndFeel (std::make_unique<EPaperLookAndFeel>())
{
    // Set initial size using fixed base dimensions
    setSize (UISize::baseWidth, UISize::baseHeight);
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
    addAndMakeVisible (killDryButton);
    addAndMakeVisible (killWetButton);
    addAndMakeVisible (modeValueLabel);
    addAndMakeVisible (trigRateValueLabel);
    addAndMakeVisible (tapBpmLabel);
    addAndMakeVisible (tapButton);
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

    // Style Kill Dry button (momentary button - ON while pressed)
    // Parameter is in APVTS so Ableton Configure can map it
    killDryButton.setColour (juce::TextButton::buttonColourId, uiColors.buttonBackground);
    killDryButton.setColour (juce::TextButton::buttonOnColourId, uiColors.buttonBackgroundPressed);
    killDryButton.setColour (juce::TextButton::textColourOffId, uiColors.buttonText);
    killDryButton.setColour (juce::TextButton::textColourOnId, uiColors.position.outline);
    killDryButton.setColour (juce::ComboBox::outlineColourId, uiColors.buttonText);
    killDryButton.setLookAndFeel (ePaperLookAndFeel.get());

    // Kill Dry: Momentary - ON while mouse is down
    killDryButton.onStateChange = [this]
    {
        if (auto* param = processor.apvts.getParameter ("killDry"))
        {
            bool isDown = killDryButton.isDown();
            param->beginChangeGesture();
            param->setValueNotifyingHost (isDown ? 1.0f : 0.0f);
            param->endChangeGesture();
        }
    };

    // Style Kill Wet button (momentary button - ON while pressed)
    killWetButton.setColour (juce::TextButton::buttonColourId, uiColors.buttonBackground);
    killWetButton.setColour (juce::TextButton::buttonOnColourId, uiColors.buttonBackgroundPressed);
    killWetButton.setColour (juce::TextButton::textColourOffId, uiColors.buttonText);
    killWetButton.setColour (juce::TextButton::textColourOnId, uiColors.position.outline);
    killWetButton.setColour (juce::ComboBox::outlineColourId, uiColors.buttonText);
    killWetButton.setLookAndFeel (ePaperLookAndFeel.get());

    // Kill Wet: Momentary - ON while mouse is down
    killWetButton.onStateChange = [this]
    {
        if (auto* param = processor.apvts.getParameter ("killWet"))
        {
            bool isDown = killWetButton.isDown();
            param->beginChangeGesture();
            param->setValueNotifyingHost (isDown ? 1.0f : 0.0f);
            param->endChangeGesture();
        }
    };

    // Style tap BPM label (E-Paper: circular display like a knob)
    tapBpmLabel.setFont (juce::Font ("Courier New", 10.0f, juce::Font::bold));
    tapBpmLabel.setColour (juce::Label::textColourId, uiColors.knobLabel);
    tapBpmLabel.setJustificationType (juce::Justification::centred);
    tapBpmLabel.setText ("---", juce::dontSendNotification);
    tapBpmLabel.setInterceptsMouseClicks (false, false);  // Let tap button receive clicks

    // Style tap button (transparent overlay for Manual mode tap trigger)
    // Parameter is in APVTS so Ableton Configure can map it
    tapButton.setButtonText ("");  // No text, transparent overlay
    tapButton.setColour (juce::TextButton::buttonColourId, juce::Colours::transparentWhite);
    tapButton.setColour (juce::TextButton::buttonOnColourId, juce::Colours::transparentWhite);
    tapButton.setColour (juce::ComboBox::outlineColourId, juce::Colours::transparentWhite);

    // Tap button: Momentary - ON while mouse is down
    tapButton.onStateChange = [this]
    {
        if (auto* param = processor.apvts.getParameter ("tap"))
        {
            bool isDown = tapButton.isDown();
            param->beginChangeGesture();
            param->setValueNotifyingHost (isDown ? 1.0f : 0.0f);
            param->endChangeGesture();
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
    // Note: killDry/killWet don't use ButtonAttachment to preserve momentary behavior
    // Parameters are still in APVTS for Ableton Configure mapping
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
    killDryButton.setLookAndFeel (nullptr);
    killWetButton.setLookAndFeel (nullptr);

    // unique_ptr will automatically clean up ePaperLookAndFeel
}

void CloudLikeGranularEditor::timerCallback()
{
    // Get current parameter values
    bool trigMode = processor.apvts.getRawParameterValue("trigMode")->load() > 0.5f;
    int mode = static_cast<int>(processor.apvts.getRawParameterValue("mode")->load());
    float trigRate = processor.apvts.getRawParameterValue("trigRate")->load();

    float position = processor.apvts.getRawParameterValue("position")->load();
    float size = processor.apvts.getRawParameterValue("size")->load();
    float pitch = processor.apvts.getRawParameterValue("pitch")->load();
    float density = processor.apvts.getRawParameterValue("density")->load();
    float texture = processor.apvts.getRawParameterValue("texture")->load();
    float spread = processor.apvts.getRawParameterValue("spread")->load();
    float feedback = processor.apvts.getRawParameterValue("feedback")->load();
    float reverb = processor.apvts.getRawParameterValue("reverb")->load();
    float mix = processor.apvts.getRawParameterValue("mix")->load();
    bool killDry = processor.apvts.getRawParameterValue("killDry")->load() > 0.5f;
    bool killWet = processor.apvts.getRawParameterValue("killWet")->load() > 0.5f;

    // Update MIX knob indicator position when Kill Dry/Wet is active
    bool needsRepaint = false;
    if (mixLookAndFeel->forceMaxPosition != killDry)
    {
        mixLookAndFeel->forceMaxPosition = killDry;
        needsRepaint = true;
    }
    if (mixLookAndFeel->forceMinPosition != killWet)
    {
        mixLookAndFeel->forceMinPosition = killWet;
        needsRepaint = true;
    }
    if (needsRepaint)
        mixKnob.slider.repaint();

    // Update UI components using helper functions
    updateButtonStates(trigMode);
    updateModeLabels(mode);
    updateTrigRateLabel(trigRate);
    updateKnobValueLabels(mode, position, size, pitch, density, texture, spread, feedback, reverb, mix);
    updateLedIndicators();
    updateBpmDisplay(trigMode);
}

// ========== REFACTORED: timerCallback helper functions ==========

void CloudLikeGranularEditor::updateButtonStates(bool trigMode)
{
    // Update TRIG mode button
    juce::String trigModeText = trigMode ? "Auto" : "Manual";
    if (trigModeButton.getButtonText() != trigModeText)
        trigModeButton.setButtonText(trigModeText);
    if (trigModeButton.getToggleState() != trigMode)
        trigModeButton.setToggleState(trigMode, juce::dontSendNotification);

    // Update freeze button
    bool freezeState = processor.apvts.getRawParameterValue("freeze")->load() > 0.5f;
    if (freezeButton.getToggleState() != freezeState)
    {
        freezeButton.setToggleState(freezeState, juce::dontSendNotification);
        repaint();
    }

    juce::Colour freezeTextColor = freezeState ? uiColors.freezeTextOn : uiColors.freezeTextOff;
    if (freezeButton.findColour(juce::ToggleButton::textColourId) != freezeTextColor)
        freezeButton.setColour(juce::ToggleButton::textColourId, freezeTextColor);

    // Update Kill Dry/Wet button text colors based on parameter state
    // (for external control via Ableton Configure MIDI mapping)
    bool killDryState = processor.apvts.getRawParameterValue("killDry")->load() > 0.5f;
    bool killWetState = processor.apvts.getRawParameterValue("killWet")->load() > 0.5f;

    juce::Colour killDryTextColor = killDryState ? uiColors.position.outline : uiColors.buttonText;
    if (killDryButton.findColour(juce::TextButton::textColourOffId) != killDryTextColor)
    {
        killDryButton.setColour(juce::TextButton::textColourOffId, killDryTextColor);
        killDryButton.repaint();
    }

    juce::Colour killWetTextColor = killWetState ? uiColors.position.outline : uiColors.buttonText;
    if (killWetButton.findColour(juce::TextButton::textColourOffId) != killWetTextColor)
    {
        killWetButton.setColour(juce::TextButton::textColourOffId, killWetTextColor);
        killWetButton.repaint();
    }

    // Update tap button (only active in Manual mode)
    if (!trigMode)
    {
        // Manual mode: show tap button with pointing hand cursor
        if (!tapButton.isVisible())
            tapButton.setVisible(true);
        tapButton.setMouseCursor(juce::MouseCursor::PointingHandCursor);
    }
    else
    {
        // Auto mode: hide tap button (BPM display from DAW)
        if (tapButton.isVisible())
            tapButton.setVisible(false);
    }
}

void CloudLikeGranularEditor::updateModeLabels(int mode)
{
    // Update MODE value label
    juce::String modeValueText;
    switch (mode)
    {
        case 0: modeValueText = "GranProc"; break;
        case 1: modeValueText = "Pitch/Time"; break;
        case 2: modeValueText = "LoopDly"; break;
        case 3: modeValueText = "SpctrlMad"; break;
        case 4: modeValueText = "Oliverb"; break;
        case 5: modeValueText = "Resonstr"; break;
        case 6: modeValueText = "BeatRpt"; break;
        case 7: modeValueText = "SpctrlCld"; break;
        default: modeValueText = "Unknown"; break;
    }
    if (modeValueLabel.getText() != modeValueText)
        modeValueLabel.setText(modeValueText, juce::dontSendNotification);

    // Define knob labels for each mode
    juce::String posLabel, sizeLabel, pitchLabel, densityLabel, textureLabel;
    switch (mode)
    {
        case 0: posLabel = "Position"; sizeLabel = "Size"; pitchLabel = "Pitch"; densityLabel = "Density"; textureLabel = "Texture"; break;
        case 1: posLabel = "Position"; sizeLabel = "Window"; pitchLabel = "Pitch"; densityLabel = "Density"; textureLabel = "Texture"; break;
        case 2: posLabel = "LoopPos"; sizeLabel = "LoopLen"; pitchLabel = "Speed"; densityLabel = "Density"; textureLabel = "Texture"; break;
        case 3: posLabel = "Delay"; sizeLabel = "Window"; pitchLabel = "FreqShft"; densityLabel = "Density"; textureLabel = "Texture"; break;
        case 4: posLabel = "ModRate"; sizeLabel = "Decay"; pitchLabel = "Pitch"; densityLabel = "ModDepth"; textureLabel = "Diffuse"; break;
        case 5: posLabel = "Excite"; sizeLabel = "Decay"; pitchLabel = "Pitch"; densityLabel = "Pattern"; textureLabel = "Bright"; break;
        case 6: posLabel = "Capture"; sizeLabel = "Length"; pitchLabel = "Speed"; densityLabel = "Rate"; textureLabel = "Stutter"; break;
        case 7: posLabel = "Filter"; sizeLabel = "Bands"; pitchLabel = "Pitch"; densityLabel = "Smooth"; textureLabel = "Phase"; break;
        default: posLabel = "Position"; sizeLabel = "Size"; pitchLabel = "Pitch"; densityLabel = "Density"; textureLabel = "Texture"; break;
    }

    if (positionKnob.label.getText() != posLabel) positionKnob.label.setText(posLabel, juce::dontSendNotification);
    if (sizeKnob.label.getText() != sizeLabel) sizeKnob.label.setText(sizeLabel, juce::dontSendNotification);
    if (pitchKnob.label.getText() != pitchLabel) pitchKnob.label.setText(pitchLabel, juce::dontSendNotification);
    if (densityKnob.label.getText() != densityLabel) densityKnob.label.setText(densityLabel, juce::dontSendNotification);
    if (textureKnob.label.getText() != textureLabel) textureKnob.label.setText(textureLabel, juce::dontSendNotification);
}

void CloudLikeGranularEditor::updateTrigRateLabel(float trigRate)
{
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
        trigRateValueLabel.setText(trigRateValueText, juce::dontSendNotification);
}

void CloudLikeGranularEditor::updateKnobValueLabels(int mode, float position, float size, float pitch,
                                                     float density, float texture, float spread,
                                                     float feedback, float reverb, float mix)
{
    juce::String posValueText, sizeValueText, pitchValueText, densityValueText, textureValueText;

    switch (mode)
    {
        case 0: case 1: case 3:
            posValueText = juce::String(static_cast<int>(position * 100.0f)) + "%";
            sizeValueText = juce::String(static_cast<int>(size * 1000.0f)) + "ms";
            pitchValueText = (pitch >= 0 ? "+" : "") + juce::String(static_cast<int>(pitch)) + "st";
            densityValueText = juce::String(static_cast<int>(density * 100.0f)) + "%";
            textureValueText = juce::String(static_cast<int>(texture * 100.0f)) + "%";
            break;
        case 2: case 6:
            posValueText = juce::String(static_cast<int>(position * 100.0f)) + "%";
            sizeValueText = juce::String(static_cast<int>(size * 1000.0f)) + "ms";
            pitchValueText = juce::String(std::pow(2.0f, pitch / 12.0f), 2) + "x";
            densityValueText = juce::String(static_cast<int>(density * 100.0f)) + "%";
            textureValueText = juce::String(static_cast<int>(texture * 100.0f)) + "%";
            break;
        case 4:
            posValueText = juce::String(position * 10.0f, 1) + "Hz";
            sizeValueText = juce::String(static_cast<int>((size - 0.016f) / (1.0f - 0.016f) * 98.0f + 2.0f)) + "%";
            pitchValueText = (pitch >= 0 ? "+" : "") + juce::String(static_cast<int>(pitch)) + "st";
            densityValueText = juce::String(static_cast<int>(density * 100.0f)) + "%";
            textureValueText = juce::String(static_cast<int>(texture * 100.0f)) + "%";
            break;
        case 5:
            posValueText = juce::String(static_cast<int>(position * 100.0f)) + "%";
            sizeValueText = juce::String(static_cast<int>((size - 0.016f) / (1.0f - 0.016f) * 98.0f + 2.0f)) + "%";
            pitchValueText = (pitch >= 0 ? "+" : "") + juce::String(static_cast<int>(pitch)) + "st";
            densityValueText = juce::String(static_cast<int>(density * 100.0f)) + "%";
            textureValueText = juce::String(static_cast<int>(texture * 100.0f)) + "%";
            break;
        case 7:
            posValueText = juce::String(static_cast<int>(position * 100.0f)) + "%";
            sizeValueText = juce::String(4 + static_cast<int>(size * 60.0f));
            pitchValueText = (pitch >= 0 ? "+" : "") + juce::String(static_cast<int>(pitch)) + "st";
            densityValueText = juce::String(static_cast<int>(density * 100.0f)) + "%";
            textureValueText = juce::String(static_cast<int>(texture * 100.0f)) + "%";
            break;
        default:
            posValueText = juce::String(static_cast<int>(position * 100.0f)) + "%";
            sizeValueText = juce::String(static_cast<int>(size * 1000.0f)) + "ms";
            pitchValueText = (pitch >= 0 ? "+" : "") + juce::String(static_cast<int>(pitch)) + "st";
            densityValueText = juce::String(static_cast<int>(density * 100.0f)) + "%";
            textureValueText = juce::String(static_cast<int>(texture * 100.0f)) + "%";
            break;
    }

    juce::String spreadValueText = juce::String(static_cast<int>(spread * 100.0f)) + "%";
    juce::String feedbackValueText = juce::String(static_cast<int>(feedback * 100.0f)) + "%";
    juce::String reverbValueText = juce::String(static_cast<int>(reverb * 100.0f)) + "%";

    // Kill Dry/Wet: Show 100% or 0% when button is pressed (Kill Dry takes priority)
    bool killDry = processor.apvts.getRawParameterValue("killDry")->load() > 0.5f;
    bool killWet = processor.apvts.getRawParameterValue("killWet")->load() > 0.5f;
    juce::String mixValueText = killDry ? "100%" : (killWet ? "0%" : juce::String(static_cast<int>(mix * 100.0f)) + "%");

    if (positionValueLabel.getText() != posValueText) positionValueLabel.setText(posValueText, juce::dontSendNotification);
    if (sizeValueLabel.getText() != sizeValueText) sizeValueLabel.setText(sizeValueText, juce::dontSendNotification);
    if (pitchValueLabel.getText() != pitchValueText) pitchValueLabel.setText(pitchValueText, juce::dontSendNotification);
    if (densityValueLabel.getText() != densityValueText) densityValueLabel.setText(densityValueText, juce::dontSendNotification);
    if (textureValueLabel.getText() != textureValueText) textureValueLabel.setText(textureValueText, juce::dontSendNotification);
    if (spreadValueLabel.getText() != spreadValueText) spreadValueLabel.setText(spreadValueText, juce::dontSendNotification);
    if (feedbackValueLabel.getText() != feedbackValueText) feedbackValueLabel.setText(feedbackValueText, juce::dontSendNotification);
    if (reverbValueLabel.getText() != reverbValueText) reverbValueLabel.setText(reverbValueText, juce::dontSendNotification);
    if (mixValueLabel.getText() != mixValueText) mixValueLabel.setText(mixValueText, juce::dontSendNotification);
}

void CloudLikeGranularEditor::updateLedIndicators()
{
    // LED 1: Base tempo
    if (processor.baseTempoBlink.load())
    {
        baseTempoLedOn = true;
        ledBlinkDuration = 3;
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

    // LED 2: TRIG RATE tempo / MIDI note
    bool trigMode = processor.apvts.getRawParameterValue("trigMode")->load() > 0.5f;

    if (!trigMode)
    {
        // Manual mode: LED follows MIDI note held state
        bool noteHeld = processor.midiNoteHeld.load();
        if (processor.trigRateBlink.load())
        {
            trigRateLedOn = true;
            processor.trigRateBlink.store(false);
            repaint();
        }
        else if (!noteHeld && trigRateLedOn)
        {
            trigRateLedOn = false;
            repaint();
        }
    }
    else
    {
        // Auto mode: standard blink behavior
        if (processor.trigRateBlink.load())
        {
            trigRateLedOn = true;
            ledBlinkDuration2 = 3;
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
}

void CloudLikeGranularEditor::updateBpmDisplay(bool trigMode)
{
    // Manual mode: show "---" (no BPM display)
    // Auto mode: show host BPM
    juce::String bpmText = "---";
    if (trigMode)
    {
        float displayBPM = processor.hostBPM.load();
        if (displayBPM > 0.0f)
            bpmText = juce::String(static_cast<int>(displayBPM));
    }
    if (tapBpmLabel.getText() != bpmText)
        tapBpmLabel.setText(bpmText, juce::dontSendNotification);
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

    // Scaled values for paint
    const int margin = scaled (UISize::windowMargin);
    const float ledLabelFontSize = scaledF (static_cast<float> (UISize::ledLabelFontSize));
    const float bpmFontSize = scaledF (static_cast<float> (UISize::bpmFontSize));
    const float glowScale = scaledF (3.0f);
    const float borderScale = scaledF (1.0f);

    // Draw border
    g.setColour (uiColors.buttonText);
    g.drawRect (getLocalBounds(), scaled (2));

    // Draw separator line above buttons
    int separatorY = getHeight() - scaled (UISize::buttonRowHeight) - margin;
    g.setColour (juce::Colour::fromRGB (224, 224, 224));
    g.fillRect (margin, separatorY, getWidth() - margin * 2, scaled (1));

    // Draw subtle border around Freeze button when active (E-Paper style)
    if (freezeButton.getToggleState())
    {
        auto freezeBounds = freezeButton.getBounds().toFloat().reduced (borderScale);
        g.setColour (uiColors.buttonText);
        g.drawRoundedRectangle (freezeBounds, 0.0f, scaledF (2.0f));
    }

    // Draw circular background for tap BPM label (knob-style display)
    auto bpmBounds = tapBpmLabel.getBounds().toFloat();

    // Check if Manual mode (clickable)
    bool trigMode = processor.apvts.getRawParameterValue ("trigMode")->load() > 0.5f;
    bool isManualMode = !trigMode;

    // Background color: green when base tempo blinks, otherwise normal colors
    // Draw diffuse glow effect when LED is on (realistic LED appearance)
    if (baseTempoLedOn)
    {
        // Outer glow layers (diffuse effect)
        auto glowColor = juce::Colour::fromRGB (100, 180, 100);
        for (int i = 4; i >= 1; --i)
        {
            float glowExpand = i * glowScale;
            float alpha = 0.15f / i;  // Fading alpha for outer layers
            g.setColour (glowColor.withAlpha (alpha));
            g.fillEllipse (bpmBounds.expanded (glowExpand));
        }
        // Main LED color
        g.setColour (glowColor);
    }
    else if (isManualMode)
    {
        g.setColour (juce::Colour::fromRGB (250, 250, 250));  // Almost white (clickable)
    }
    else
    {
        g.setColour (juce::Colour::fromRGB (230, 230, 230));  // Darker gray (disabled)
    }
    g.fillEllipse (bpmBounds);

    // Border: thinner line, color matches MIX knob outline
    g.setColour (uiColors.mix.outline);
    float borderThickness = isManualMode ? scaledF (1.25f) : scaledF (0.75f);
    g.drawEllipse (bpmBounds, borderThickness);

    // Draw label below the circle: "TAP" in Manual mode, "BPM" in Auto mode
    auto bpmLabelArea = bpmBounds.withY (bpmBounds.getBottom() + scaledF (2.0f)).withHeight (scaledF (15.0f));
    g.setFont (juce::Font ("Courier New", bpmFontSize, juce::Font::plain));
    g.setColour (uiColors.knobLabel);
    g.drawText (isManualMode ? "TAP" : "BPM", bpmLabelArea, juce::Justification::centred);

    // Draw LED indicator (TRIG RATE tempo) - positioned to the right of BPM circle
    float ledSize = scaledF (static_cast<float> (UISize::ledDiameter));
    float ledSpacing = scaledF (static_cast<float> (UISize::ledSpacing));
    float ledX = bpmBounds.getRight() + ledSpacing;
    float ledY = bpmBounds.getCentreY() - ledSize / 2.0f;

    // LED: TRIG RATE tempo (red) with diffuse glow effect
    auto trigLedBounds = juce::Rectangle<float> (ledX, ledY, ledSize, ledSize);

    // Draw diffuse glow effect when LED is on (realistic LED appearance)
    if (trigRateLedOn)
    {
        auto glowColor = juce::Colour::fromRGB (180, 100, 100);
        for (int i = 4; i >= 1; --i)
        {
            float glowExpand = i * glowScale;
            float alpha = 0.15f / i;  // Fading alpha for outer layers
            g.setColour (glowColor.withAlpha (alpha));
            g.fillEllipse (trigLedBounds.expanded (glowExpand));
        }
        // Main LED color
        g.setColour (glowColor);
        g.fillEllipse (trigLedBounds);
    }
    else
    {
        g.setColour (juce::Colour::fromRGB (200, 200, 200));  // Off color (light gray)
        g.fillEllipse (trigLedBounds);
    }
    g.setColour (uiColors.mix.outline);  // Same color as BPM LED border
    g.drawEllipse (trigLedBounds, scaledF (0.75f));

    // LED label (below LED)
    g.setColour (uiColors.knobLabel);
    g.setFont (juce::Font ("Courier New", ledLabelFontSize, juce::Font::plain));
    int labelWidth = static_cast<int> (ledSize + scaledF (16.0f));
    int labelX = static_cast<int> (ledX - scaledF (8.0f));
    int labelY = static_cast<int> (ledY + ledSize + scaledF (2.0f));
    g.drawText ("TRIG", labelX, labelY, labelWidth, scaled (12), juce::Justification::centred);
}

void CloudLikeGranularEditor::resized()
{
    // ===== Fixed Layout with Scaling Support =====
    // All sizes are based on UISize constants, multiplied by uiScale

    const int margin = scaled (UISize::windowMargin);
    const int cellW = scaled (UISize::knobCellWidth);
    const int rowH = scaled (UISize::knobRowHeight);
    const int row3H = scaled (UISize::row3Height);
    const int btnRowH = scaled (UISize::buttonRowHeight);
    const int knobSize = scaled (UISize::knobDiameter);
    const int labelH = scaled (UISize::labelHeight);
    const int labelOffsetY = scaled (UISize::knobLabelOffsetY);
    const int btnPadding = scaled (UISize::buttonPadding);
    const int bpmSize = scaled (UISize::bpmDisplaySize);

    // Helper to center a knob within a cell
    auto placeKnobInCell = [&] (Knob& k, int cellX, int cellY)
    {
        int knobX = cellX + (cellW - knobSize) / 2;
        int knobY = cellY + (rowH - knobSize - labelH) / 2;
        k.slider.setBounds (knobX, knobY, knobSize, knobSize);
    };

    // Helper to place value label below knob
    auto placeLabelInCell = [&] (juce::Label& label, int cellX, int cellY)
    {
        int labelY = cellY + rowH - labelOffsetY;
        label.setBounds (cellX, labelY, cellW, labelH);
    };

    // ===== Row 1: Position, Density, Size, Texture, Pitch =====
    int row1Y = margin;
    placeKnobInCell (positionKnob, margin + cellW * 0, row1Y);
    placeKnobInCell (densityKnob,  margin + cellW * 1, row1Y);
    placeKnobInCell (sizeKnob,     margin + cellW * 2, row1Y);
    placeKnobInCell (textureKnob,  margin + cellW * 3, row1Y);
    placeKnobInCell (pitchKnob,    margin + cellW * 4, row1Y);

    placeLabelInCell (positionValueLabel, margin + cellW * 0, row1Y);
    placeLabelInCell (densityValueLabel,  margin + cellW * 1, row1Y);
    placeLabelInCell (sizeValueLabel,     margin + cellW * 2, row1Y);
    placeLabelInCell (textureValueLabel,  margin + cellW * 3, row1Y);
    placeLabelInCell (pitchValueLabel,    margin + cellW * 4, row1Y);

    // ===== Row 2: Spread, Feedback, Reverb, Mix, Mode =====
    int row2Y = margin + rowH;
    placeKnobInCell (spreadKnob,   margin + cellW * 0, row2Y);
    placeKnobInCell (feedbackKnob, margin + cellW * 1, row2Y);
    placeKnobInCell (reverbKnob,   margin + cellW * 2, row2Y);
    placeKnobInCell (mixKnob,      margin + cellW * 3, row2Y);
    placeKnobInCell (modeKnob,     margin + cellW * 4, row2Y);

    placeLabelInCell (spreadValueLabel,   margin + cellW * 0, row2Y);
    placeLabelInCell (feedbackValueLabel, margin + cellW * 1, row2Y);
    placeLabelInCell (reverbValueLabel,   margin + cellW * 2, row2Y);
    placeLabelInCell (mixValueLabel,      margin + cellW * 3, row2Y);
    placeLabelInCell (modeValueLabel,     margin + cellW * 4, row2Y);

    // ===== Row 3: TRIG Rate (left), BPM/TAP display (center) =====
    int row3Y = margin + rowH * 2;

    // TRIG Rate knob (first cell)
    int trigRateKnobX = margin + (cellW - knobSize) / 2;
    int trigRateKnobY = row3Y + (row3H - knobSize - labelH) / 2;
    trigRateKnob.slider.setBounds (trigRateKnobX, trigRateKnobY, knobSize, knobSize);

    // TRIG Rate label
    int trigRateLabelY = row3Y + row3H - labelOffsetY;
    trigRateValueLabel.setBounds (margin, trigRateLabelY, cellW, labelH);

    // BPM/TAP display (centered in remaining area)
    int bpmAreaX = margin + cellW;  // After first column
    int bpmAreaW = cellW * 4;       // Remaining 4 columns
    int bpmX = bpmAreaX + (bpmAreaW - bpmSize) / 2;
    int bpmY = row3Y + (row3H - bpmSize) / 2;
    tapBpmLabel.setBounds (bpmX, bpmY, bpmSize, bpmSize);
    tapButton.setBounds (bpmX, bpmY, bpmSize, bpmSize);

    // ===== Button Row =====
    int btnRowY = margin + rowH * 2 + row3H;
    int btnW = scaled (UISize::buttonWidth);
    int btnH = scaled (UISize::buttonHeight);
    int btnY = btnRowY + (btnRowH - btnH) / 2;

    // Center all 5 buttons horizontally
    int totalBtnWidth = btnW * 5 + btnPadding * 8;  // 5 buttons + padding between
    int btnStartX = (scaled (UISize::baseWidth) - totalBtnWidth) / 2;
    int btnSpacing = btnW + btnPadding * 2;

    trigModeButton.setBounds (btnStartX + btnSpacing * 0, btnY, btnW, btnH);
    freezeButton.setBounds   (btnStartX + btnSpacing * 1, btnY, btnW, btnH);
    randomButton.setBounds   (btnStartX + btnSpacing * 2, btnY, btnW, btnH);
    killDryButton.setBounds  (btnStartX + btnSpacing * 3, btnY, btnW, btnH);
    killWetButton.setBounds  (btnStartX + btnSpacing * 4, btnY, btnW, btnH);
}

void CloudLikeGranularEditor::mouseDown (const juce::MouseEvent& /*event*/)
{
    // Tap functionality now handled by tapButton
}

void CloudLikeGranularEditor::mouseUp (const juce::MouseEvent& event)
{
    // Right-click shows scale selection menu
    if (event.mods.isPopupMenu())
    {
        juce::PopupMenu menu;
        menu.addSectionHeader ("UI Scale");

        for (int i = 0; i < UISize::numScales; ++i)
        {
            float scale = UISize::scales[i];
            int percent = static_cast<int> (scale * 100);
            juce::String label = juce::String (percent) + "%";

            bool isTicked = (std::abs (uiScale - scale) < 0.01f);
            menu.addItem (i + 1, label, true, isTicked);
        }

        menu.showMenuAsync (juce::PopupMenu::Options(),
            [this] (int result)
            {
                if (result > 0 && result <= UISize::numScales)
                {
                    setScale (UISize::scales[result - 1]);
                }
            });
    }
}

void CloudLikeGranularEditor::mouseMove (const juce::MouseEvent& /*event*/)
{
    // Cursor handling now done by tapButton directly
}

void CloudLikeGranularEditor::setScale (float newScale)
{
    if (std::abs (uiScale - newScale) < 0.01f)
        return;  // No change

    uiScale = newScale;

    // Resize window
    int newWidth = scaled (UISize::baseWidth);
    int newHeight = scaled (UISize::baseHeight);
    setSize (newWidth, newHeight);

    // resized() will be called automatically after setSize()
    repaint();
}
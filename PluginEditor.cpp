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

    // Setup knobs with their individual LookAndFeel instances
    setupKnob (modeKnob, "Mode", modeLookAndFeel.get());
    modeKnob.slider.setSliderStyle (juce::Slider::RotaryHorizontalVerticalDrag);
    modeKnob.slider.setTextBoxStyle (juce::Slider::TextBoxBelow, false, 60, 22);
    modeKnob.slider.setNumDecimalPlacesToDisplay(0);  // Display integers only

    setupKnob (positionKnob, "Position", positionLookAndFeel.get());
    setupKnob (sizeKnob,     "Size",     sizeLookAndFeel.get());
    setupKnob (pitchKnob,    "Pitch",    pitchLookAndFeel.get());
    setupKnob (densityKnob,  "Density",  densityLookAndFeel.get());
    setupKnob (textureKnob,  "Texture",  textureLookAndFeel.get());
    setupKnob (spreadKnob,   "Spread",   spreadLookAndFeel.get());
    setupKnob (feedbackKnob, "Feedback", feedbackLookAndFeel.get());
    setupKnob (reverbKnob,   "Reverb",   reverbLookAndFeel.get());
    setupKnob (mixKnob,      "Mix",      mixLookAndFeel.get());
    setupKnob (trigRateKnob, "Trig Rate", trigRateLookAndFeel.get());

    addAndMakeVisible (trigModeButton);
    addAndMakeVisible (freezeButton);
    addAndMakeVisible (randomButton);
    addAndMakeVisible (modeLabel);
    addAndMakeVisible (trigRateLabel);

    // Style mode label (E-Paper: ink blue for accent)
    modeLabel.setFont (juce::Font ("Courier New", 16.0f, juce::Font::bold));
    modeLabel.setColour (juce::Label::textColourId, uiColors.modeLabel);
    modeLabel.setJustificationType (juce::Justification::centred);

    // Style TRIG RATE label (E-Paper: shows current division)
    trigRateLabel.setFont (juce::Font ("Courier New", 14.0f, juce::Font::bold));
    trigRateLabel.setColour (juce::Label::textColourId, uiColors.knobLabel);
    trigRateLabel.setJustificationType (juce::Justification::centred);

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

    // Update mode label and knob labels based on mode parameter
    int mode = static_cast<int>(processor.apvts.getRawParameterValue ("mode")->load());
    juce::String modeText;

    // Define knob labels for each mode
    juce::String posLabel, sizeLabel, pitchLabel, densityLabel, textureLabel;

    switch (mode)
    {
        case 0: // Granular
            modeText = "MODE: Granular";
            posLabel = "Position";
            sizeLabel = "Size";
            pitchLabel = "Pitch";
            densityLabel = "Density";
            textureLabel = "Texture";
            break;

        case 1: // Pitch Shifter (Clouds-style)
            modeText = "MODE: Pitch Shifter";
            posLabel = "Position";
            sizeLabel = "Window";  // Small=Grainy, Large=Smooth
            pitchLabel = "Pitch";
            densityLabel = "Density";
            textureLabel = "Texture";
            break;

        case 2: // Looping
            modeText = "MODE: Looping";
            posLabel = "LoopPos";
            sizeLabel = "LoopLen";
            pitchLabel = "Speed";
            densityLabel = "Density";
            textureLabel = "Texture";
            break;

        case 3: // Spectral
            modeText = "MODE: Spectral";
            posLabel = "Delay";
            sizeLabel = "Window";
            pitchLabel = "FreqShft";
            densityLabel = "Density";
            textureLabel = "Texture";
            break;

        case 4: // Oliverb
            modeText = "MODE: Oliverb";
            posLabel = "ModRate";
            sizeLabel = "Decay";
            pitchLabel = "Pitch";
            densityLabel = "ModDepth";
            textureLabel = "Diffuse";
            break;

        case 5: // Resonestor
            modeText = "MODE: Resonestor";
            posLabel = "Excite";
            sizeLabel = "Decay";
            pitchLabel = "Pitch";
            densityLabel = "Pattern";
            textureLabel = "Bright";
            break;

        case 6: // Beat Repeat
            modeText = "MODE: Beat Repeat";
            posLabel = "Capture";
            sizeLabel = "Length";
            pitchLabel = "Speed";
            densityLabel = "Rate";
            textureLabel = "Stutter";
            break;

        default:
            modeText = "MODE: Unknown";
            posLabel = "Position";
            sizeLabel = "Size";
            pitchLabel = "Pitch";
            densityLabel = "Density";
            textureLabel = "Texture";
            break;
    }

    // Update mode label
    if (modeLabel.getText() != modeText)
    {
        modeLabel.setText (modeText, juce::dontSendNotification);
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

    // Update TRIG RATE label to show current division
    float trigRate = processor.apvts.getRawParameterValue ("trigRate")->load();
    juce::String trigRateText;

    if (trigRate < -3.4f)      trigRateText = "1/16";
    else if (trigRate < -2.8f) trigRateText = "1/16T";
    else if (trigRate < -2.2f) trigRateText = "1/8";
    else if (trigRate < -1.6f) trigRateText = "1/8T";
    else if (trigRate < -0.8f) trigRateText = "1/4";
    else if (trigRate < 0.0f)  trigRateText = "1/4T";
    else if (trigRate < 0.8f)  trigRateText = "1/2";
    else if (trigRate < 1.6f)  trigRateText = "1/2T";
    else if (trigRate < 2.4f)  trigRateText = "1 bar";
    else if (trigRate < 3.2f)  trigRateText = "1 barT";
    else                       trigRateText = "2 bars";

    if (trigRateLabel.getText() != trigRateText)
    {
        trigRateLabel.setText (trigRateText, juce::dontSendNotification);
    }

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

void CloudLikeGranularEditor::setupKnob (Knob& k, const juce::String& name, EPaperLookAndFeel* lookAndFeel)
{
    addAndMakeVisible (k.slider);
    addAndMakeVisible (k.label);

    k.slider.setSliderStyle (juce::Slider::RotaryHorizontalVerticalDrag);
    k.slider.setTextBoxStyle (juce::Slider::TextBoxBelow, false, 60, 22);
    k.slider.setPopupDisplayEnabled (true, false, this);

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

    // Draw separator line below mode label
    g.setColour (juce::Colour::fromRGB (224, 224, 224));
    g.fillRect (10, 45, getWidth() - 20, 1);

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
    g.drawEllipse (ledX1, ledY, ledSize, ledSize, 1.5f);

    // LED 2: TRIG RATE tempo
    g.setColour (juce::Colour::fromRGB (200, 200, 200));  // Off color (light gray)
    g.fillEllipse (ledX2, ledY, ledSize, ledSize);
    if (trigRateLedOn)
    {
        g.setColour (juce::Colour::fromRGB (180, 100, 100));  // On color (red)
        g.fillEllipse (ledX2 + 2, ledY + 2, ledSize - 4, ledSize - 4);
    }
    g.setColour (uiColors.buttonText);
    g.drawEllipse (ledX2, ledY, ledSize, ledSize, 1.5f);

    // LED labels
    g.setColour (uiColors.knobLabel);
    g.setFont (juce::Font ("Courier New", 9.0f, juce::Font::plain));
    g.drawText ("×1", ledX1 - 5, ledY + ledSize + 2, ledSize + 10, 12, juce::Justification::centred);
    g.drawText ("TRIG", ledX2 - 8, ledY + ledSize + 2, ledSize + 16, 12, juce::Justification::centred);
}

void CloudLikeGranularEditor::resized()
{
    auto area = getLocalBounds().reduced (10);

    // E-Paper UI: Mode label at top center
    auto titleArea = area.removeFromTop (35);
    modeLabel.setBounds (titleArea.withTrimmedLeft (titleArea.getWidth() / 4)
                                   .withTrimmedRight (titleArea.getWidth() / 4));

    // E-Paper UI: 3 rows × 5 columns grid (added 3rd row for TRIG rate)
    auto buttonRow = area.removeFromBottom (50);
    auto knobArea = area.reduced (0, 10);

    auto rowHeight = knobArea.getHeight() / 3;
    auto row1 = knobArea.removeFromTop (rowHeight);
    auto row2 = knobArea.removeFromTop (rowHeight);
    auto row3 = knobArea;

    auto colWidth = row1.getWidth() / 5;

    auto placeKnob = [] (Knob& k, juce::Rectangle<int> r)
    {
        // Reduce knob size to half by increasing margin
        k.slider.setBounds (r.reduced (30));
    };

    // Row 1: Position, Density, Size, Texture, Pitch
    placeKnob (positionKnob, row1.removeFromLeft (colWidth));
    placeKnob (densityKnob,  row1.removeFromLeft (colWidth));
    placeKnob (sizeKnob,     row1.removeFromLeft (colWidth));
    placeKnob (textureKnob,  row1.removeFromLeft (colWidth));
    placeKnob (pitchKnob,    row1.removeFromLeft (colWidth));

    // Row 2: Spread, Feedback, Reverb, Mix, Mode
    placeKnob (spreadKnob,   row2.removeFromLeft (colWidth));
    placeKnob (feedbackKnob, row2.removeFromLeft (colWidth));
    placeKnob (reverbKnob,   row2.removeFromLeft (colWidth));
    placeKnob (mixKnob,      row2.removeFromLeft (colWidth));
    placeKnob (modeKnob,     row2.removeFromLeft (colWidth));

    // Row 3: TRIG Rate (centered, single knob)
    auto trigRateArea = row3.withSizeKeepingCentre (colWidth, rowHeight);
    placeKnob (trigRateKnob, trigRateArea);

    // TRIG RATE label (shows current division: 1/16, 1/8T, etc.)
    auto trigRateLabelArea = trigRateArea.withHeight (20).withY (trigRateArea.getBottom() + 35);
    trigRateLabel.setBounds (trigRateLabelArea);

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
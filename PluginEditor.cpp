// PluginEditor.cpp
#include "PluginEditor.h"

//==============================================================================
CloudLikeGranularEditor::CloudLikeGranularEditor (CloudLikeGranularProcessor& p)
    : AudioProcessorEditor (&p)
    , processor (p)
    , lookAndFeel (std::make_unique<EPaperLookAndFeel>())
{
    setSize (UISize::baseWidth, UISize::baseHeight);
    startTimerHz (30);

    // Setup all knobs using array
    for (int i = 0; i < kNumKnobs; ++i)
    {
        setupKnob (static_cast<KnobId>(i), knobNames[i]);
        setupValueLabel (valueLabels[i]);
        addAndMakeVisible (valueLabels[i]);
    }

    // Setup buttons
    addAndMakeVisible (trigModeButton);
    addAndMakeVisible (freezeButton);
    addAndMakeVisible (randomButton);
    addAndMakeVisible (killDryButton);
    addAndMakeVisible (killWetButton);
    addAndMakeVisible (tapBpmLabel);
    addAndMakeVisible (tapButton);

    // TrigMode toggle switch style
    trigModeButton.setComponentID ("toggleSwitch");
    trigModeButton.setLookAndFeel (lookAndFeel.get());

    // Freeze button style
    freezeButton.setColour (juce::ToggleButton::textColourId, uiColors.freezeTextOff);
    freezeButton.setLookAndFeel (lookAndFeel.get());

    // Randomize button
    randomButton.setColour (juce::TextButton::buttonColourId, uiColors.buttonBackground);
    randomButton.setColour (juce::TextButton::textColourOffId, uiColors.buttonText);
    randomButton.setLookAndFeel (lookAndFeel.get());
    randomButton.onClick = [this]
    {
        if (auto* param = processor.apvts.getParameter ("randomize"))
        {
            param->beginChangeGesture();
            param->setValueNotifyingHost (1.0f);
            param->endChangeGesture();
            param->setValueNotifyingHost (0.0f);
        }
    };

    // Kill Dry/Wet buttons (momentary)
    setupMomentaryButton (killDryButton, "killDry");
    setupMomentaryButton (killWetButton, "killWet");

    // Tap BPM label
    tapBpmLabel.setFont (juce::Font ("Courier New", 10.0f, juce::Font::bold));
    tapBpmLabel.setColour (juce::Label::textColourId, uiColors.knobLabel);
    tapBpmLabel.setJustificationType (juce::Justification::centred);
    tapBpmLabel.setText ("---", juce::dontSendNotification);
    tapBpmLabel.setInterceptsMouseClicks (false, false);

    // Tap button (transparent overlay)
    tapButton.setButtonText ("");
    tapButton.setColour (juce::TextButton::buttonColourId, juce::Colours::transparentWhite);
    tapButton.setColour (juce::TextButton::buttonOnColourId, juce::Colours::transparentWhite);
    tapButton.setColour (juce::ComboBox::outlineColourId, juce::Colours::transparentWhite);
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

    // Create slider attachments
    auto& apvts = processor.apvts;
    for (int i = 0; i < kNumKnobs; ++i)
    {
        sliderAttachments[i] = std::make_unique<SliderAttachment> (apvts, paramIds[i], knobs[i].slider);
    }

    trigModeAttachment = std::make_unique<ButtonAttachment> (apvts, "trigMode", trigModeButton);
    freezeAttachment = std::make_unique<ButtonAttachment> (apvts, "freeze", freezeButton);
}

CloudLikeGranularEditor::~CloudLikeGranularEditor()
{
    stopTimer();

    // Clear LookAndFeel references
    for (auto& k : knobs)
        k.slider.setLookAndFeel (nullptr);

    trigModeButton.setLookAndFeel (nullptr);
    freezeButton.setLookAndFeel (nullptr);
    randomButton.setLookAndFeel (nullptr);
    killDryButton.setLookAndFeel (nullptr);
    killWetButton.setLookAndFeel (nullptr);
}

//==============================================================================
// Setup helper functions

void CloudLikeGranularEditor::setupKnob (KnobId id, const juce::String& name)
{
    auto& k = knobs[static_cast<int>(id)];

    k.slider.setSliderStyle (juce::Slider::RotaryHorizontalVerticalDrag);
    k.slider.setTextBoxStyle (juce::Slider::NoTextBox, false, 0, 0);
    k.slider.setLookAndFeel (lookAndFeel.get());

    // Register slider for color lookup
    lookAndFeel->registerSlider (&k.slider, id);

    k.label.setText (name, juce::dontSendNotification);
    k.label.attachToComponent (&k.slider, false);
    k.label.setJustificationType (juce::Justification::centred);
    k.label.setColour (juce::Label::textColourId, uiColors.knobLabel);
    k.label.setFont (juce::Font ("Courier New", 12.0f, juce::Font::plain));

    addAndMakeVisible (k.slider);
    addAndMakeVisible (k.label);
}

void CloudLikeGranularEditor::setupMomentaryButton (juce::TextButton& button, const juce::String& paramId)
{
    button.setColour (juce::TextButton::buttonColourId, uiColors.buttonBackground);
    button.setColour (juce::TextButton::textColourOffId, uiColors.buttonText);
    button.setColour (juce::TextButton::textColourOnId, uiColors.momentaryOnColor);
    button.setLookAndFeel (lookAndFeel.get());

    button.onStateChange = [this, &button, paramId]
    {
        if (auto* param = processor.apvts.getParameter (paramId))
        {
            bool isDown = button.isDown();
            param->beginChangeGesture();
            param->setValueNotifyingHost (isDown ? 1.0f : 0.0f);
            param->endChangeGesture();
        }
    };
}

void CloudLikeGranularEditor::setupValueLabel (juce::Label& label)
{
    label.setFont (juce::Font ("Courier New", 11.0f, juce::Font::plain));
    label.setColour (juce::Label::textColourId, uiColors.knobLabel);
    label.setJustificationType (juce::Justification::centred);
}

//==============================================================================
void CloudLikeGranularEditor::timerCallback()
{
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
    if (lookAndFeel->forceMaxPosition != killDry)
    {
        lookAndFeel->forceMaxPosition = killDry;
        needsRepaint = true;
    }
    if (lookAndFeel->forceMinPosition != killWet)
    {
        lookAndFeel->forceMinPosition = killWet;
        needsRepaint = true;
    }
    if (needsRepaint)
        knob(KnobId::Mix).slider.repaint();

    updateButtonStates(trigMode);
    updateModeLabels(mode);
    updateTrigRateLabel(trigRate);
    updateKnobValueLabels(mode, position, size, pitch, density, texture, spread, feedback, reverb, mix);
    updateLedIndicators();
    updateBpmDisplay(trigMode);
}

//==============================================================================
void CloudLikeGranularEditor::updateButtonStates(bool trigMode)
{
    // Toggle switch reads state directly, just ensure state is synced
    if (trigModeButton.getToggleState() != trigMode)
    {
        trigModeButton.setToggleState(trigMode, juce::dontSendNotification);
        trigModeButton.repaint();
    }

    bool freezeState = processor.apvts.getRawParameterValue("freeze")->load() > 0.5f;
    if (freezeButton.getToggleState() != freezeState)
    {
        freezeButton.setToggleState(freezeState, juce::dontSendNotification);
        repaint();
    }

    juce::Colour freezeTextColor = freezeState ? uiColors.freezeTextOn : uiColors.freezeTextOff;
    if (freezeButton.findColour(juce::ToggleButton::textColourId) != freezeTextColor)
        freezeButton.setColour(juce::ToggleButton::textColourId, freezeTextColor);

    // Update Kill Dry/Wet button text colors
    bool killDryState = processor.apvts.getRawParameterValue("killDry")->load() > 0.5f;
    bool killWetState = processor.apvts.getRawParameterValue("killWet")->load() > 0.5f;

    juce::Colour killDryTextColor = killDryState ? uiColors.momentaryOnColor : uiColors.buttonText;
    if (killDryButton.findColour(juce::TextButton::textColourOffId) != killDryTextColor)
    {
        killDryButton.setColour(juce::TextButton::textColourOffId, killDryTextColor);
        killDryButton.repaint();
    }

    juce::Colour killWetTextColor = killWetState ? uiColors.momentaryOnColor : uiColors.buttonText;
    if (killWetButton.findColour(juce::TextButton::textColourOffId) != killWetTextColor)
    {
        killWetButton.setColour(juce::TextButton::textColourOffId, killWetTextColor);
        killWetButton.repaint();
    }

    // Update tap button visibility
    if (!trigMode)
    {
        if (!tapButton.isVisible())
            tapButton.setVisible(true);
        tapButton.setMouseCursor(juce::MouseCursor::PointingHandCursor);
    }
    else
    {
        if (tapButton.isVisible())
            tapButton.setVisible(false);
    }
}

void CloudLikeGranularEditor::updateModeLabels(int mode)
{
    static const char* modeNames[] = {
        "GranProc", "Pitch/Time", "LoopDly", "SpctrlMad",
        "Oliverb", "Resonstr", "BeatRpt", "SpctrlCld"
    };

    juce::String modeValueText = (mode >= 0 && mode < 8) ? modeNames[mode] : "Unknown";
    if (valueLabel(KnobId::Mode).getText() != modeValueText)
        valueLabel(KnobId::Mode).setText(modeValueText, juce::dontSendNotification);

    // Define knob labels for each mode
    static const char* posLabels[] = { "Position", "Position", "LoopPos", "Delay", "ModRate", "Excite", "Capture", "Filter" };
    static const char* sizeLabels[] = { "Size", "Window", "LoopLen", "Window", "Decay", "Decay", "Length", "Bands" };
    static const char* pitchLabels[] = { "Pitch", "Pitch", "Speed", "FreqShft", "Pitch", "Pitch", "Speed", "Pitch" };
    static const char* densLabels[] = { "Density", "Density", "Density", "Density", "ModDepth", "Pattern", "Rate", "Smooth" };
    static const char* texLabels[] = { "Texture", "Texture", "Texture", "Texture", "Diffuse", "Bright", "Stutter", "Phase" };

    int m = (mode >= 0 && mode < 8) ? mode : 0;
    auto& posKnob = knob(KnobId::Position);
    auto& sizeKnob = knob(KnobId::Size);
    auto& pitchKnob = knob(KnobId::Pitch);
    auto& densKnob = knob(KnobId::Density);
    auto& texKnob = knob(KnobId::Texture);

    if (posKnob.label.getText() != posLabels[m]) posKnob.label.setText(posLabels[m], juce::dontSendNotification);
    if (sizeKnob.label.getText() != sizeLabels[m]) sizeKnob.label.setText(sizeLabels[m], juce::dontSendNotification);
    if (pitchKnob.label.getText() != pitchLabels[m]) pitchKnob.label.setText(pitchLabels[m], juce::dontSendNotification);
    if (densKnob.label.getText() != densLabels[m]) densKnob.label.setText(densLabels[m], juce::dontSendNotification);
    if (texKnob.label.getText() != texLabels[m]) texKnob.label.setText(texLabels[m], juce::dontSendNotification);
}

void CloudLikeGranularEditor::updateTrigRateLabel(float trigRate)
{
    juce::String trigRateText;
    if (trigRate < -3.4f)      trigRateText = "1/16";
    else if (trigRate < -2.8f) trigRateText = "1/16T";
    else if (trigRate < -2.2f) trigRateText = "1/8";
    else if (trigRate < -1.6f) trigRateText = "1/8T";
    else if (trigRate < -0.8f) trigRateText = "1/4";
    else if (trigRate < 0.0f)  trigRateText = "1/4T";
    else if (trigRate < 0.8f)  trigRateText = "1/2";
    else if (trigRate < 1.6f)  trigRateText = "1/2T";
    else if (trigRate < 2.4f)  trigRateText = "1bar";
    else if (trigRate < 3.2f)  trigRateText = "1barT";
    else                       trigRateText = "2bars";

    if (valueLabel(KnobId::TrigRate).getText() != trigRateText)
        valueLabel(KnobId::TrigRate).setText(trigRateText, juce::dontSendNotification);
}

void CloudLikeGranularEditor::updateKnobValueLabels(int mode, float position, float size, float pitch,
                                                     float density, float texture, float spread,
                                                     float feedback, float reverb, float mix)
{
    // Position
    juce::String posText = juce::String(static_cast<int>(position * 100)) + "%";
    if (mode == 4) posText = juce::String(position * 10.0f, 1) + "Hz";

    // Size
    juce::String sizeText;
    if (mode == 7)
        sizeText = juce::String(4 + static_cast<int>(size * 60));
    else if (mode == 4 || mode == 5)
        sizeText = juce::String(static_cast<int>((size - 0.016f) / (1.0f - 0.016f) * 98.0f + 2.0f)) + "%";
    else
        sizeText = juce::String(static_cast<int>(size * 1000.0f)) + "ms";

    // Pitch
    juce::String pitchText;
    if (mode == 2 || mode == 6)
        pitchText = juce::String(std::pow(2.0f, pitch / 12.0f), 2) + "x";
    else
        pitchText = (pitch >= 0 ? "+" : "") + juce::String(static_cast<int>(pitch)) + "st";

    juce::String densText = juce::String(static_cast<int>(density * 100)) + "%";
    juce::String texText = juce::String(static_cast<int>(texture * 100)) + "%";
    juce::String spreadText = juce::String(static_cast<int>(spread * 100)) + "%";
    juce::String fbText = juce::String(static_cast<int>(feedback * 100)) + "%";
    juce::String revText = juce::String(static_cast<int>(reverb * 100)) + "%";

    bool killDry = processor.apvts.getRawParameterValue("killDry")->load() > 0.5f;
    bool killWet = processor.apvts.getRawParameterValue("killWet")->load() > 0.5f;
    juce::String mixText = killDry ? "100%" : (killWet ? "0%" : juce::String(static_cast<int>(mix * 100)) + "%");

    if (valueLabel(KnobId::Position).getText() != posText) valueLabel(KnobId::Position).setText(posText, juce::dontSendNotification);
    if (valueLabel(KnobId::Size).getText() != sizeText) valueLabel(KnobId::Size).setText(sizeText, juce::dontSendNotification);
    if (valueLabel(KnobId::Pitch).getText() != pitchText) valueLabel(KnobId::Pitch).setText(pitchText, juce::dontSendNotification);
    if (valueLabel(KnobId::Density).getText() != densText) valueLabel(KnobId::Density).setText(densText, juce::dontSendNotification);
    if (valueLabel(KnobId::Texture).getText() != texText) valueLabel(KnobId::Texture).setText(texText, juce::dontSendNotification);
    if (valueLabel(KnobId::Spread).getText() != spreadText) valueLabel(KnobId::Spread).setText(spreadText, juce::dontSendNotification);
    if (valueLabel(KnobId::Feedback).getText() != fbText) valueLabel(KnobId::Feedback).setText(fbText, juce::dontSendNotification);
    if (valueLabel(KnobId::Reverb).getText() != revText) valueLabel(KnobId::Reverb).setText(revText, juce::dontSendNotification);
    if (valueLabel(KnobId::Mix).getText() != mixText) valueLabel(KnobId::Mix).setText(mixText, juce::dontSendNotification);
}

void CloudLikeGranularEditor::updateLedIndicators()
{
    // Base tempo LED
    if (processor.baseTempoBlink.load())
    {
        processor.baseTempoBlink.store(false);
        baseTempoLedOn = true;
        ledBlinkDuration = 3;
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

    // TRIG rate LED
    bool trigMode = processor.apvts.getRawParameterValue("trigMode")->load() > 0.5f;

    if (!trigMode)
    {
        bool noteHeld = processor.midiNoteHeld.load();
        if (processor.trigRateBlink.load())
        {
            processor.trigRateBlink.store(false);
            trigRateLedOn = true;
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
        if (processor.trigRateBlink.load())
        {
            processor.trigRateBlink.store(false);
            trigRateLedOn = true;
            ledBlinkDuration2 = 3;
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
    juce::String bpmText = "---";
    if (trigMode)
    {
        float hostBpm = processor.hostBPM.load();
        if (hostBpm > 0.0f)
            bpmText = juce::String(static_cast<int>(hostBpm));
    }
    if (tapBpmLabel.getText() != bpmText)
        tapBpmLabel.setText(bpmText, juce::dontSendNotification);
}

//==============================================================================
void CloudLikeGranularEditor::paint (juce::Graphics& g)
{
    g.fillAll (uiColors.background);

    const int margin = scaled (UISize::windowMargin);
    const float ledLabelFontSize = scaledF (static_cast<float>(UISize::ledLabelFontSize));
    const float bpmFontSize = scaledF (static_cast<float>(UISize::bpmFontSize));
    const float glowScale = scaledF (3.0f);

    // Draw border (same color as MIX knob outline)
    g.setColour (uiColors.getKnobColors(KnobId::Mix).outline);
    g.drawRect (getLocalBounds(), scaled (2));

    // Draw separator line above buttons
    int separatorY = getHeight() - scaled (UISize::buttonRowHeight) - margin;
    g.setColour (juce::Colour::fromRGB (224, 224, 224));
    g.fillRect (margin, separatorY, getWidth() - margin * 2, scaled (1));

    // Draw freeze button border when active
    if (freezeButton.getToggleState())
    {
        auto freezeBounds = freezeButton.getBounds().toFloat().reduced (scaledF (1.0f));
        g.setColour (uiColors.buttonText);
        g.drawRoundedRectangle (freezeBounds, 0.0f, scaledF (2.0f));
    }

    // Draw BPM/TAP display
    auto bpmBounds = tapBpmLabel.getBounds().toFloat();
    bool trigMode = processor.apvts.getRawParameterValue ("trigMode")->load() > 0.5f;
    bool isManualMode = !trigMode;

    if (baseTempoLedOn)
    {
        auto glowColor = juce::Colour::fromRGB (100, 180, 100);
        for (int i = 4; i >= 1; --i)
        {
            g.setColour (glowColor.withAlpha (0.15f / i));
            g.fillEllipse (bpmBounds.expanded (i * glowScale));
        }
        g.setColour (glowColor);
    }
    else if (isManualMode)
    {
        g.setColour (juce::Colour::fromRGB (250, 250, 250));
    }
    else
    {
        g.setColour (juce::Colour::fromRGB (230, 230, 230));
    }
    g.fillEllipse (bpmBounds);

    g.setColour (uiColors.getKnobColors(KnobId::Mix).outline);
    g.drawEllipse (bpmBounds, isManualMode ? scaledF (1.25f) : scaledF (0.75f));

    // BPM/TAP label
    auto bpmLabelArea = bpmBounds.withY (bpmBounds.getBottom() + scaledF (2.0f)).withHeight (scaledF (15.0f));
    g.setFont (juce::Font ("Courier New", bpmFontSize, juce::Font::plain));
    g.setColour (uiColors.knobLabel);
    g.drawText (isManualMode ? "TAP" : "BPM", bpmLabelArea, juce::Justification::centred);

    // Draw TRIG LED
    float ledSize = scaledF (static_cast<float>(UISize::ledDiameter));
    float ledSpacing = scaledF (static_cast<float>(UISize::ledSpacing));
    float ledX = bpmBounds.getRight() + ledSpacing;
    float ledY = bpmBounds.getCentreY() - ledSize / 2.0f;
    auto trigLedBounds = juce::Rectangle<float> (ledX, ledY, ledSize, ledSize);

    if (trigRateLedOn)
    {
        auto glowColor = juce::Colour::fromRGB (180, 100, 100);
        for (int i = 4; i >= 1; --i)
        {
            g.setColour (glowColor.withAlpha (0.15f / i));
            g.fillEllipse (trigLedBounds.expanded (i * glowScale));
        }
        g.setColour (glowColor);
    }
    else
    {
        g.setColour (juce::Colour::fromRGB (200, 200, 200));
    }
    g.fillEllipse (trigLedBounds);

    g.setColour (uiColors.getKnobColors(KnobId::Mix).outline);
    g.drawEllipse (trigLedBounds, scaledF (0.75f));

    // TRIG label
    g.setColour (uiColors.knobLabel);
    g.setFont (juce::Font ("Courier New", ledLabelFontSize, juce::Font::plain));
    int labelX = static_cast<int>(ledX - scaledF (8.0f));
    int labelY = static_cast<int>(ledY + ledSize + scaledF (2.0f));
    g.drawText ("TRIG", labelX, labelY, static_cast<int>(ledSize + scaledF (16.0f)), scaled (12), juce::Justification::centred);
}

//==============================================================================
void CloudLikeGranularEditor::resized()
{
    // Direct specification approach - use knob size and gap directly
    // Layout: 4 knobs per row, compact buttons
    const int margin = scaled (UISize::windowMargin);
    const int knobSize = scaled (UISize::knobDiameter);
    const int knobGap = scaled (UISize::knobGap);
    const int rowH = scaled (UISize::knobRowHeight);
    const int btnRowH = scaled (UISize::buttonRowHeight);
    const int labelH = scaled (UISize::labelHeight);
    const int btnPadding = scaled (UISize::buttonPadding);
    const int bpmSize = scaled (UISize::bpmDisplaySize);

    // Knob step = knob size + gap between knobs
    const int knobStep = knobSize + knobGap;

    // Helper: place knob at column index (0-3) in a row
    auto placeKnob = [&](KnobId id, int col, int rowY)
    {
        int knobX = margin + col * knobStep;
        int knobY = rowY + 10;  // Small top padding
        knob(id).slider.setBounds (knobX, knobY, knobSize, knobSize);
    };

    // Helper: place value label below knob
    auto placeLabel = [&](KnobId id, int col, int rowY)
    {
        int labelX = margin + col * knobStep - knobGap / 2;
        int labelY = rowY + 10 + knobSize + 2;
        int labelW = knobSize + knobGap;
        valueLabel(id).setBounds (labelX, labelY, labelW, labelH);
    };

    // Row 1: Position, Density, Size, Texture (4 knobs)
    int row1Y = margin;
    placeKnob (KnobId::Position, 0, row1Y);
    placeKnob (KnobId::Density,  1, row1Y);
    placeKnob (KnobId::Size,     2, row1Y);
    placeKnob (KnobId::Texture,  3, row1Y);

    placeLabel (KnobId::Position, 0, row1Y);
    placeLabel (KnobId::Density,  1, row1Y);
    placeLabel (KnobId::Size,     2, row1Y);
    placeLabel (KnobId::Texture,  3, row1Y);

    // Row 2: Pitch, Spread, Feedback, Reverb (4 knobs)
    int row2Y = margin + rowH;
    placeKnob (KnobId::Pitch,    0, row2Y);
    placeKnob (KnobId::Spread,   1, row2Y);
    placeKnob (KnobId::Feedback, 2, row2Y);
    placeKnob (KnobId::Reverb,   3, row2Y);

    placeLabel (KnobId::Pitch,    0, row2Y);
    placeLabel (KnobId::Spread,   1, row2Y);
    placeLabel (KnobId::Feedback, 2, row2Y);
    placeLabel (KnobId::Reverb,   3, row2Y);

    // Row 3: Mix, Mode, TrigRate + BPM display (3 knobs + BPM)
    int row3Y = margin + rowH * 2;
    placeKnob (KnobId::Mix,      0, row3Y);
    placeKnob (KnobId::Mode,     1, row3Y);
    placeKnob (KnobId::TrigRate, 2, row3Y);

    placeLabel (KnobId::Mix,      0, row3Y);
    placeLabel (KnobId::Mode,     1, row3Y);
    placeLabel (KnobId::TrigRate, 2, row3Y);

    // BPM/TAP display - in 4th column position
    int bpmX = margin + knobStep * 3 + (knobSize - bpmSize) / 2;
    int bpmY = row3Y + 10 + (knobSize - bpmSize) / 2;
    tapBpmLabel.setBounds (bpmX, bpmY, bpmSize, bpmSize);
    tapButton.setBounds (bpmX, bpmY, bpmSize, bpmSize);

    // Button row (compact layout)
    int btnRowY = margin + rowH * 3;
    int btnW = scaled (UISize::buttonWidth);
    int btnWSmall = scaled (UISize::buttonWidthSmall);
    int btnH = scaled (UISize::buttonHeight);
    int btnY = btnRowY + (btnRowH - btnH) / 2;

    // Toggle switch dimensions
    int toggleW = scaled (UISize::toggleWidth);
    int toggleH = scaled (UISize::toggleHeight);
    int toggleY = btnRowY + (btnRowH - toggleH) / 2;

    // Calculate compact layout: toggle + Freeze + Rnd(small) + KillDry + KillWet
    int totalBtnWidth = toggleW + btnW * 3 + btnWSmall + btnPadding * 8;
    int btnStartX = (scaled (UISize::baseWidth) - totalBtnWidth) / 2;

    // Place toggle switch first
    trigModeButton.setBounds (btnStartX, toggleY, toggleW, toggleH);

    // Place remaining buttons after toggle (compact spacing)
    int curX = btnStartX + toggleW + btnPadding * 2;
    freezeButton.setBounds (curX, btnY, btnW, btnH);
    curX += btnW + btnPadding * 2;
    randomButton.setBounds (curX, btnY, btnWSmall, btnH);
    curX += btnWSmall + btnPadding * 2;
    killDryButton.setBounds (curX, btnY, btnW, btnH);
    curX += btnW + btnPadding * 2;
    killWetButton.setBounds (curX, btnY, btnW, btnH);
}

//==============================================================================
void CloudLikeGranularEditor::mouseDown (const juce::MouseEvent&)
{
}

void CloudLikeGranularEditor::mouseUp (const juce::MouseEvent& event)
{
    if (event.mods.isPopupMenu())
    {
        juce::PopupMenu menu;
        menu.addSectionHeader ("UI Scale");

        for (int i = 0; i < UISize::numScales; ++i)
        {
            float scale = UISize::scales[i];
            int percent = static_cast<int>(scale * 100);
            bool isTicked = (std::abs(uiScale - scale) < 0.01f);
            menu.addItem (i + 1, juce::String(percent) + "%", true, isTicked);
        }

        menu.showMenuAsync (juce::PopupMenu::Options(),
            [this](int result)
            {
                if (result > 0 && result <= UISize::numScales)
                    setScale (UISize::scales[result - 1]);
            });
    }
}

void CloudLikeGranularEditor::mouseMove (const juce::MouseEvent&)
{
}

void CloudLikeGranularEditor::setScale (float newScale)
{
    if (std::abs(uiScale - newScale) < 0.01f)
        return;

    uiScale = newScale;
    setSize (scaled (UISize::baseWidth), scaled (UISize::baseHeight));
    repaint();
}

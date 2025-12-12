// Chatbot.js
import React, { useState, useEffect } from 'react';
import ChatButton from './ChatButton';
import ChatWindow from './ChatWindow';
import SelectionTooltip from './SelectionTooltip';

function Chatbot({ selectedText, setSelectedText }) {
  const [isOpen, setIsOpen] = useState(false);
  const [tooltipVisible, setTooltipVisible] = useState(false);
  const [tooltipPos, setTooltipPos] = useState({ top: 0, left: 0 });

  // Show tooltip after selection completes
  useEffect(() => {
    const handleMouseUp = () => {
      try {
        const selection = window.getSelection();
        const text = selection ? selection.toString().trim() : '';

        // Hide tooltip if no valid selection
        if (!text || text.length < 3) {
          setTooltipVisible(false);
          setSelectedText('');
          return;
        }

        setSelectedText(text);

        // Get bounding rect for tooltip position
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        setTooltipPos({
          top: window.scrollY + rect.top - 10, // slightly above
          left: rect.left + rect.width / 2,
        });

        setTooltipVisible(true);
      } catch (err) {
        // safe fallback
        setTooltipVisible(false);
        setSelectedText('');
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    return () => document.removeEventListener('mouseup', handleMouseUp);
  }, [setSelectedText]);

  // Auto hide tooltip on scroll
  useEffect(() => {
    const hide = () => setTooltipVisible(false);
    window.addEventListener('scroll', hide);
    return () => window.removeEventListener('scroll', hide);
  }, []);

  const handleTooltipClick = () => {
    if (!selectedText) return;

    setChatbotOpen(true); // Chatbot open karo
    setTimeout(() => {
      handleSendMessage(selectedText); // Auto message send
    }, 150); // thoda delay taake chatbot open ho jaye
  };

  // IMPORTANT: open chat AND auto-send preserved selection
  const openChatWithSelection = () => {
    // Preserve the text immediately — do NOT rely on selectedText later
    const textToExplain = selectedText?.trim();

    if (!textToExplain || textToExplain.length === 0) return;

    // Open chat first so ChatWindow mounts
    setIsOpen(true);

    // Hide tooltip (this may clear browser selection, but we've preserved text)
    setTooltipVisible(false);

    // Small delay to allow ChatWindow to render. Use preserved text variable.
    setTimeout(() => {
      window.dispatchEvent(
        new CustomEvent('AUTO_EXPLAIN_SELECTED_TEXT', {
          detail: textToExplain,
        })
      );

      // Clear shared selectedText state so UI in ChatWindow shows empty selection area
      setSelectedText('');
    }, 200); // 150-250ms is fine; 200ms is safe
  };

  // Normal toggle
  const toggleChat = () => {
    setIsOpen((v) => !v);
    setTooltipVisible(false);
  };

  return (
    <>
      <SelectionTooltip
        visible={tooltipVisible}
        position={tooltipPos}
        onClick={openChatWithSelection}
      />

      <ChatButton onClick={toggleChat} />

      <ChatWindow
        isOpen={isOpen}
        onClose={() => {
          setIsOpen(false);
          setTooltipVisible(false);
        }}
        selectedText={selectedText}
        setSelectedText={setSelectedText}
      />
    </>
  );
}

export default Chatbot;

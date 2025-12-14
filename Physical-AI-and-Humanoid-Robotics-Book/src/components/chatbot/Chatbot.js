// Chatbot.js
import React, { useState, useEffect } from 'react';
import ChatButton from './ChatButton';
import ChatWindow from './ChatWindow';
import SelectionTooltip from './SelectionTooltip';

function Chatbot({ selectedText, setSelectedText }) {
  const [isOpen, setIsOpen] = useState(false);
  const [tooltipVisible, setTooltipVisible] = useState(false);
  const [tooltipPos, setTooltipPos] = useState({ top: 0, left: 0 });

  const isMobileDevice = () => {
    if (typeof window === 'undefined') return false;
    return 'ontouchstart' in window || navigator.maxTouchPoints > 0;
  };

  // ✅ Desktop + Mobile text selection handler
  useEffect(() => {
    // 🔹 SHOW tooltip only when selection is COMPLETE
    const handleSelectionEnd = () => {
      try {
        const selection = window.getSelection();
        if (!selection) return;

        const text = selection.toString().trim();

        if (!text || text.length < 3) {
          setTooltipVisible(false);
          setSelectedText('');
          return;
        }

        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        const isMobile = 'ontouchstart' in window || navigator.maxTouchPoints > 0;

        setSelectedText(text);

        setTooltipPos({
          top: isMobile
            ? window.scrollY + rect.bottom + 12 // 📱 mobile → neeche
            : window.scrollY + rect.top - 10, // 💻 desktop → upar
          left: rect.left + rect.width / 2,
        });

        setTooltipVisible(true);
      } catch (err) {
        setTooltipVisible(false);
        setSelectedText('');
      }
    };

    // 🔹 HIDE tooltip instantly when selection changes / clears
    const handleSelectionChange = () => {
      const selection = window.getSelection();
      if (!selection) return;

      const text = selection.toString().trim();

      if (!text || text.length < 3) {
        setTooltipVisible(false);
        setSelectedText('');
      }
    };

    // Desktop
    document.addEventListener('mouseup', handleSelectionEnd);
    // Mobile
    document.addEventListener('touchend', handleSelectionEnd);
    // IMPORTANT: hide only
    document.addEventListener('selectionchange', handleSelectionChange);

    return () => {
      document.removeEventListener('mouseup', handleSelectionEnd);
      document.removeEventListener('touchend', handleSelectionEnd);
      document.removeEventListener('selectionchange', handleSelectionChange);
    };
  }, [setSelectedText]);

  // Auto hide tooltip on scroll
  useEffect(() => {
    const hide = () => setTooltipVisible(false);
    window.addEventListener('scroll', hide);
    return () => window.removeEventListener('scroll', hide);
  }, []);

  // ✅ Tooltip click → open chat + auto send
  const openChatWithSelection = () => {
    const textToExplain = selectedText?.trim();
    if (!textToExplain) return;

    setIsOpen(true);
    setTooltipVisible(false);

    setTimeout(() => {
      window.dispatchEvent(
        new CustomEvent('AUTO_EXPLAIN_SELECTED_TEXT', {
          detail: textToExplain,
        })
      );
      setSelectedText('');
    }, 200);
  };

  // Normal chat toggle
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

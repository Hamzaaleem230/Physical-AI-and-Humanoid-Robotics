import React, { useState, useEffect } from 'react';
import ChatButton from './ChatButton';
import ChatWindow from './ChatWindow';
import SelectionTooltip from './SelectionTooltip';

function Chatbot({ selectedText, setSelectedText }) {
  const [isOpen, setIsOpen] = useState(false);
  const [tooltipVisible, setTooltipVisible] = useState(false);
  const [tooltipPos, setTooltipPos] = useState({ top: 0, left: 0 });

  // ---------------------------------------------------
  // ⭐ Show tooltip ONLY after selection completes
  // ---------------------------------------------------
  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      // No valid text → hide tooltip
      if (!text || text.length < 3) {
        setTooltipVisible(false);
        setSelectedText('');
        return;
      }

      setSelectedText(text);

      // Get perfect rect
      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();

      setTooltipPos({
        top: window.scrollY + rect.top - 10, // small distance above text
        left: rect.left + rect.width / 2    // centered
      });

      setTooltipVisible(true);
    };

    document.addEventListener("mouseup", handleMouseUp);

    return () => document.removeEventListener("mouseup", handleMouseUp);
  }, []);

  // ---------------------------------------------------
  // ⭐ Auto hide tooltip on scroll
  // ---------------------------------------------------
  useEffect(() => {
    const hide = () => setTooltipVisible(false);
    window.addEventListener('scroll', hide);
    return () => window.removeEventListener('scroll', hide);
  }, []);

  // ---------------------------------------------------
  // ⭐ When tooltip clicked → open chatbot with auto message
  // ---------------------------------------------------
  const openChatWithSelection = () => {
  if (selectedText?.trim()?.length > 0) {
    setIsOpen(true);
    setTooltipVisible(false);

    // Wait 150ms until chat window fully appears
    setTimeout(() => {
      window.dispatchEvent(
        new CustomEvent("AUTO_EXPLAIN_SELECTED_TEXT", {
          detail: selectedText,
        })
      );
    }, 150);
  }
};


  // ---------------------------------------------------
  // ⭐ Normal button toggle
  // ---------------------------------------------------
  const toggleChat = () => {
    setIsOpen(!isOpen);
    setTooltipVisible(false);
  };

  return (
    <>
      {/* Tooltip above selection */}
      <SelectionTooltip
        visible={tooltipVisible}
        position={tooltipPos}
        onClick={openChatWithSelection}
      />

      {/* Chat button floating bubble */}
      <ChatButton onClick={toggleChat} />

      {/* Main Chatwindow */}
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

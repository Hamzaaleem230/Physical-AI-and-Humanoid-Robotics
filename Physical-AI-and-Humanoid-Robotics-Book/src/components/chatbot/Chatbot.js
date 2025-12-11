import React, { useState } from 'react';
import ChatButton from './ChatButton';
import ChatWindow from './ChatWindow';

function Chatbot({ selectedText, setSelectedText }) {
  // Accept props
  const [isOpen, setIsOpen] = useState(false);
  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <>
      <ChatButton onClick={toggleChat} />
      <ChatWindow
        isOpen={isOpen}
        onClose={toggleChat}
        selectedText={selectedText} // Pass to ChatWindow
        setSelectedText={setSelectedText} // Pass to ChatWindow
      />
    </>
  );
}

export default Chatbot;

import React, { useState, useEffect } from 'react';
import Chatbot from '../components/chatbot/Chatbot'; // Assuming this will be our main chatbot component

function Root({children}) {
  const [selectedText, setSelectedText] = useState('');

  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().length > 0) {
        setSelectedText(selection.toString());
      } else {
        setSelectedText('');
      }
    };

    document.addEventListener('mouseup', handleMouseUp);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, []);

  return (
    <>
      {children}
      <Chatbot selectedText={selectedText} setSelectedText={setSelectedText} />
    </>
  );
}

export default Root;

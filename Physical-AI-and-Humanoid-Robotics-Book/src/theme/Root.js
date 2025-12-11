// src/theme/Root.js
import React, { useEffect, useState } from 'react';
import ReactDOM from 'react-dom/client';
import { AuthProvider } from '../auth/context/AuthProvider';
import AuthNavbarItem from '../auth/components/AuthNavbarItem';
import Chatbot from '../components/chatbot/Chatbot';

function Root({ children }) {

  const [selectedText, setSelectedText] = useState('');
  
  // Inject navbar item ONE TIME, safely
  useEffect(() => {
    const container = document.getElementById('auth-status');
    if (container) {
      const root = ReactDOM.createRoot(container);
      root.render(
        <AuthProvider>
          <AuthNavbarItem />
        </AuthProvider>
      );
    }
  }, []);

  return (
    <AuthProvider>
      {children}

      {/* Chatbot always receives one shared provider */}
      <Chatbot selectedText={selectedText} setSelectedText={setSelectedText} />
    </AuthProvider>
  );
}

export default Root;

import React, { useState, useEffect, useCallback } from 'react';
import { createPortal } from 'react-dom';
import { AuthProvider } from '../auth/context/AuthProvider';
import AuthNavbarItem from './NavbarItem/AuthNavbarItem';
import Chatbot from '../components/chatbot/Chatbot';
import BrowserOnly from '@docusaurus/BrowserOnly';

export default function Root({ children }) {
  const [selectedText, setSelectedText] = useState('');
  const [navbarContainer, setNavbarContainer] = useState(null);

  // 🔹 Function jo check karega ke div mil gaya ya nahi
  const updateContainer = useCallback(() => {
    const el = document.getElementById('auth-status');
    if (el && el !== navbarContainer) {
      setNavbarContainer(el);
    }
  }, [navbarContainer]);

  useEffect(() => {
    // 1. Pehli baar check karein
    updateContainer();

    // 2. MutationObserver setup karein (Ye DOM changes ko monitor karega)
    const observer = new MutationObserver(() => {
      updateContainer();
    });

    observer.observe(document.body, {
      childList: true,
      subtree: true,
    });

    return () => observer.disconnect();
  }, [updateContainer]);

  return (
    <AuthProvider>
      {children}

      {/* 🔹 Portal logic with safety check */}
      <BrowserOnly>
        {() => (
          navbarContainer 
            ? createPortal(<AuthNavbarItem />, navbarContainer) 
            : null
        )}
      </BrowserOnly>

      <Chatbot
        selectedText={selectedText}
        setSelectedText={setSelectedText}
      />
    </AuthProvider>
  );
}
// src/theme/Root.js
import React, { useEffect, useState } from 'react';
import { createPortal } from 'react-dom';
import { AuthProvider } from '../auth/context/AuthProvider';
import AuthNavbarItem from '@theme/NavbarItem/AuthNavbarItem';
import Chatbot from '../components/chatbot/Chatbot';

export default function Root({ children }) {
  const [selectedText, setSelectedText] = useState('');
  const [navbarContainer, setNavbarContainer] = useState(null);

  useEffect(() => {
    const findContainer = () => {
      const el = document.getElementById('auth-status');
      if (el) {
        setNavbarContainer(el);
        return;
      }
      setTimeout(findContainer, 100);
    };
    findContainer();
  }, []);

  return (
    <AuthProvider>
      {children}

      {/* ✅ AUTH NAVBAR */}
      {navbarContainer &&
        createPortal(<AuthNavbarItem />, navbarContainer)}

      {/* ✅ CHATBOT */}
      <Chatbot
        selectedText={selectedText}
        setSelectedText={setSelectedText}
      />
    </AuthProvider>
  );
}

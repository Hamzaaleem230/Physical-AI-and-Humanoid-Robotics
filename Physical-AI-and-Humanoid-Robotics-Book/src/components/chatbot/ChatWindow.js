import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatWindow.module.css';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const SESSION_STORAGE_KEY = 'chatbot_messages';

function ChatWindow({ isOpen, onClose, selectedText, setSelectedText }) {
  // Context Hook ka istemaal karke configuration se URL hasil karna
  const { siteConfig } = useDocusaurusContext();
  const BACKEND_URL = siteConfig.customFields.BACKEND_URL;

  // FIX: Lazy initialization function ka use
  // Yeh function sirf client-side (browser) mein chalta hai, server (SSG) mein nahi.
  const [messages, setMessages] = useState(() => {
    // SSG/Server side check: Agar window ya sessionStorage available nahi hai, toh default value return karo.
    if (typeof window === 'undefined' || typeof window.sessionStorage === 'undefined') {
      return [{ id: 1, text: 'Welcome! How can I help you today?', sender: 'bot' }];
    }

    // Client side (Browser) logic:
    try {
      const savedMessages = window.sessionStorage.getItem(SESSION_STORAGE_KEY);
      return savedMessages
        ? JSON.parse(savedMessages)
        : [{ id: 1, text: 'Welcome! How can I help you today?', sender: 'bot' }];
    } catch (error) {
      console.error('Error reading sessionStorage:', error);
      return [{ id: 1, text: 'Welcome! How can I help you today?', sender: 'bot' }];
    }
  });

  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [apiError, setApiError] = useState(null);
  const messagesEndRef = useRef(null);

  useEffect(() => {
    const handleSelection = () => {
      const text = window.getSelection().toString().trim();
      if (text.length > 5) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, []);

  // FIX: useEffect mein bhi sessionStorage access se pehle window check zaroori hai
  useEffect(() => {
    if (typeof window !== 'undefined' && window.sessionStorage) {
      window.sessionStorage.setItem(SESSION_STORAGE_KEY, JSON.stringify(messages));
    }
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    if (!isOpen) {
      setApiError(null);
    }
  }, [isOpen]);

  useEffect(() => {
    if (isOpen && selectedText) {
      // Logic for handling selected text when the window opens
    }
  }, [isOpen, selectedText]);

  if (!isOpen) {
    return null;
  }

  const handleInputChange = (e) => {
    setInput(e.target.value);
  };

  const sendQueryToBackend = async (userMessageText, context = null) => {
    setIsLoading(true);
    setApiError(null);
    const botResponseId = messages.length + 2;

    setMessages((prevMessages) => [
      ...prevMessages,
      { id: botResponseId, text: '', sender: 'bot', streaming: true },
    ]);

    try {
      const response = await fetch(`${BACKEND_URL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ message: userMessageText, context: context }),
      });

      if (!response.ok) {
        let errorMessage = `Backend API error: ${response.status}`;
        try {
          const errorData = await response.json();
          errorMessage = errorData.detail || errorMessage;
        } catch {
          // If response is not JSON, use default message
        }
        throw new Error(errorMessage);
      }

      const fullResponse = await response.text();

      setMessages((prevMessages) =>
        prevMessages.map((msg) =>
          msg.id === botResponseId ? { ...msg, text: fullResponse, streaming: false } : msg
        )
      );
    } catch (error) {
      console.error('Error fetching from backend:', error);
      setApiError(
        `Could not connect to backend. Please ensure the backend is running. (${error.message})`
      );
      setMessages((prevMessages) =>
        prevMessages.map((msg) =>
          msg.id === botResponseId
            ? { ...msg, text: `Error: ${error.message}`, streaming: false }
            : msg
        )
      );
    } finally {
      setIsLoading(false);
    }
  };

  const handleSendMessage = () => {
    if (input.trim() && !isLoading) {
      const userMessage = { id: messages.length + 1, text: input, sender: 'user' };
      setMessages((prevMessages) => [...prevMessages, userMessage]);
      setInput('');
      sendQueryToBackend(userMessage.text);
    }
  };

  const handleExplainSelectedText = () => {
    if (selectedText.trim() && !isLoading) {
      const userMessage = {
        id: messages.length + 1,
        text: `Explain: "${selectedText}"`,
        sender: 'user',
      };
      setMessages((prevMessages) => [...prevMessages, userMessage]);
      setInput('');
      sendQueryToBackend(`Explain: "${selectedText}"`, selectedText);
      setSelectedText('');
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && input.trim() && !isLoading) {
      handleSendMessage();
    }
  };

  return (
    <div className={styles.chatWindow}>
      <div className={styles.header}>
        <h3>Chat with our AI</h3>
        <button onClick={onClose} className={styles.closeButton}>
          X
        </button>
      </div>
      <div className={styles.messages}>
        {apiError && (
          <div className={styles.apiError}>
            <p>{apiError}</p>
          </div>
        )}
        {selectedText && (
          <div className={styles.selectedTextPrompt}>
            <p>Selected text:</p>
            <blockquote>{selectedText}</blockquote>
            <button onClick={handleExplainSelectedText} disabled={isLoading}>
              Explain Selected Text
            </button>
            <button
              onClick={() => setSelectedText('')}
              disabled={isLoading}
              className={styles.clearSelectedTextButton}
            >
              Clear
            </button>
          </div>
        )}
        {messages.map((message) => (
          <div key={message.id} className={`${styles.message} ${styles[message.sender]}`}>
            {message.text}
            {message.streaming && <span className={styles.streamingIndicator}>Thinking...</span>}
          </div>
        ))}
        {isLoading && messages[messages.length - 1]?.sender !== 'bot' && (
          <div className={`${styles.message} ${styles.bot}`}>
            Thinking...
            <span className={styles.streamingIndicator}>Thinking...</span>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>
      <div className={styles.inputArea}>
        <input
          type="text"
          placeholder="Type your message..."
          value={input}
          onChange={handleInputChange}
          onKeyPress={handleKeyPress}
          disabled={isLoading}
        />
        <button onClick={handleSendMessage} disabled={!input.trim() || isLoading}>
          Send
        </button>
      </div>
    </div>
  );
}

export default ChatWindow;

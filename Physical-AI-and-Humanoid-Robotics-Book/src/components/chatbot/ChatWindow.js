import React, { useState, useRef, useEffect } from "react";
import styles from "./ChatWindow.module.css";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";

const SESSION_STORAGE_KEY = "chatbot_messages";

function ChatWindow({ isOpen, onClose, selectedText, setSelectedText }) {
  const { siteConfig } = useDocusaurusContext();
  const BACKEND_URL = siteConfig.customFields.BACKEND_URL;

  const [messages, setMessages] = useState(() => {
    if (typeof window === "undefined" || typeof window.sessionStorage === "undefined") {
      return [{ id: 1, text: "Welcome! How can I help you today?", sender: "bot" }];
    }

    try {
      const saved = window.sessionStorage.getItem(SESSION_STORAGE_KEY);
      return saved
        ? JSON.parse(saved)
        : [{ id: 1, text: "Welcome! How can I help you today?", sender: "bot" }];
    } catch (error) {
      return [{ id: 1, text: "Welcome! How can I help you today?", sender: "bot" }];
    }
  });

  const [input, setInput] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [apiError, setApiError] = useState(null);
  const messagesEndRef = useRef(null);

  useEffect(() => {
    if (typeof window !== "undefined") {
      window.sessionStorage.setItem(SESSION_STORAGE_KEY, JSON.stringify(messages));
    }
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // ⭐⭐⭐ AUTO EXPLAIN USE EFFECT — INSERTED EXACTLY HERE ⭐⭐⭐
  useEffect(() => {
    const handler = (event) => {
      const text = event.detail;

      if (!text || text.trim().length === 0) return;

      // Add user message
      const userMessage = {
        id: messages.length + 1,
        text: `Explain: "${text}"`,
        sender: "user",
      };
      setMessages((prev) => [...prev, userMessage]);

      // Auto-send to backend
      sendQueryToBackend(`Explain: "${text}"`, text);

      // Clear selection
      setSelectedText("");
    };

    window.addEventListener("AUTO_EXPLAIN_SELECTED_TEXT", handler);
    return () => window.removeEventListener("AUTO_EXPLAIN_SELECTED_TEXT", handler);
  }, [messages, setSelectedText]);
  // ⭐⭐⭐ END OF AUTO EXPLAIN USE EFFECT ⭐⭐⭐

  useEffect(() => {
    if (!isOpen) setApiError(null);
  }, [isOpen]);

  const sendQueryToBackend = async (userMessageText, context = null) => {
    setIsLoading(true);
    setApiError(null);

    const botResponseId = messages.length + 2;

    setMessages((prev) => [
      ...prev,
      { id: botResponseId, text: "", sender: "bot", streaming: true },
    ]);

    try {
      const response = await fetch(`${BACKEND_URL}/chat`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ message: userMessageText, context }),
      });

      if (!response.ok) {
        let errorMessage = `Backend error: ${response.status}`;
        try {
          const errorJSON = await response.json();
          errorMessage = errorJSON.detail || errorMessage;
        } catch {}
        throw new Error(errorMessage);
      }

      const fullResponse = await response.text();

      setMessages((prev) =>
        prev.map((msg) =>
          msg.id === botResponseId
            ? { ...msg, text: fullResponse, streaming: false }
            : msg
        )
      );
    } catch (error) {
      setApiError(error.message);
      setMessages((prev) =>
        prev.map((msg) =>
          msg.id === botResponseId
            ? { ...msg, text: "Error: " + error.message, streaming: false }
            : msg
        )
      );
    } finally {
      setIsLoading(false);
    }
  };

  const handleSendMessage = () => {
    if (input.trim() && !isLoading) {
      const userMessage = {
        id: messages.length + 1,
        text: input,
        sender: "user",
      };

      setMessages((prev) => [...prev, userMessage]);
      sendQueryToBackend(input);
      setInput("");
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === "Enter" && input.trim() && !isLoading) {
      handleSendMessage();
    }
  };

  if (!isOpen) return null;

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

            <button
              onClick={() =>
                sendQueryToBackend(`Explain: "${selectedText}"`, selectedText)
              }
              disabled={isLoading}
            >
              Explain Selected Text
            </button>

            <button
              onClick={() => setSelectedText("")}
              disabled={isLoading}
              className={styles.clearSelectedTextButton}
            >
              Clear
            </button>
          </div>
        )}

        {messages.map((msg) => (
          <div key={msg.id} className={`${styles.message} ${styles[msg.sender]}`}>
            {msg.text}
            {msg.streaming && (
              <span className={styles.streamingIndicator}>Thinking...</span>
            )}
          </div>
        ))}

        <div ref={messagesEndRef}></div>
      </div>

      <div className={styles.inputArea}>
        <input
          type="text"
          placeholder="Type your message..."
          value={input}
          onChange={(e) => setInput(e.target.value)}
          disabled={isLoading}
          onKeyPress={handleKeyPress}
        />
        <button onClick={handleSendMessage} disabled={!input.trim() || isLoading}>
          Send
        </button>
      </div>
    </div>
  );
}

export default ChatWindow;
